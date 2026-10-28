"""Web server (yanshee_sim/web/server.py): every endpoint in sim mode, robot and both modes against
tests/fake_robot.py, the WebSocket, MJPEG streams (sim cameras and the proxied robot camera), bad input,
path traversal, and a real uvicorn server with open streams shut down cleanly.
Run: python3 tests/test_server.py   (headless: MUJOCO_GL=egl is set when there is no display)
"""
import http.client
import json
import os
import shutil
import signal
import socket
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")
import fake_robot  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402
from starlette.websockets import WebSocketDisconnect  # noqa: E402

from yanshee_sim import rest  # noqa: E402
from yanshee_sim.controller import Controller  # noqa: E402
from yanshee_sim.servos import RESET_POSE  # noqa: E402
from yanshee_sim.web import server as web  # noqa: E402

FAKE_PORTS = range(9200, 9250)  # a port with nothing listening
AUX_PORTS = range(9191, 9200)  # fake robot camera, uvicorn
CAM_MARK = b"FAKE-ROBOT-CAM"
_S = {}


# -- fixtures ---------------------------------------------------------------------------------------
def setup():
    """Shared controller (sim mode) + TestClient on an app with a temporary static folder."""
    if "client" not in _S:
        _S["dir"] = tempfile.mkdtemp(prefix="yanshee_srv_")
        _S["motions"] = os.path.join(_S["dir"], "motions")
        _S["static"] = os.path.join(_S["dir"], "static")
        os.makedirs(_S["motions"])
        os.makedirs(_S["static"])
        shutil.copy(os.path.join(ROOT, "motions", "wave_right.json"), _S["motions"])
        with open(os.path.join(_S["static"], "index.html"), "w") as f:
            f.write("<!doctype html><title>test gui</title>")
        with open(os.path.join(_S["static"], "app.js"), "w") as f:
            f.write("console.log('gui');")
        with open(os.path.join(_S["dir"], "secret.txt"), "w") as f:
            f.write("outside the static folder")
        _S["ctl"] = Controller(motions_dir=_S["motions"])
        _S["app"] = web.create_app(_S["ctl"], static_dir=_S["static"])
        _S["client"] = TestClient(_S["app"]).__enter__()
    c = _S["ctl"]
    if c.robot is not None:
        c.disconnect()
    c.stop_all()
    if c._fallen:
        c.reset()
    c.charge()
    return _S["client"], c


def free_port(ports):
    for p in ports:
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(("127.0.0.1", p))
                return p
            except OSError:
                continue
    raise RuntimeError(f"no free port in {ports}")


def start_fake(**options):
    """Fake robot on an OS-assigned port (fixed ranges collide with parallel test runs and manual fakes)."""
    options.setdefault("motion_seconds", 1.5)
    options.setdefault("vision_seconds", 0.3)
    return fake_robot.start(0, **options)


def wait_until(cond, timeout=5.0, step=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(step)
    return cond()


def logged(srv, method, path):
    return [e for e in list(srv.state.log) if e[0] == method and e[1] == path]


def marked_jpeg():
    """A test-card JPEG with a COM segment saying it came from the fake robot camera."""
    jpeg = fake_robot.make_jpeg("64x48", "cam")
    return jpeg[:2] + b"\xff\xfe" + (len(CAM_MARK) + 2).to_bytes(2, "big") + CAM_MARK + jpeg[2:]


class _CamHandler(BaseHTTPRequestHandler):
    """The robot's http://IP:8000/stream.mjpg (picamera style); ?nolen=1 leaves out Content-Length."""

    def log_message(self, *a):
        pass

    def do_GET(self):
        if not self.path.startswith("/stream.mjpg"):
            self.send_error(404)
            return
        nolen = "nolen=1" in self.path
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        jpeg = marked_jpeg()
        try:
            while not self.server.stopping:
                head = b"--FRAME\r\nContent-Type: image/jpeg\r\n"
                head += b"\r\n" if nolen else b"Content-Length: %d\r\n\r\n" % len(jpeg)
                self.wfile.write(head + jpeg + b"\r\n")
                time.sleep(0.05)
        except OSError:
            pass


def start_cam():
    srv = ThreadingHTTPServer(("127.0.0.1", free_port(AUX_PORTS)), _CamHandler)
    srv.daemon_threads, srv.stopping = True, False
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    return srv


def stop_cam(srv):
    srv.stopping = True
    srv.shutdown()
    srv.server_close()


def parse_mjpeg(body, boundary=b"frame"):
    """[jpeg bytes] from a multipart/x-mixed-replace body, checking each part's headers."""
    frames, sep = [], b"--" + boundary + b"\r\n"
    assert body.startswith(sep), body[:40]
    for part in body.split(sep)[1:]:
        head, _, rest_ = part.partition(b"\r\n\r\n")
        headers = dict(line.split(b": ", 1) for line in head.split(b"\r\n"))
        assert headers[b"Content-Type"] == b"image/jpeg", headers
        n = int(headers[b"Content-Length"])
        jpeg = rest_[:n]
        assert rest_[n:n + 2] == b"\r\n" and jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"
        frames.append(jpeg)
    return frames


def ok(r, status=200):
    assert r.status_code == status, (r.status_code, r.text[:300])
    data = r.json()
    assert data["ok"] is True, data
    return data


def refused(r, status=200):
    assert r.status_code == status, (r.status_code, r.text[:300])
    data = r.json()
    assert data["ok"] is False and data["error"], data
    return data


# -- sim mode -------------------------------------------------------------------------------------------
def test_state_and_static():
    client, c = setup()
    st = ok(client.get("/api/state"))
    for key in ("mode", "robot", "sim", "servos", "activity", "leds", "volume", "language", "fall_management",
                "limits", "reset_pose", "events"):
        assert key in st, key
    assert st["mode"] == "sim" and st["servos"]["robot"] is None and st["limits"]["NeckLR"] == [15, 165]
    r = client.get("/")
    assert r.status_code == 200 and "test gui" in r.text and r.headers["content-type"].startswith("text/html")
    assert client.get("/app.js").status_code == 200
    refused(client.get("/api/nothing"), 404)
    refused(client.post("/api/state"), 405)
    assert client.get("/api/docs").status_code == 200


def test_path_traversal():
    client, c = setup()
    for path in ("/..%2fsecret.txt", "/%2e%2e/secret.txt", "/%2e%2e%2fsecret.txt", "/static/../secret.txt",
                 "/..%5csecret.txt", "/%2e%2e/%2e%2e/%2e%2e/etc/passwd"):
        r = client.get(path)
        assert r.status_code in (400, 404) and "outside the static" not in r.text, (path, r.status_code)
    before = set(os.listdir(_S["dir"]))
    motion = {"frames": [{"angles": {"NeckLR": 60}, "runtime": 500}]}
    for name in ("..", "...", "a.b", "x.json", "~root", "%2e%2e", "..%2fevil", "a%2fb", "a%5cb", "a b", "é",
                 "x" * 65):
        for method in ("GET", "PUT", "DELETE"):
            r = client.request(method, f"/api/motions/{name}", json=motion if method == "PUT" else None)
            assert r.status_code in (400, 404, 405) and r.json()["ok"] is False, (method, name, r.status_code)
        r = client.post(f"/api/motions/{name}/play", json={})
        assert r.status_code in (400, 404, 405) and r.json()["ok"] is False, (name, r.status_code)
    refused(client.post("/api/motions/check", json={"name": "../wave_right"}), 400)
    assert set(os.listdir(_S["dir"])) == before
    assert sorted(os.listdir(_S["motions"])) == ["wave_right.json"]


def test_bad_input_rejected():
    client, c = setup()
    cases = [("/api/mode", {"mode": "xx"}), ("/api/mode", {}), ("/api/connect", {"ip": ""}),
             ("/api/connect", {"ip": "a b/c?d"}), ("/api/connect", {"ip": 5}),
             ("/api/servos", {"angles": {"NeckLR": "60"}}), ("/api/servos", {"angles": {}}),
             ("/api/servos", {"angles": {"NeckLR": True}}), ("/api/servos", {"angles": {"NeckLR": 60}, "runtime": -1}),
             ("/api/servos", {"angles": [60]}), ("/api/servos/mode", {"mode": "limp"}),
             ("/api/servos/mode", {"mode": "program", "servos": "NeckLR"}),
             ("/api/action", {"name": ""}), ("/api/action", {"name": "wave", "speed": "warp"}),
             ("/api/action", {"name": "wave", "repeat": 0}), ("/api/action", {"name": "wave", "version": "v3"}),
             ("/api/gait", {"speed_v": 9}), ("/api/gait", {"speed_v": 1.5}), ("/api/gait", {"steps": -1}),
             ("/api/gait", {"period": 6}), ("/api/gait", {"wave": "yes"}), ("/api/music", {}),
             ("/api/record", {"runtime": 50}), ("/api/record", {"hold": -1}), ("/api/led", {"type": "x"}),
             ("/api/volume", {"volume": 101}), ("/api/volume", {"volume": "50"}), ("/api/language", {"language": "fr"}),
             ("/api/fall", {"enable": "yes"}), ("/api/tts", {"text": ""}), ("/api/tts", {"text": "x" * 501}),
             ("/api/vision", {"task": "bogus"}), ("/api/vision", {"task": "color", "timeout": 0}),
             ("/api/face_entry", {"name": "a/b"}), ("/api/motions/play", {"motion": "wave_right"}),
             ("/api/motions/wave_right/play", {"speed": 0}), ("/api/motions/check", [1, 2])]
    for path, body in cases:
        refused(client.post(path, json=body), 400)
    headers = {"Content-Type": "application/json"}
    for raw in (b"{not json", b'{"angles": {"NeckLR": NaN}}', b'{"angles": {"NeckLR": Infinity}}', b"[]", b""):
        refused(client.post("/api/servos", content=raw, headers=headers), 400)
    refused(client.post("/api/volume", content=b"volume=5", headers={"Content-Type": "text/plain"}), 400)
    refused(client.put("/api/motions/big", content=b"{" + b" " * (web.MAX_BODY + 10) + b"}", headers=headers), 413)
    refused(client.post("/api/stop", headers={"Origin": "http://evil.example"}), 403)
    ok(client.post("/api/stop", headers={"Origin": "http://testserver"}))
    refused(client.get("/api/stream/back.mjpg"), 400)
    refused(client.get("/api/stream/front.mjpg?frames=-1"), 400)
    refused(client.get("/api/stream/front.mjpg?source=cloud"), 400)
    st = ok(client.get("/api/state"))
    assert st["mode"] == "sim" and st["volume"] in range(0, 101)


def test_sim_endpoints():
    client, c = setup()
    srv = ok(client.get("/api/servos"))
    assert set(srv["sim"]) == set(RESET_POSE) and srv["robot"] is None
    r = ok(client.post("/api/servos", json={"angles": {"NeckLR": 60, "LeftAnkleUD": 130}, "runtime": 400}))
    assert r["sent_to"] == ["sim"] and r["warnings"]
    assert wait_until(lambda: abs(client.get("/api/servos").json()["sim"]["NeckLR"] - 60) <= 2, 3)
    refused(client.post("/api/servos", json={"angles": {"Nope": 3}}))
    assert ok(client.post("/api/servos/mode", json={"mode": "program", "servos": ["NeckLR"]}))["names"] == ["NeckLR"]
    ok(client.post("/api/servos/mode", json={"mode": "work"}))
    ok(client.post("/api/reset"))

    acts = ok(client.get("/api/actions"))["actions"]
    by_name = {a["name"]: a for a in acts}
    assert by_name["wave"]["sim"] and by_name["wave_right"]["category"] == "user"
    assert {"gestures", "locomotion", "dances", "user"} <= {a["category"] for a in acts}
    r = ok(client.post("/api/action", json={"name": "wave", "direction": "right", "speed": "fast"}))
    assert r["target"] == "sim"
    assert client.get("/api/state").json()["activity"]["kind"] == "action"
    refused(client.post("/api/action", json={"name": "wave"}))  # needs a direction: HTTP 200, ok false
    ok(client.post("/api/stop"))
    assert wait_until(lambda: client.get("/api/state").json()["activity"]["kind"] == "idle", 6)
    ok(client.post("/api/action", json={"name": "bow"}))
    ok(client.post("/api/stop"))

    r = ok(client.post("/api/gait", json={"speed_v": 2, "speed_h": 0, "steps": 0, "period": 1}))
    assert r["updated"] is False
    assert ok(client.post("/api/gait", json={"speed_v": 3, "speed_h": 0}))["updated"] is True
    ok(client.post("/api/gait/stop"))
    assert wait_until(lambda: client.get("/api/state").json()["activity"]["kind"] == "idle", 6)
    ok(client.post("/api/gait", json={"speed_v": 1, "period": 0}))  # period 0 stops

    music = ok(client.get("/api/music"))
    assert music["source"] == "sim" and music["music"]
    assert ok(client.post("/api/music", json={"name": music["music"][0]}))["simulated"] is True
    assert client.get("/api/state").json()["activity"]["kind"] == "music"
    ok(client.post("/api/music/stop"))

    files = ok(client.get("/api/motions"))["motions"]
    assert [m["name"] for m in files] == ["wave_right"]
    mo = ok(client.get("/api/motions/wave_right"))["motion"]
    assert mo["frames"]
    refused(client.get("/api/motions/nothing_here"))
    rep = ok(client.post("/api/motions/check", json=mo))
    assert rep["safe"] and rep["duration_s"] > 0 and rep["start"] == "sim"
    assert ok(client.post("/api/motions/check?speed=2", json={"name": "wave_right"}))["duration_s"] < rep["duration_s"]
    bad = ok(client.post("/api/motions/check", json={"frames": [{"angles": {"NeckLR": 500}, "runtime": 10}]}))
    assert not bad["safe"] and bad["errors"]
    saved = ok(client.put("/api/motions/my_pose-1", json={"frames": [{"angles": {"NeckLR": 70}, "runtime": 600}]}))
    assert saved["name"] == "my_pose-1" and os.path.isfile(os.path.join(_S["motions"], "my_pose-1.json"))
    refused(client.put("/api/motions/broken", json={"frames": "nope"}))
    assert ok(client.post("/api/motions/my_pose-1/play", json={"speed": 2}))["target"] == "sim"
    ok(client.post("/api/stop"))
    assert ok(client.post("/api/motions/my_pose-1/play"))["target"] == "sim"  # no body: speed 1
    ok(client.post("/api/stop"))
    unsaved = {"frames": [{"angles": {"NeckLR": 80}, "runtime": 400}]}
    r = ok(client.post("/api/motions/play", json={"motion": unsaved, "speed": 1}))
    assert r["target"] == "sim"
    ok(client.post("/api/stop"))
    ok(client.delete("/api/motions/my_pose-1"))
    refused(client.delete("/api/motions/my_pose-1"))
    frame = ok(client.post("/api/record", json={"runtime": 700, "hold": 200}))
    assert frame["source"] == "sim" and frame["frame"]["runtime"] == 700 and frame["frame"]["hold"] == 200
    assert ok(client.post("/api/record"))["frame"]["runtime"] == 500

    ok(client.post("/api/led", json={"type": "button", "color": "red", "mode": "blink"}))
    refused(client.post("/api/led", json={"type": "camera", "color": "white", "mode": "on"}))
    ok(client.post("/api/volume", json={"volume": 35}))
    ok(client.post("/api/language", json={"language": "zh"}))
    ok(client.post("/api/fall", json={"enable": True}))
    st = ok(client.get("/api/state"))
    assert st["volume"] == 35 and st["language"] == "zh" and st["fall_management"] is True
    assert st["leds"]["button"] == {"color": "red", "mode": "blink"}
    ok(client.post("/api/language", json={"language": "en"}))
    ok(client.post("/api/fall", json={"enable": False}))
    ok(client.post("/api/led", json={"type": "button", "color": "blue", "mode": "breath"}))

    info = ok(client.get("/api/info"))
    assert info["mode"] == "sim" and info["robot"] is None and "versions" in info["sim"]
    sens = ok(client.get("/api/sensors"))
    assert "euler-x" in sens["sim"]["gyro"] and sens["robot"] == "not connected"
    c.battery.percent = 50.0
    assert ok(client.post("/api/sim/charge"))["battery"] == 100.0

    assert ok(client.post("/api/tts", json={"text": " hello "}))["speak_in_browser"] == "hello"
    col = ok(client.post("/api/vision", json={"task": "color", "timeout": 5}))
    assert col["target"] == "sim" and "color" in col["data"]
    refused(client.post("/api/vision", json={"task": "face:age"}))
    refused(client.post("/api/face_entry", json={"name": "Visitor"}))
    r = client.get("/api/photo")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg" and r.content[:2] == b"\xff\xd8"
    assert "no-store" in r.headers["cache-control"]

    ok(client.post("/api/disconnect"))
    t = time.monotonic()
    refused(client.post("/api/connect", json={"ip": f"127.0.0.1:{free_port(FAKE_PORTS)}"}))  # nothing listening
    assert time.monotonic() - t < 15
    refused(client.post("/api/mode", json={"mode": "robot"}))
    assert ok(client.post("/api/mode", json={"mode": "sim"}))["mode"] == "sim"


def test_mjpeg_sim_cameras():
    client, c = setup()
    for cam in ("front", "side", "free", "head"):
        r = client.get(f"/api/stream/{cam}.mjpg?frames=2")
        assert r.status_code == 200, (cam, r.text[:200])
        assert r.headers["content-type"] == "multipart/x-mixed-replace; boundary=frame", r.headers
        assert "no-store" in r.headers["cache-control"]
        frames = parse_mjpeg(r.content)
        assert len(frames) == 2 and all(len(f) > 2000 for f in frames), (cam, [len(f) for f in frames])
        assert CAM_MARK not in frames[0]
    # head in sim mode never proxies, even when asked to: no robot connected
    frames = parse_mjpeg(client.get("/api/stream/head.mjpg?frames=1&source=robot").content)
    assert CAM_MARK not in frames[0]


def test_websocket():
    client, c = setup()
    with client.websocket_connect("/ws") as ws:
        first = ws.receive_json()
        assert first["type"] == "state" and first["mode"] == "sim" and "servos" in first and "events" in first
        ok(client.post("/api/volume", json={"volume": 44}))
        t0, states, events = time.monotonic(), 0, []
        while time.monotonic() - t0 < 1.5:
            msg = ws.receive_json()
            if msg["type"] == "state":
                states += 1
            else:
                assert msg["type"] == "event" and {"id", "t", "level", "text"} <= set(msg)
                events.append(msg)
        assert 4 <= states <= 12, states  # about 5 Hz
        assert any("volume 44" in e["text"] for e in events), events
        assert [e["id"] for e in events] == sorted({e["id"] for e in events})
        ws.send_text("ping")
        for _ in range(20):
            if ws.receive_json()["type"] == "pong":
                break
        else:
            raise AssertionError("no pong")
    try:
        with client.websocket_connect("/ws", headers={"Origin": "http://evil.example"}) as ws:
            ws.receive_json()
    except WebSocketDisconnect as e:
        assert e.code == 1008
    else:
        raise AssertionError("cross-origin WebSocket accepted")


# -- robot and both modes (tests/fake_robot.py) -------------------------------------------------------------
def test_robot_and_both_modes():
    client, c = setup()
    fake, cam = start_fake(), start_cam()
    old_port = rest.STREAM_PORT
    rest.STREAM_PORT = cam.server_address[1]  # the robot's camera stream is http://IP:STREAM_PORT/stream.mjpg
    try:
        r = ok(client.post("/api/connect", json={"ip": fake.address}))
        assert r["ip"] == fake.address and r["battery"] == 90
        st = ok(client.get("/api/state"))
        assert st["robot"]["connected"] and st["mode"] == "sim"
        assert ok(client.post("/api/mode", json={"mode": "robot"}))["mode"] == "robot"
        assert wait_until(lambda: client.get("/api/state").json()["servos"]["robot"] is not None, 3)

        n = len(logged(fake, "PUT", "servos/angles"))
        r = ok(client.post("/api/servos", json={"angles": {"NeckLR": 70}, "runtime": 300}))
        assert r["sent_to"] == ["robot"] and r["runtime"] >= 300
        assert len(logged(fake, "PUT", "servos/angles")) == n + 1
        r = refused(client.post("/api/servos", json={"angles": {"RightHipLR": 120, "LeftHipLR": 60}}))
        assert r["blocked"]  # self-collision: refused before reaching the robot
        assert len(logged(fake, "PUT", "servos/angles")) == n + 1
        assert ok(client.get("/api/servos"))["robot"]
        assert ok(client.post("/api/record"))["source"] == "robot"
        ok(client.post("/api/servos/mode", json={"mode": "program", "servos": ["LeftShoulderFlex"]}))
        ok(client.post("/api/servos/mode", json={"mode": "work"}))
        assert logged(fake, "PUT", "servos/mode")

        acts = ok(client.get("/api/actions"))["actions"]
        assert any(a["source"] == "robot" for a in acts)
        r = ok(client.post("/api/action", json={"name": "wave", "direction": "both", "speed": "fast"}))
        assert r["target"] == "robot" and logged(fake, "PUT", "motions")
        ok(client.post("/api/stop"))
        stops = [e[3] for e in logged(fake, "PUT", "motions") if e[3].get("operation") == "stop"]
        assert {s.get("version", "v1") for s in stops} >= {"v1", "v2"}, stops
        assert logged(fake, "DELETE", "motions/gait") and logged(fake, "DELETE", "voice/tts")

        ok(client.post("/api/gait", json={"speed_v": 2, "speed_h": 0}))
        assert logged(fake, "PUT", "motions/gait")
        ok(client.post("/api/gait/stop"))

        music = ok(client.get("/api/music"))
        assert music["source"] == "robot" and music["music"]
        assert ok(client.post("/api/music", json={"name": music["music"][0]}))["simulated"] is False
        ok(client.post("/api/music/stop"))

        ok(client.post("/api/volume", json={"volume": 30}))
        ok(client.post("/api/led", json={"type": "camera", "color": "green", "mode": "on"}))
        ok(client.post("/api/language", json={"language": "en"}))
        ok(client.post("/api/fall", json={"enable": False}))
        assert logged(fake, "PUT", "devices/volume")[-1][3]["volume"] == 30
        info = ok(client.get("/api/info"))
        assert info["robot"]["ip"] == fake.address and info["robot"]["percent"] == 90
        sens = ok(client.get("/api/sensors"))
        assert {"infrared", "ultrasonic"} <= set(sens["robot"]["addons"])
        assert ok(client.post("/api/tts", json={"text": "hello"}))["target"] == "robot"
        assert logged(fake, "PUT", "voice/tts")
        assert ok(client.post("/api/vision", json={"task": "face:age", "timeout": 5}))["data"]
        assert ok(client.post("/api/vision", json={"task": "qr", "timeout": 5}))["target"] == "robot"
        ok(client.post("/api/face_entry", json={"name": "Visitor"}))
        r = client.get("/api/photo")
        assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg" and r.content[:2] == b"\xff\xd8"

        # head proxies the robot camera in robot mode; the other cameras show the twin
        r = client.get("/api/stream/head.mjpg?frames=3")
        assert r.headers["content-type"] == web.MJPEG_TYPE
        frames = parse_mjpeg(r.content)
        assert len(frames) == 3 and all(CAM_MARK in f for f in frames)
        assert logged(fake, "POST", "visions/streams")
        assert CAM_MARK not in parse_mjpeg(client.get("/api/stream/front.mjpg?frames=1").content)[0]
        assert CAM_MARK not in parse_mjpeg(client.get("/api/stream/head.mjpg?frames=1&source=sim").content)[0]
        # one camera: a vision task first closes the proxied stream; the last viewer leaving closes it too
        camera = _S["app"].state.camera
        assert camera.opened and camera.users == 0
        ok(client.post("/api/vision", json={"task": "face:quantity", "timeout": 5}))
        calls = [(e[0], e[1]) for e in list(fake.state.log)]
        last = {k: max(i for i, x in enumerate(calls) if x == k) for k in (("DELETE", "visions/streams"),
                                                                           ("PUT", "visions"))}
        assert last[("DELETE", "visions/streams")] < last[("PUT", "visions")], calls[-12:]
        assert not camera.opened
        n = len(logged(fake, "DELETE", "visions/streams"))
        assert CAM_MARK in parse_mjpeg(client.get("/api/stream/head.mjpg?frames=1").content)[0] and camera.opened
        assert wait_until(lambda: len(logged(fake, "DELETE", "visions/streams")) == n + 1, 5) and not camera.opened

        # both: commands go to the sim and the robot
        assert ok(client.post("/api/mode", json={"mode": "both"}))["mode"] == "both"
        r = ok(client.post("/api/servos", json={"angles": {"NeckLR": 80}, "runtime": 400}))
        assert r["sent_to"] == ["robot", "sim"]
        assert ok(client.post("/api/action", json={"name": "raise", "direction": "left"}))["target"] == "both"
        ok(client.post("/api/stop"))
        assert ok(client.post("/api/motions/wave_right/play", json={"speed": 1}))["target"] == "both"
        ok(client.post("/api/stop"))
        assert CAM_MARK not in parse_mjpeg(client.get("/api/stream/head.mjpg?frames=1").content)[0]
        assert CAM_MARK in parse_mjpeg(client.get("/api/stream/head.mjpg?frames=1&source=robot").content)[0]

        # the robot camera is unreachable: the head stream falls back to the twin and says so
        rest.STREAM_PORT = free_port(AUX_PORTS)
        ok(client.post("/api/mode", json={"mode": "robot"}))
        last = c.events[-1]["id"]
        frames = parse_mjpeg(client.get("/api/stream/head.mjpg?frames=2").content)
        assert len(frames) == 2 and CAM_MARK not in frames[0]
        assert any("camera stream" in e["text"] for e in c.events_since(last))

        # the robot dies mid-session: errors come back as ok false, the server keeps going
        fake.stop()
        t = time.monotonic()
        refused(client.post("/api/action", json={"name": "bow"}))
        refused(client.post("/api/tts", json={"text": "anyone?"}))
        assert time.monotonic() - t < 20
        ok(client.get("/api/state"))
        ok(client.post("/api/stop"))
        assert ok(client.post("/api/disconnect"))
        assert ok(client.get("/api/state"))["mode"] == "sim"
    finally:
        rest.STREAM_PORT = old_port
        if c.robot is not None:
            c.disconnect()
        stop_cam(cam)
        fake.stop()


def test_low_battery_and_refusals():
    client, c = setup()
    fake = start_fake(battery=20)
    try:
        ok(client.post("/api/connect", json={"ip": fake.address}))
        ok(client.post("/api/mode", json={"mode": "robot"}))
        r = refused(client.post("/api/action", json={"name": "wave", "direction": "left"}))
        assert "battery" in r["error"]
        ok(client.post("/api/servos", json={"angles": {"NeckLR": 80}, "runtime": 600}))  # servos still work
        ok(client.post("/api/volume", json={"volume": 20}))
    finally:
        c.disconnect()
        fake.stop()


def test_chunked_body_limit():
    """The 2 MB limit also holds for chunked bodies (no Content-Length)."""
    client, c = setup()
    big = json.dumps({"frames": [{"angles": {"NeckLR": 90}, "runtime": 500}], "pad": "y" * (web.MAX_BODY + 10)})

    def chunks(data, n=65536):
        for i in range(0, len(data), n):
            yield data[i:i + n].encode()
    headers = {"Content-Type": "application/json"}
    r = refused(client.put("/api/motions/zz_big", content=chunks(big), headers=headers), 413)
    assert "over" in r["error"] and not os.path.exists(os.path.join(_S["motions"], "zz_big.json"))
    small = json.dumps({"frames": [{"angles": {"NeckLR": 80}, "runtime": 500}]})
    ok(client.put("/api/motions/zz_small", content=chunks(small, 7), headers=headers))
    ok(client.delete("/api/motions/zz_small"))


def test_gait_dead_man():
    """/api/gait with steps 0 is a lease: keepalives hold it, silence ends it, and so does the last page leaving."""
    client, c = setup()
    ok(client.post("/api/gait", json={"speed_v": 2, "lease": 0.6}))
    for _ in range(4):
        time.sleep(0.3)
        assert ok(client.post("/api/gait/keepalive"))["walking"]
    assert ok(client.get("/api/state"))["activity"]["kind"] == "gait"
    assert wait_until(lambda: client.get("/api/state").json()["activity"]["kind"] == "idle", 4)
    assert ok(client.post("/api/gait/keepalive"))["walking"] is False
    refused(client.post("/api/gait", json={"speed_v": 2, "lease": 0}), 400)
    ok(client.post("/api/gait", json={"speed_v": 2, "steps": 4}))  # a finite walk needs no keepalive
    time.sleep(1.2)
    assert client.get("/api/state").json()["activity"]["kind"] == "gait"
    ok(client.post("/api/gait/stop"))
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ok(client.post("/api/gait", json={"speed_v": 2, "lease": 10}))
        assert client.get("/api/state").json()["activity"]["kind"] == "gait"
    assert wait_until(lambda: client.get("/api/state").json()["activity"]["kind"] == "idle", 4)
    assert any("last controller page" in e["text"] for e in c.events_since(0))
    c.reset()


def test_getup_route():
    client, c = setup()
    assert ok(client.post("/api/getup"))["standing"] is True  # already standing
    refused(client.post("/api/tipover", json={"direction": "up"}), 400)
    assert ok(client.post("/api/tipover", json={"direction": "front"}))["direction"] == "front"
    assert wait_until(lambda: client.get("/api/state").json()["sim"]["fallen"], 8)
    assert wait_until(lambda: client.get("/api/state").json()["activity"]["kind"] == "idle", 8)
    assert ok(client.post("/api/getup"))["posture"] == "front"
    assert wait_until(lambda: not client.get("/api/state").json()["sim"]["fallen"], 20)
    c.reset()
    assert "balance_margin_cm" in client.get("/api/state").json()["sim"]


def test_gait_turn_param():
    client, c = setup()
    refused(client.post("/api/gait", json={"turn": 7}), 400)
    ok(client.post("/api/gait", json={"speed_v": 0, "turn": -3, "lease": 2}))
    assert wait_until(lambda: "clockwise" in client.get("/api/state").json()["activity"]["detail"], 5)
    ok(client.post("/api/gait/stop"))
    assert wait_until(lambda: client.get("/api/state").json()["activity"]["kind"] == "idle", 8)
    c.reset()


def test_robot_extras_pause_tts_stop_photo_resolution():
    client, c = setup()
    fake = start_fake(motion_seconds=20)
    try:
        ok(client.post("/api/connect", json={"ip": fake.address}))
        ok(client.post("/api/mode", json={"mode": "robot"}))
        ok(client.post("/api/action", json={"name": "WakaWaka"}))
        ok(client.post("/api/action/pause"))
        assert fake.state.motion["status"] == "pause"
        assert client.get("/api/state").json()["activity"]["detail"] == "paused"
        ok(client.post("/api/action/resume"))
        assert fake.state.motion["status"] == "run"
        ok(client.post("/api/tts", json={"text": "hello there"}))
        assert ok(client.post("/api/tts/stop"))["target"] == "robot"
        assert fake.state.tts["status"] == "idle" and fake.state.motion["status"] == "run"  # the dance goes on
        refused(client.post("/api/action", json={"name": "Shutdown"}))
        r = client.get("/api/photo?resolution=1280x720")
        assert r.status_code == 200 and r.content[:2] == b"\xff\xd8"
        assert logged(fake, "POST", "visions/photos")[-1][3]["resolution"] == "1280x720"
        refused(client.get("/api/photo?resolution=99x99"), 400)
        ok(client.post("/api/stop"))
        refused(client.post("/api/action/pause"))
    finally:
        c.disconnect()
        fake.stop()


def test_robot_camera_open_race():
    """exclusive() arriving while a proxy is still opening the robot stream waits, then the stream gets closed."""
    closes = []

    class Rest:
        def stream_close(self):
            closes.append(time.monotonic())

    class Ctl:
        robot = type("R", (), {"rest": Rest()})()

    class Reader:
        closed = False

        def close(self):
            self.closed = True
    cam, reader, order = web.RobotCamera(), Reader(), []

    def opener():
        time.sleep(0.4)  # stream_open round trip on a slow link
        return reader
    t = threading.Thread(target=lambda: order.append(("open", cam.open(Ctl(), opener))))
    t.start()
    time.sleep(0.1)
    with cam.exclusive(Ctl()):
        order.append(("exclusive", len(closes)))
    t.join(5)
    assert ("exclusive", 1) in order and ("open", None) in order and reader.closed, order
    assert not cam.opened and not cam.busy and not cam.opening


def test_mjpeg_reader_without_content_length():
    cam = start_cam()
    try:
        reader = web.MjpegReader(f"http://127.0.0.1:{cam.server_address[1]}/stream.mjpg?nolen=1")
        frames = [reader.next_frame() for _ in range(3)]
        reader.close()
        assert all(f == marked_jpeg() for f in frames)
        try:
            web.MjpegReader(f"http://127.0.0.1:{cam.server_address[1]}/nothing")
        except Exception:
            pass
        else:
            raise AssertionError("a 404 was taken for a stream")
    finally:
        stop_cam(cam)


def test_lifespan_owns_controller():
    """create_app() without a controller builds one at startup and closes it at shutdown."""
    tmp = tempfile.mkdtemp(prefix="yanshee_srv_own_")
    try:
        app = web.create_app(static_dir=os.path.join(tmp, "static"), renderer=False, motions_dir=tmp)
        with TestClient(app) as client:
            ctl = app.state.ctl
            assert ok(client.get("/api/state"))["sim"]["renderer_error"] == "renderer disabled"
            refused(client.get("/api/stream/front.mjpg"))
            refused(client.get("/api/photo"))
        assert app.state.ctl is None and ctl._running is False and os.path.isdir(os.path.join(tmp, "static"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# -- a real uvicorn server -------------------------------------------------------------------------------------
def test_uvicorn_streams_and_clean_shutdown():
    """Real HTTP: an endless MJPEG stream and a WebSocket stay open while the server shuts down."""
    from websockets.sync.client import connect as ws_connect
    _, c = setup()
    port = free_port(AUX_PORTS)
    app = web.create_app(c, static_dir=_S["static"])
    server = web.make_server(app, host="127.0.0.1", port=port, log_level="warning")
    th = threading.Thread(target=server.run, daemon=True)
    th.start()
    assert wait_until(lambda: server.started, 20)
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    conn.request("GET", "/%2e%2e/secret.txt")
    r = conn.getresponse()
    assert r.status == 404 and b"outside" not in r.read()
    conn.request("GET", "/../secret.txt")  # sent as is: no client-side normalisation
    r = conn.getresponse()
    assert r.status in (400, 404) and b"outside" not in r.read()
    conn.request("GET", "/api/motions/..%2F..%2Fsecret")
    r = conn.getresponse()
    assert r.status == 404 and b"outside" not in r.read()

    stream = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    stream.request("GET", "/api/stream/front.mjpg")
    resp = stream.getresponse()
    assert resp.status == 200 and resp.getheader("Content-Type") == web.MJPEG_TYPE
    buf = b""
    while buf.count(b"--frame\r\n") < 4:
        buf += resp.read1(65536)
    assert parse_mjpeg(buf[:buf.rfind(b"--frame\r\n")])
    ws = ws_connect(f"ws://127.0.0.1:{port}/ws", open_timeout=10)
    assert json.loads(ws.recv(timeout=5))["type"] == "state"

    t = time.monotonic()
    server.handle_exit(signal.SIGINT, None)  # what Ctrl+C does
    th.join(15)
    assert not th.is_alive(), "server did not shut down"
    assert time.monotonic() - t < 8, time.monotonic() - t
    tail = resp.read()  # the stream ends instead of hanging
    assert isinstance(tail, bytes)
    try:
        while True:
            ws.recv(timeout=3)
    except Exception as e:
        assert "timed out" not in str(e).lower(), e
    ws.close()
    stream.close()
    conn.close()
    assert c._running  # not owned by this app: still usable
    assert c.state()["mode"] == "sim"


def test_zz_close():
    if "client" in _S:
        _S.pop("client").__exit__(None, None, None)
        _S.pop("ctl").close()
        shutil.rmtree(_S.pop("dir"), ignore_errors=True)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            t = time.monotonic()
            fn()
            print("ok", name, f"({time.monotonic() - t:.1f} s)")
