"""FastAPI web server for the relay controller (docs/ARCHITECTURE.md section 3).

    python3 scripts/web_gui.py --port 8080             # then open http://localhost:8080/

    from yanshee_sim.web.server import create_app, run
    app = create_app()                                 # builds a Controller at startup, closes it at shutdown
    run(app, host="127.0.0.1", port=8080)

Every reply is {"ok": bool, ...}: a refused or failed command is HTTP 200 {"ok": false, "error"}, malformed
input is HTTP 400 {"ok": false, "error"}. Controller calls block (robot REST, activity hand-over), so they
run in the thread pool (plain `def` endpoints or run_in_threadpool), never on the event loop.
"""
import asyncio
import contextlib
import json
import logging
import math
import os
import re
import signal
import threading
import time
import urllib.request
from typing import Annotated, Any, Dict, List, Literal, Optional

import anyio
from fastapi import Body, FastAPI, Path, Query, WebSocket
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field
from starlette.concurrency import run_in_threadpool
from starlette.exceptions import HTTPException

from ..controller import GAIT_LEASE_S, PHOTO_RESOLUTIONS, VISION_TASKS, Controller, ModeError
from ..rest import SPEEDS
from ..robot import SERVO_MODES
from ..sim_extras import camera_name

log = logging.getLogger(__name__)

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MAX_BODY = 2 * 1024 * 1024  # bytes; motion files are a few kB
STATE_HZ = 5.0  # WebSocket state pushes per second
THREADS = 64  # thread-pool size (blocking controller calls, stream waits)
BOUNDARY = "frame"
MJPEG_TYPE = f"multipart/x-mixed-replace; boundary={BOUNDARY}"
STREAM_CAMS = ("head", "front", "side", "free")
NO_STORE = {"Cache-Control": "no-store, no-cache, must-revalidate", "Pragma": "no-cache"}
NAME_PATTERN = r"^[A-Za-z0-9_-]+$"


# -- JSON ------------------------------------------------------------------------------------------
def _plain(o):
    """o with non-finite floats as None and numpy scalars as Python numbers (strict JSON for browsers)."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v) for v in o]
    if hasattr(o, "item") and not isinstance(o, (str, bytes)):
        try:
            return _plain(o.item())
        except (TypeError, ValueError):
            pass
    return o


def dumps(o):
    try:
        return json.dumps(o, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError):
        return json.dumps(_plain(o), ensure_ascii=False, allow_nan=False, separators=(",", ":"), default=str)


class SafeJSONResponse(JSONResponse):
    def render(self, content):
        return dumps(content).encode("utf-8")


def _reply(res, status=200):
    if isinstance(res, list):
        res = {"ok": True, "items": res}
    return SafeJSONResponse(res, status_code=status)


def _fail(text, status=200):
    return SafeJSONResponse({"ok": False, "error": text}, status_code=status)


# -- request bodies ---------------------------------------------------------------------------------
class _Body(BaseModel):
    model_config = ConfigDict(strict=True, extra="ignore")


Finite = Annotated[float, Field(allow_inf_nan=False)]


class ModeBody(_Body):
    mode: Literal["sim", "robot", "both"]


class ConnectBody(_Body):
    ip: str = Field(min_length=1, max_length=260, pattern=r"^\s*(https?://)?[A-Za-z0-9._\-\[\]:]+/?\s*$")


class ServosBody(_Body):
    angles: Dict[str, Finite] = Field(min_length=1, max_length=17)
    runtime: float = Field(500, ge=0, le=60000, allow_inf_nan=False)


class ServoModeBody(_Body):
    mode: Literal[SERVO_MODES]
    servos: Optional[List[str]] = Field(None, max_length=17)


class ActionBody(_Body):
    name: str = Field(min_length=1, max_length=120)
    direction: str = Field("", max_length=20)
    speed: Literal[SPEEDS] = "normal"
    repeat: int = Field(1, ge=1, le=100)
    version: Literal["v1", "v2"] = "v1"


class GaitBody(_Body):
    speed_v: int = Field(0, ge=-5, le=5)
    turn: int = Field(0, ge=-5, le=5)  # + anticlockwise; simulator only (the robot turns with its built-in)
    speed_h: int = Field(0, ge=-5, le=5)
    steps: int = Field(0, ge=0, le=10000)
    period: int = Field(1, ge=0, le=5)
    wave: bool = False
    lease: float = Field(GAIT_LEASE_S, ge=0.5, le=10, allow_inf_nan=False)  # dead-man for steps=0 (see /gait)


class NameBody(_Body):
    name: str = Field(min_length=1, max_length=120)


class SpeedBody(_Body):
    speed: float = Field(1.0, gt=0, le=10, allow_inf_nan=False)


class PlayBody(_Body):
    motion: Dict[str, Any]
    speed: float = Field(1.0, gt=0, le=10, allow_inf_nan=False)


class RecordBody(_Body):
    runtime: float = Field(500, ge=200, le=4000, allow_inf_nan=False)
    hold: float = Field(0, ge=0, le=60000, allow_inf_nan=False)


class LedBody(_Body):
    type: Literal["button", "camera"]
    color: str = Field(min_length=1, max_length=20)
    mode: str = Field(min_length=1, max_length=20)


class VolumeBody(_Body):
    volume: int = Field(ge=0, le=100)


class LanguageBody(_Body):
    language: Literal["zh", "en"]


class FallBody(_Body):
    enable: bool


class TtsBody(_Body):
    text: str = Field(min_length=1, max_length=500)


class VisionBody(_Body):
    task: Literal[tuple(sorted(VISION_TASKS))]
    timeout: float = Field(8, gt=0, le=30, allow_inf_nan=False)


class FaceBody(_Body):
    name: str = Field(min_length=1, max_length=64, pattern=r"^[^\x00-\x1f/\\]+$")


MotionName = Path(min_length=1, max_length=64, pattern=NAME_PATTERN)


# -- robot camera proxy ---------------------------------------------------------------------------------
class MjpegReader:
    """JPEG frames from an upstream multipart/x-mixed-replace stream (the robot's http://IP:8000/stream.mjpg).
    Uses each part's Content-Length when given, else the JPEG start/end markers. Blocking; not thread-safe."""

    MAX_BUFFER = 8 * 1024 * 1024

    def __init__(self, url, timeout=3.0):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # never through an HTTP proxy
        self.resp = opener.open(url, timeout=timeout)
        ctype = (self.resp.headers.get("Content-Type") or "").lower()
        if "multipart" not in ctype:
            self.close()
            raise ValueError(f"not an MJPEG stream (Content-Type {ctype or 'missing'})")
        self.buf = b""

    def _fill(self):
        chunk = self.resp.read1(65536)
        if not chunk:
            raise EOFError("the robot's camera stream ended")
        self.buf += chunk
        if len(self.buf) > self.MAX_BUFFER:
            raise ValueError("no JPEG frame in 8 MB of the robot's camera stream")

    def next_frame(self):
        while True:
            start = self.buf.find(b"\xff\xd8")
            if start >= 0:
                m = re.search(rb"content-length:\s*(\d+)", self.buf[:start], re.I)
                if m and int(m.group(1)) > 0:
                    end = start + int(m.group(1))
                    if len(self.buf) >= end:
                        jpeg, self.buf = self.buf[start:end], self.buf[end:]
                        return jpeg
                else:
                    end = self.buf.find(b"\xff\xd9", start + 2)
                    if end >= 0:
                        jpeg, self.buf = self.buf[start:end + 2], self.buf[end + 2:]
                        return jpeg
            self._fill()

    def close(self):
        with contextlib.suppress(Exception):
            self.resp.close()


def _part(jpeg):
    head = b"--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % (BOUNDARY.encode(), len(jpeg))
    return head + jpeg + b"\r\n"


class RobotCamera:
    """The robot has one camera: the head-stream proxy shares it with photo, face entry and vision tasks
    ("Resource is not available" otherwise). Proxies count themselves in; the robot's stream is closed when the
    last one leaves, and while an exclusive() call runs the proxies show the twin's head camera instead."""

    RETRY_S = 10.0  # after a failed open, wait this long before trying again (one warning, not one per client)

    def __init__(self):
        self._lock = threading.Lock()
        self.users, self.busy, self.opened, self.failed_t = 0, 0, False, -1e9
        self.opening = 0  # proxies inside open(): exclusive() waits for them, then closes what they opened

    def enter(self):
        with self._lock:
            self.users += 1

    def leave(self, ctl):
        with self._lock:
            self.users -= 1
            last = self.users == 0 and self.opened
        if last:  # after a short grace, so a reloading page does not restart the robot's stream
            threading.Thread(target=self._close_if_unused, args=(ctl, 2.0), daemon=True).start()

    def may_open(self):
        return not self.busy and time.monotonic() - self.failed_t > self.RETRY_S

    def _close_if_unused(self, ctl, delay):
        time.sleep(delay)
        with self._lock:
            if self.users or not self.opened:
                return
            self.opened = False
        self._close(ctl)

    @staticmethod
    def _close(ctl):
        robot = ctl.robot
        if robot is not None:
            with contextlib.suppress(Exception):
                robot.rest.stream_close()

    def open(self, ctl, opener):
        """Run opener() (opens the robot stream; returns a reader or None) unless an exclusive() user holds the
        camera. If one arrived meanwhile, the stream just opened is closed again before exclusive() goes on."""
        with self._lock:
            if self.busy:
                return None
            self.opening += 1
        reader = bail = None
        try:
            reader = opener()
            with self._lock:
                bail = bool(self.busy) and reader is not None
            if bail:
                reader.close()
                self._close(ctl)
                reader = None
        finally:
            with self._lock:
                self.opening -= 1
                if reader is not None:
                    self.opened = True
        return reader

    @contextlib.contextmanager
    def exclusive(self, ctl, wait_s=10.0):
        """Free the camera for a robot photo / vision task (blocking; call from the thread pool)."""
        with self._lock:
            self.busy += 1
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:  # a proxy may be opening the stream right now: let it finish
            with self._lock:
                if not self.opening:
                    break
            time.sleep(0.05)
        with self._lock:
            was_open, self.opened = self.opened, False
        try:
            if was_open:
                self._close(ctl)
                time.sleep(0.3)  # let the proxies drop their connection
            yield
        finally:
            with self._lock:
                self.busy -= 1


# -- middleware -------------------------------------------------------------------------------------------
def _same_origin(origin, host):
    """Origin header (b"http://h:p") matches the Host header (b"h:p")."""
    o = origin.decode("latin-1").strip().lower()
    o = o.split("://", 1)[1] if "://" in o else o
    return bool(host) and o.rstrip("/") == host.decode("latin-1").strip().lower()


class _Guard:
    """ASGI middleware: refuses cross-site writes (a page on another origin driving the robot) and bodies
    over MAX_BODY (by Content-Length, and by counting the bytes of chunked bodies as they arrive). Same-origin
    requests from the GUI and clients without an Origin header pass."""

    def __init__(self, app, max_body=MAX_BODY):
        self.app, self.max_body = app, max_body

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http":
            headers = dict(scope.get("headers") or [])
            origin = headers.get(b"origin")
            status, text = 0, ""
            if origin and scope["method"] not in ("GET", "HEAD", "OPTIONS") and \
                    not _same_origin(origin, headers.get(b"host")):
                status, text = 403, "cross-origin request refused"
            else:
                try:
                    if int(headers.get(b"content-length") or 0) > self.max_body:
                        status, text = 413, f"request body over {self.max_body // 1024} kB"
                except ValueError:
                    status, text = 400, "bad Content-Length"
            if status:
                await SafeJSONResponse({"ok": False, "error": text}, status_code=status)(scope, receive, send)
                return
            too_big = f"request body over {self.max_body // 1024} kB"
            received, started = 0, False

            async def limited_receive():  # no or a lying Content-Length: count what really arrives
                nonlocal received
                msg = await receive()
                if msg.get("type") == "http.request":
                    received += len(msg.get("body") or b"")
                    if received > self.max_body:
                        raise HTTPException(413, too_big)  # FastAPI re-raises it from the body read
                return msg

            async def tracked_send(msg):
                nonlocal started
                started = started or msg.get("type") == "http.response.start"
                await send(msg)
            try:
                await self.app(scope, limited_receive, tracked_send)
            except HTTPException as e:
                if e.status_code != 413 or started:
                    raise
                await SafeJSONResponse({"ok": False, "error": too_big}, status_code=413)(scope, receive, send)
            return
        await self.app(scope, receive, send)


# -- state fan-out ----------------------------------------------------------------------------------------
class _Hub:
    """One controller snapshot per tick, shared by every WebSocket client."""

    def __init__(self, ctl, ttl=0.8 / STATE_HZ):
        self.ctl, self.ttl = ctl, ttl
        self._text, self._t = None, 0.0
        self._lock = asyncio.Lock()

    async def state_text(self):
        async with self._lock:
            if self._text is None or time.monotonic() - self._t >= self.ttl:
                st = await run_in_threadpool(self.ctl.state)
                self._text, self._t = dumps({"type": "state", **st}), time.monotonic()
            return self._text


# -- app ---------------------------------------------------------------------------------------------------
def create_app(controller=None, static_dir=STATIC_DIR, close_controller=None, **controller_kwargs):
    """The FastAPI app. With controller=None one is built at startup from controller_kwargs (Controller
    arguments) and closed at shutdown; a given controller is closed only if close_controller is true."""
    own = controller is None if close_controller is None else bool(close_controller)
    os.makedirs(static_dir, exist_ok=True)

    @contextlib.asynccontextmanager
    async def lifespan(app):
        import anyio.to_thread
        limiter = anyio.to_thread.current_default_thread_limiter()
        limiter.total_tokens = max(limiter.total_tokens, THREADS)
        app.state.closing.clear()
        if app.state.ctl is None:
            app.state.ctl = await run_in_threadpool(lambda: Controller(**controller_kwargs))
        app.state.hub = _Hub(app.state.ctl)
        try:
            yield
        finally:
            app.state.closing.set()  # ends MJPEG streams and WebSocket loops
            if own and app.state.ctl is not None:
                ctl, app.state.ctl = app.state.ctl, None
                await run_in_threadpool(ctl.close)
                log.info("controller closed")

    app = FastAPI(title="Yanshee relay", lifespan=lifespan, default_response_class=SafeJSONResponse,
                  docs_url="/api/docs", redoc_url=None, openapi_url="/api/openapi.json")
    app.state.ctl, app.state.closing, app.state.camera = controller, threading.Event(), RobotCamera()
    app.state.ws_clients = 0
    app.add_middleware(_Guard)

    def ctl():
        c = app.state.ctl
        if c is None:
            raise HTTPException(503, "the controller is not running")
        return c

    def event(level, text):
        c = app.state.ctl
        if c is not None and hasattr(c, "_event"):
            c._event(level, text)

    # -- errors --
    @app.exception_handler(RequestValidationError)
    async def bad_input(request, exc):
        parts = []
        for e in exc.errors()[:5]:
            loc = ".".join(str(x) for x in e.get("loc", ()) if x != "body")
            parts.append(f"{loc}: {e.get('msg')}" if loc else str(e.get("msg")))
        return _fail("invalid input: " + "; ".join(parts), 400)

    @app.exception_handler(HTTPException)
    async def http_error(request, exc):
        return _fail(str(exc.detail), exc.status_code)

    @app.exception_handler(Exception)
    async def crashed(request, exc):  # a bug: report it, keep serving
        log.error("unhandled error in %s: %r", request.url.path, exc)  # uvicorn logs the traceback
        event("error", f"server error in {request.url.path}: {exc}")
        return _fail(f"server error: {exc}", 500)

    # -- state, mode, connection --
    @app.get("/api/state")
    def get_state():
        return _reply({"ok": True, **ctl().state()})

    @app.post("/api/mode")
    def set_mode(body: ModeBody):
        try:
            return _reply(ctl().set_mode(body.mode))
        except ModeError as e:
            event("warn", str(e))
            return _fail(str(e))

    @app.post("/api/connect")
    def connect(body: ConnectBody):
        return _reply(ctl().connect(body.ip.strip()))

    @app.post("/api/disconnect")
    def disconnect():
        return _reply(ctl().disconnect())

    @app.post("/api/stop")
    def stop():
        return _reply(ctl().stop_all())

    @app.post("/api/reset")
    def reset():
        return _reply(ctl().reset())

    class TipBody(_Body):
        direction: Literal["front", "back", "left", "right"] = "front"

    @app.post("/api/tipover")
    def tipover(body: TipBody = Body(default=TipBody())):
        """Simulator test: push the robot over gently so Get up can be tried. Refused in robot/both mode."""
        return _reply(ctl().tip_over(body.direction))

    @app.post("/api/getup")
    def getup():
        """Stand up after a fall: the robot's GetupFront/GetupRear by its IMU, or the sim's get-up motion (respawn
        as a fallback)."""
        return _reply(ctl().get_up())

    # -- servos --
    @app.get("/api/servos")
    def get_servos():
        return _reply(ctl().get_servos())

    @app.post("/api/servos")
    def set_servos(body: ServosBody):
        return _reply(ctl().set_servos(body.angles, body.runtime))

    @app.post("/api/servos/mode")
    def set_servo_mode(body: ServoModeBody):
        return _reply(ctl().set_servo_mode(body.mode, body.servos))

    # -- actions, gait, music --
    @app.get("/api/actions")
    def actions():
        return _reply({"ok": True, "actions": ctl().list_actions()})

    @app.post("/api/action")
    def play_action(body: ActionBody):
        return _reply(ctl().play_action(body.name, body.direction, body.speed, body.repeat, body.version))

    @app.post("/api/action/pause")
    def action_pause():
        return _reply(ctl().pause_action())

    @app.post("/api/action/resume")
    def action_resume():
        return _reply(ctl().resume_action())

    @app.post("/api/gait")
    def gait(body: GaitBody):
        """steps=0 (walk until stopped) is a dead-man lease: the walk stops unless /api/gait or /api/gait/keepalive
        arrives within `lease` seconds (the GUI sends a keepalive every 0.4 s while a d-pad button is held)."""
        return _reply(ctl().gait(body.speed_v, body.speed_h, body.steps, body.period, body.wave,
                                 lease_s=body.lease if body.steps == 0 else None, turn=body.turn))

    @app.post("/api/gait/keepalive")
    def gait_keepalive():
        return _reply(ctl().gait_keepalive())

    @app.post("/api/gait/stop")
    def gait_stop():
        return _reply(ctl().gait_stop())

    @app.get("/api/music")
    def music_list():
        return _reply(ctl().music_list())

    @app.post("/api/music")
    def music_play(body: NameBody):
        return _reply(ctl().music_play(body.name))

    @app.post("/api/music/stop")
    def music_stop():
        return _reply(ctl().music_stop())

    # -- motion files (fixed paths first, then /api/motions/{name}) --
    @app.get("/api/motions")
    def motions():
        return _reply({"ok": True, "motions": ctl().motion_files()})

    @app.post("/api/motions/check")
    def check_motion(motion: Dict[str, Any] = Body(...), speed: float = Query(1.0, gt=0, le=10)):
        """Body: a motion object, or {"name": "saved_motion"} to check a saved file."""
        if "frames" not in motion and isinstance(motion.get("name"), str):
            if not re.match(NAME_PATTERN, motion["name"]):
                return _fail("motion names may only use letters, digits, '_' and '-'", 400)
            return _reply(ctl().check_motion(motion["name"], speed))
        return _reply(ctl().check_motion(motion, speed))

    @app.post("/api/motions/play")
    def play_unsaved(body: PlayBody):
        """Play an unsaved motion (the recorder's keyframe list) through the same checks as a file."""
        return _reply(ctl().play_motion(body.motion, body.speed))

    @app.get("/api/motions/{name}")
    def load_motion(name: str = MotionName):
        return _reply(ctl().load_motion(name))

    @app.put("/api/motions/{name}")
    def save_motion(name: str = MotionName, motion: Dict[str, Any] = Body(...)):
        return _reply(ctl().save_motion(name, motion))

    @app.delete("/api/motions/{name}")
    def delete_motion(name: str = MotionName):
        return _reply(ctl().delete_motion(name))

    @app.post("/api/motions/{name}/play")
    def play_motion(name: str = MotionName, body: Optional[SpeedBody] = None):
        return _reply(ctl().play_motion(name, (body or SpeedBody()).speed))

    @app.post("/api/record")
    def record(body: Optional[RecordBody] = None):
        body = body or RecordBody()
        return _reply(ctl().record_frame(body.runtime, body.hold))

    # -- settings, info, sensors --
    @app.post("/api/led")
    def led(body: LedBody):
        return _reply(ctl().set_led(body.type, body.color, body.mode))

    @app.post("/api/volume")
    def volume(body: VolumeBody):
        return _reply(ctl().set_volume(body.volume))

    @app.post("/api/language")
    def language(body: LanguageBody):
        return _reply(ctl().set_language(body.language))

    @app.post("/api/fall")
    def fall(body: FallBody):
        return _reply(ctl().set_fall_management(body.enable))

    @app.get("/api/info")
    def info():
        return _reply(ctl().info())

    @app.get("/api/sensors")
    def sensors():
        return _reply(ctl().sensors())

    @app.post("/api/sim/charge")
    def charge():
        return _reply(ctl().charge())

    # -- voice and vision --
    @app.post("/api/tts")
    def tts(body: TtsBody):
        return _reply(ctl().tts(body.text))

    @app.post("/api/tts/stop")
    def tts_stop():
        return _reply(ctl().tts_stop())

    def with_camera(fn):
        """Run fn(controller) with the robot camera freed from the head-stream proxy when the robot is a target."""
        c = ctl()
        if c.robot is None or c.mode == "sim":
            return fn(c)
        with app.state.camera.exclusive(c):
            return fn(c)

    @app.post("/api/vision")
    def vision(body: VisionBody):
        return _reply(with_camera(lambda c: c.vision(body.task, body.timeout)))

    @app.post("/api/face_entry")
    def face_entry(body: FaceBody):
        return _reply(with_camera(lambda c: c.face_entry(body.name)))

    @app.get("/api/photo")
    def photo(resolution: Literal[PHOTO_RESOLUTIONS] = "640x480"):
        res = with_camera(lambda c: c.photo(resolution))
        if isinstance(res, (bytes, bytearray)):
            return Response(bytes(res), media_type="image/jpeg", headers=NO_STORE)
        return _reply(res if isinstance(res, dict) else {"ok": False, "error": "no photo"})

    # -- MJPEG --
    @app.get("/api/stream/{cam}.mjpg")
    async def stream(cam: str, source: Literal["auto", "robot", "sim"] = "auto",
                     frames: int = Query(0, ge=0, le=100000)):
        """MJPEG of a sim camera. head in robot mode (or source=robot with a robot connected) proxies the robot's
        camera, falling back to the twin's head camera. frames > 0 ends the stream after that many frames."""
        if cam not in STREAM_CAMS:
            return _fail(f"unknown camera {cam!r}; use one of {list(STREAM_CAMS)}", 400)
        c = ctl()
        sim_cam = camera_name(cam)
        if not _robot_wanted(c, cam, source) and (c.renderer is None or c.renderer.error):
            return _fail(f"no simulator video: {c.renderer.error if c.renderer else 'renderer disabled'}")
        return StreamingResponse(_frames(c, cam, sim_cam, source, frames), media_type=MJPEG_TYPE,
                                 headers={**NO_STORE, "X-Accel-Buffering": "no"})

    def _robot_wanted(c, cam, source):
        if c.robot is None or cam != "head" or source == "sim":
            return False
        return source == "robot" or c.mode == "robot"

    def _open_upstream(c):
        camera = app.state.camera

        def opener():
            res = c.open_robot_stream()
            if res.get("ok"):
                try:
                    return MjpegReader(res["url"])
                except Exception as e:
                    event("warn", f"robot camera stream {res['url']} unavailable ({e}): showing the sim head camera")
            camera.failed_t = time.monotonic()
            return None
        return camera.open(c, opener)

    async def _frames(c, cam, sim_cam, source, limit):
        camera = app.state.camera
        upstream, n, sent, proxy = None, 0, 0, cam == "head" and source != "sim"
        if proxy:
            camera.enter()
        try:
            while not app.state.closing.is_set() and app.state.ctl is c:
                robot = _robot_wanted(c, cam, source) and not camera.busy
                if robot and upstream is None and camera.may_open():
                    upstream = await run_in_threadpool(_open_upstream, c)
                elif not robot and upstream is not None:
                    upstream.close()
                    upstream = None
                jpeg = None
                if upstream is not None:
                    try:
                        jpeg = await run_in_threadpool(upstream.next_frame)
                    except Exception as e:
                        if not camera.busy:
                            event("warn", f"robot camera stream stopped ({e}): showing the sim head camera")
                            camera.failed_t = time.monotonic() - camera.RETRY_S + 3.0  # retry in 3 s
                        upstream.close()
                        upstream = None
                        continue
                else:
                    r = c.renderer
                    if r is None or r.error:
                        if robot or camera.busy:  # no twin video: wait for the robot camera
                            await asyncio.sleep(0.5)
                            continue
                        break
                    jpeg, n = await run_in_threadpool(r.wait_frame, sim_cam, n, 1.0)
                if jpeg:
                    yield _part(jpeg)
                    sent += 1
                    if limit and sent >= limit:
                        break
        except Exception as e:  # never let a stream take the server down
            log.warning("stream %s: %s", cam, e)
        finally:
            if upstream is not None:
                upstream.close()
            if proxy:
                camera.leave(c)

    # -- WebSocket --
    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        origin = websocket.headers.get("origin")
        if origin and not _same_origin(origin.encode("latin-1"), websocket.headers.get("host", "").encode()):
            await websocket.close(code=1008)
            return
        await websocket.accept()
        c = app.state.ctl
        if c is None:
            await websocket.close(code=1011)
            return
        hub = getattr(app.state, "hub", None)
        if hub is None or hub.ctl is not c:
            hub = app.state.hub = _Hub(c)
        last = c.events[-1]["id"] if c.events else 0  # older events travel in the state snapshot
        sending = asyncio.Lock()
        app.state.ws_clients += 1

        async def send(text):
            async with sending:
                await websocket.send_text(text)

        async def reader():  # drains client messages ("ping" -> pong); returns when the client goes away
            while True:
                msg = await websocket.receive()
                if msg.get("type") == "websocket.disconnect":
                    return
                if msg.get("text") == "ping":
                    await send('{"type":"pong"}')

        rtask = asyncio.ensure_future(reader())
        try:
            while not app.state.closing.is_set() and not rtask.done():
                await send(await hub.state_text())
                for ev in c.events_since(last):
                    await send(dumps({"type": "event", **ev}))
                    last = ev["id"]
                await asyncio.wait({rtask}, timeout=1.0 / STATE_HZ)
        except Exception as e:  # client gone mid-send, or shutdown
            log.debug("websocket closed: %s", e)
        finally:
            rtask.cancel()
            app.state.ws_clients -= 1
            # shielded: a handler cancelled as the client leaves must still stop an orphaned walk
            with anyio.CancelScope(shield=True):
                with contextlib.suppress(Exception):
                    await websocket.close(code=1001 if app.state.closing.is_set() else 1000)
                if app.state.ws_clients == 0 and not app.state.closing.is_set():
                    with contextlib.suppress(Exception):
                        await run_in_threadpool(_stop_orphan_walk, c)

    def _stop_orphan_walk(c):
        """The last GUI page went away: a walk it started must not go on (the lease would end it too, later)."""
        if c.state()["activity"]["kind"] == "gait":
            event("warn", "walking stopped: the last controller page disconnected")
            c.gait_stop()

    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
    return app


def make_server(app, host="0.0.0.0", port=8080, log_level="info", graceful_s=5):
    """A uvicorn.Server for app whose shutdown (Ctrl+C, SIGTERM or handle_exit()) first ends open MJPEG streams
    and WebSockets, then runs the lifespan shutdown, which closes the controller (activity, renderer, sim)."""
    import uvicorn
    nested = hasattr(asyncio, "_run_orig")

    class Server(uvicorn.Server):
        def handle_exit(self, sig, frame):
            app.state.closing.set()
            super().handle_exit(sig, frame)

        def run(self, sockets=None):
            if not nested:
                return super().run(sockets)
            loop = asyncio.SelectorEventLoop()  # not asyncio.run: nest_asyncio's version may pick uvloop
            try:
                return loop.run_until_complete(self.serve(sockets=sockets))
            finally:
                loop.close()

        @contextlib.contextmanager
        def capture_signals(self):  # like uvicorn's, minus re-raising the signal after a clean shutdown
            if threading.current_thread() is not threading.main_thread():
                yield
                return
            old = {s: signal.signal(s, self.handle_exit) for s in (signal.SIGINT, signal.SIGTERM)}
            try:
                yield
            finally:
                for s, h in old.items():
                    signal.signal(s, h)

    # YanAPI (RealRobot.api) applies nest_asyncio, whose patched asyncio.run cannot drive uvloop's loop
    loop = "asyncio" if nested else "auto"
    config = uvicorn.Config(app, host=host, port=port, log_level=log_level, access_log=False, lifespan="on",
                            timeout_graceful_shutdown=graceful_s, loop=loop)
    return Server(config)


def run(app, host="0.0.0.0", port=8080, log_level="info", graceful_s=5):
    """Serve app with uvicorn until Ctrl+C / SIGTERM, then return (see make_server)."""
    make_server(app, host, port, log_level, graceful_s).run()


__all__ = ["create_app", "make_server", "run", "dumps", "MjpegReader", "RobotCamera", "STATIC_DIR", "MJPEG_TYPE",
           "BOUNDARY", "STREAM_CAMS"]
