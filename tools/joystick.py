#!/usr/bin/env python3
"""Minimal joystick page for the robot's own (firmware) walk: drag the knob to walk, release to stop gently.

    python3 tools/joystick.py --ip ROBOT_IP          # then open http://127.0.0.1:8100 (phone: http://LAPTOP_IP:8100 with --host 0.0.0.0)

It drives the robot the way the Yanshee mobile app does (captured from the app on 29 Oct 2026,
robot_dump/app_capture): the knob's main direction decides. Up/down walks with the firmware gait
(PUT /motions/gait, speed_v +-speed, period 1, arm swing on); left/right turns in place with the robot's TurnLeft /
TurnRight (PUT /motions, operation start_async, repeat 0, resent every 0.4 s while held, the robot then plays its own
ResetSwitch); release stops the app's way, a gait command with every value 0. "Side steps" maps left/right to speed_h
instead. The page repeats its command every 0.3 s while the knob is held; if that stops for 1 s the robot is stopped.
Auto get-up (on by default): after a real fall (upright to lying within 0.8 s, then still) the walk stops and the
robot plays its get-up (face down Base_GetupF + reset, on its back GetupRear; yanshee_sim/recovery.py).
Every command is logged to robot_dump/joystick_<time>.log for comparing with the simulator's walk later.
"""
import argparse
import datetime
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import recovery  # noqa: E402
from yanshee_sim.walkstats import WalkStats  # noqa: E402
from yanshee_sim.rest import YansheeRest  # noqa: E402

PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Yanshee joystick</title>
<style>body{font:16px system-ui;background:#11161c;color:#e4e9ef;margin:0;display:grid;justify-items:center;gap:14px;padding:18px}
#pad{width:260px;height:260px;border-radius:50%;background:#1c2733;border:2px solid #3a4756;position:relative;touch-action:none}
#knob{width:90px;height:90px;border-radius:50%;background:#6ea8ee;position:absolute;left:85px;top:85px}
button{font:inherit;padding:10px 16px;border-radius:6px;border:1px solid #3a4756;background:#1c2733;color:inherit}
#stop{background:#c8442a;border-color:#c8442a;width:260px}.row{display:flex;gap:10px;align-items:center}</style>
<div id=pad><div id=knob></div></div>
<div class=row>speed <input id=sp type=range min=1 max=5 value=2> <b id=spv>2</b>
<label><input id=wave type=checkbox checked> arm swing</label> <label><input id=side type=checkbox> side steps</label>
<label><input id=auto type=checkbox checked> auto get-up</label></div>
<div class=row><button id=tl>&#8634; Rotate left</button><button id=tr>Rotate right &#8635;</button></div>
<button id=stop>STOP</button><div id=st>ready</div>
<script>
const pad=document.getElementById('pad'),knob=document.getElementById('knob'),st=document.getElementById('st');
let cmd=null,timer=null;
const post=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})})
  .then(r=>r.json()).then(j=>st.textContent=j.msg).catch(e=>st.textContent='error: '+e);
sp.oninput=()=>spv.textContent=sp.value;
function move(e){const r=pad.getBoundingClientRect(),R=r.width/2;let x=e.clientX-r.left-R,y=e.clientY-r.top-R;
 const d=Math.hypot(x,y),m=R-45;if(d>m){x*=m/d;y*=m/d}knob.style.left=(x+R-45)+'px';knob.style.top=(y+R-45)+'px';
 const s=+sp.value,X=-x/m,Y=-y/m,q=v=>Math.abs(v)<0.25?0:Math.round(v*s);
 if(side.checked){cmd={v:q(Y),h:q(X),wave:wave.checked};return}
 if(Math.hypot(X,Y)<0.25){cmd={v:0,h:0,wave:wave.checked};return}
 cmd=Math.abs(Y)>=Math.abs(X)?{v:Y>0?s:-s,h:0,wave:wave.checked}:{turn:X>0?'left':'right',wave:wave.checked}}
function send(){if(cmd)post(cmd.turn?'/turnhold':'/gait',cmd)}
pad.onpointerdown=e=>{pad.setPointerCapture(e.pointerId);move(e);send();timer=setInterval(send,300)};
pad.onpointermove=e=>{if(timer)move(e)};
const release=()=>{if(!timer)return;clearInterval(timer);timer=null;cmd=null;knob.style.left=knob.style.top='85px';post('/stop')};
pad.onpointerup=pad.onpointercancel=release;window.onblur=release;
auto.onchange=()=>post('/auto',{enable:auto.checked});setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{if(!timer)st.textContent=j.msg}),1000);
stop.onclick=()=>{release();post('/stop')};tl.onclick=()=>post('/turn',{dir:'left'});tr.onclick=()=>post('/turn',{dir:'right'});
</script>"""


class Walker:
    """The robot's gait, updated only when the speeds change, stopped gently on release or a lost heartbeat."""

    def __init__(self, rest, log, stats=None):
        self.rest, self.log, self.lock, self.stats = rest, log, threading.Lock(), stats
        self.cur, self.last = None, 0.0  # (v, h, wave) while walking
        self.turning, self.turn_sent = None, 0.0  # the turn motion resent while the knob is held left/right
        self.auto, self.status, self.busy = True, "ready", False
        threading.Thread(target=self._watchdog, daemon=True).start()
        threading.Thread(target=self._fall_watch, daemon=True).start()

    def note(self, text):
        line = f"{datetime.datetime.now():%H:%M:%S.%f}"[:-3] + " " + text
        print(line, flush=True)
        self.log.write(line + "\n")
        self.log.flush()
        self.status = text
        return text

    def gait(self, v, h, wave):
        if self.busy:
            return "getting up: wait"
        with self.lock:
            self.last = time.monotonic()
            want = (max(-5, min(5, int(v))), max(-5, min(5, int(h))), bool(wave))
            if want[:2] == (0, 0):
                return self._stop("centred")
            self.turning = None  # walking after a turn: the gait takes over, as in the app
            if self.stats:
                self.stats.command("diag" if want[0] and want[1] else "side" if want[1] else "walk+" if want[0] > 0 else "walk-")
            if want == self.cur:
                return f"walking v={want[0]} h={want[1]}"
            res = self.rest.gait(want[0], want[1], 0, 1, want[2])
            self.cur = want if res.get("code") == 0 else None
            return self.note(f"gait v={want[0]} h={want[1]} wave={want[2]}: {res.get('msg')} (code {res.get('code')})")

    def stop(self, why="stop"):
        with self.lock:
            return self._stop(why)

    def _stop(self, why):
        if self.cur is None and not self.turning:
            return "stopped"
        was, self.cur, self.turning = self.cur, None, None
        if self.stats:
            self.stats.command("stop")
        res = self.rest.gait(0, 0, 0, 0, bool(was and was[2]))  # the app's stop: a gait command with every value 0
        return self.note(f"{why}: stop{' from v=%d h=%d' % was[:2] if was else ' after turning'}: {res.get('msg')}")

    def turn_hold(self, direction, wave=True):
        """Turn in place the app's way: TurnLeft/TurnRight with start_async, repeat 0, resent every 0.4 s while held."""
        if self.busy:
            return "getting up: wait"
        with self.lock:
            self.last = time.monotonic()
            if self.cur is not None:
                self._stop("before turning")
            name = "TurnLeft" if direction == "left" else "TurnRight"
            if self.turning == name and time.monotonic() - self.turn_sent < 0.4:
                return f"turning {direction}"
            res = self.rest.motion_control("start_async", name, "v1", "left", "normal", 0, int(time.time() * 1000))
            first = self.turning != name
            self.turning, self.turn_sent = name, time.monotonic()
            if self.stats:
                self.stats.command("turn_left" if direction == "left" else "turn_right")
            msg = f"{name} start_async repeat 0: {res.get('msg')} (code {res.get('code')})"
            return self.note(msg) if first else msg

    def turn(self, direction):
        self.stop("before turning")
        res = self.rest.play_motion("TurnLeft" if direction == "left" else "TurnRight")
        return self.note(f"turn {direction}: {res.get('msg')} (code {res.get('code')}, {(res.get('data') or {}).get('total_time')} ms)")

    def _fall_watch(self):
        det = recovery.FallDetector()
        while True:
            time.sleep(0.25)
            try:
                ex, ey, rate = recovery.read_imu(self.rest, with_y=True)
            except Exception:  # noqa: BLE001 - keep watching
                continue
            ev = det.feed(time.monotonic(), ex, rate, ey, active=self.cur is not None or bool(self.turning))
            if ev is None or self.busy:
                continue
            where = ev["posture"].replace("_", " ")
            if self.stats:
                self.stats.fall(ev["posture"], ev["fell"], at=time.time() - (time.monotonic() - ev["down_t"]))
            if not ev["fell"]:
                self.note(f"lying {where} (laid down, not a fall: no automatic get-up)")
                continue
            with self.lock:
                if self.cur is not None:
                    self.cur = None
                    self.rest.gait_exit()
            if not self.auto or ev["posture"] not in recovery.GETUP:
                self.note(f"FELL {where}" + (" (auto get-up is off)" if not self.auto else ": no get-up for that, stand it up by hand"))
                continue
            self.busy = True
            try:
                self.note(f"FELL {where}: getting up")
                ok, msg = recovery.robot_get_up(self.rest, ev["posture"])
                self.note(("stood up: " if ok else "did not stand up: ") + msg)
                if self.stats:
                    self.stats.getup(ok, msg)
            finally:
                self.busy = False

    def _watchdog(self):
        while True:
            time.sleep(0.2)
            if (self.cur is not None or self.turning) and time.monotonic() - self.last > 1.0:
                self.stop("no heartbeat for 1 s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to open it from a phone on the same network")
    ap.add_argument("--port", type=int, default=8100)
    ap.add_argument("--method", default="AppWalk", help="name for the fall statistics (tools/walk_stats.py)")
    ap.add_argument("--note", default="", help="e.g. the floor: smooth / mat")
    args = ap.parse_args()
    os.makedirs(os.path.join(ROOT, "robot_dump"), exist_ok=True)
    log = open(os.path.join(ROOT, "robot_dump", f"joystick_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"), "a")
    walker = Walker(YansheeRest(args.ip, timeout=4.0), log, WalkStats(args.method, args.note))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _reply(self, code, body, ctype="application/json"):
            data = body.encode() if isinstance(body, str) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/status":
                return self._reply(200, {"msg": walker.status})
            self._reply(200, PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            try:
                if self.path == "/gait":
                    msg = walker.gait(body.get("v", 0), body.get("h", 0), body.get("wave", False))
                elif self.path == "/turnhold":
                    msg = walker.turn_hold(body.get("turn", "left"), body.get("wave", True))
                elif self.path == "/stop":
                    msg = walker.stop()
                elif self.path == "/turn":
                    msg = walker.turn(body.get("dir", "left"))
                elif self.path == "/auto":
                    walker.auto = bool(body.get("enable", True))
                    msg = walker.note(f"auto get-up {'on' if walker.auto else 'off'}")
                else:
                    return self._reply(404, {"msg": "unknown"})
                self._reply(200, {"msg": msg})
            except Exception as e:  # noqa: BLE001 - shown on the page
                self._reply(200, {"msg": walker.note(f"error: {e}")})

    print(f"joystick for {args.ip}: http://{args.host}:{args.port}  (Ctrl+C stops the walk and quits)")
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        walker.stop("quit")
        walker.stats.close()


if __name__ == "__main__":
    main()
