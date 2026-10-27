# yanshee_sim architecture: relay controller + web GUI

Status: design contract (2026-10-27). The implementation must follow it. If something has to change,
update this file in the same change.

```
 Browser (static/index.html + app.js)   <-- HTTP + WebSocket -->   FastAPI server (yanshee_sim/web/server.py)
                                                                          |
                                                          Controller (yanshee_sim/controller.py)
                                                  mode = "sim" | "robot" | "both"      safety gate
                                                    /                              \
                                   SimRobot (+ SimRenderer, LEDs,          RealRobot -> YansheeRest (yanshee_sim/rest.py)
                                   sim built-ins, sim gait)                 per-instance REST client with timeouts
                                                                            -> robot http://IP[:port]/v1/...
```

## 1. Relay modes

| Mode | Commands go to | Simulator shows |
|---|---|---|
| `sim` | simulator only | the commands |
| `robot` | real robot only (after the safety gate) | the robot's live servo readback (digital twin, 4 Hz) |
| `both` | simulator **and** robot | its own simulation of each command. For things the sim cannot emulate (robot-only built-ins, gait, dances), it follows the robot's readback until that activity ends. |

- Switching to `robot` or `both` needs a connected robot (`connect(ip)`); otherwise the call fails with a clear message.
- Switching modes stops everything first.
- The safety gate is the same for `robot` and `both`, and checks every command before it reaches hardware:
  - **Servo commands:** limits (clamp plus warn), self-collision via `CollisionChecker` (refuse), and a speed cap (90 deg/s) that uses the *estimated in-flight position* (refuse or stretch the runtime). After a robot built-in or gait (which move the robot outside the gate) or a failed servo command, the gate is re-synced from a fresh readback before the next servo command; if that read fails the command is refused.
  - **Motion files:** `validate` and `self_collisions`, including the `end_in_reset` return move, starting from the current pose. On the robot every frame (and the return to reset) also obeys the speed cap: its runtime is stretched from the estimated pose, the sim gets the same runtime in `both`, and playback waits the extra time; `check_motion` lists the stretched frames.
  - **Battery:** below 30% refuses motions; all other commands still work.
  - **System motions** (`Shutdown`, `calibration`, `ActionAging`, `ResetSwitch`, `Enter/ExitEnergySaving`) are hidden from the action list and refused by `play_action` and `RealRobot.play_builtin` (unless `allow_system=True`).
- STOP always goes to every target, whatever the mode:
  - **Robot:** `stop_play_motion` v1 and v2, `exit_motion_gait`, `stop_play_music`, `stop_voice_tts`; then, if a servo command or motion-file frame is still in flight (the gate says so), the robot is held: its readback is commanded back over 200 ms.
  - **Simulator:** cancel playback and hold the current pose.
- Races with STOP: each motion command reads the STOP generation together with its targets before it waits for
  anything. STOP, a mode switch and a disconnect bump that generation, so a command that was still waiting for the
  previous activity is refused afterwards (`"... cancelled by STOP"`, or `"... robot disconnected"`), and one that
  reached the robot in the gap is undone (robot stops and holds). Motion commands that arrive while a STOP or mode
  switch is still running are refused (`"refused: STOP or a mode switch is in progress"`).
- A start request (built-in, gait) that times out may still reach the robot: a best-effort stop follows (again after
  one robot timeout, unless a new activity started) and the error says "the robot may still have received it".
- Open-ended walking (`steps=0`) can carry a dead-man lease (`gait(..., lease_s=)`): it stops unless `gait()` or
  `gait_keepalive()` comes within the lease. The web server always sets one (section 3).

## 2. Python modules

### `yanshee_sim/rest.py`: `YansheeRest(host, timeout=4.0)`

Per-instance client for the robot's REST API.
- **Addressing:** `host` is `"192.168.3.13"` or `"127.0.0.1:9191"` (default port 9090).
- **No YanAPI:** it does not use the YanAPI module, so there is no global IP and every request has a timeout.
- **Errors:** methods return the robot's JSON dict. Transport errors raise `RobotUnreachable`; non-JSON replies raise `RobotProtocolError`.
- **Paths:** endpoint paths and payloads must match the official `YanAPI.py` (reference: `../yanshee robot/yanshee/yanshee_examples/yanshee_examples/YanAPI_en.py`).

| Area | Methods |
|---|---|
| Device | `battery()`, `versions(type)`, `mode()`, `get/set_volume`, `get/set_language`, `get/set_led(type,color,mode)`, `get/set_fall_management` |
| Servos | `get_angles(names)`, `set_angles(angles, runtime)`, `get_servo_mode(names)`, `set_servo_mode(mode, names)` |
| Motions | `motion_list()`, `play_motion(name, direction="", speed="normal", repeat=1, version="v1")`, `motion_control(operation, name="", version="v1")` (pause/resume/stop), `motion_state()`, `layer_motion_state()` |
| Gait | `gait(speed_v, speed_h, steps, period, wave)`, `gait_state()`, `gait_exit()` |
| Music | `music_list()`, `music_play(name)`, `music_stop()`, `music_state()` |
| Voice | `tts(text, interrupt=True)`, `tts_state(timestamp)`, `tts_stop()`, `asr(timeout)` / `iat(timeout)` (start, then poll with a timeout) |
| Vision | `vision(option, type, timeout=8)` (start, poll `get_visual_task_result`, stop on timeout), `photo(resolution)` (returns JPEG bytes: take + download), `stream_open(res)` / `stream_close()` (stream URL `http://HOST:8000/stream.mjpg`), `face_entry(name)`, `vision_tags()`, `delete_vision_tag(tag)` |
| Sensors | `sensors_list()`, `gyro()`, `infrared()`, `ultrasonic()`, `touch()`, `pressure()`, `environment()` |

### `yanshee_sim/robot.py`

`SimRobot` and `RealRobot` keep their documented API (see README section 6).
- **`RealRobot(ip, min_battery=30)`:** uses `YansheeRest` internally and exposes it as `.rest`. `.api` remains available as the official YanAPI module, imported lazily and initialised with the IP, for advanced use only.
- **Extra `RealRobot` methods:** `set_servos_mode(mode, names)`, `play_builtin(name, direction="", speed="normal", repeat=1, version="v1", allow_system=False)` (system motions raise `ValueError`), `stop_all()`.
- **Extra `SimRobot` methods:** `set_servos_mode(mode, names)` ("program" = limp: kp 0 for those actuators), `stop_all()` (cancel moves and hold the current pose), and the `led` state.

### `yanshee_sim/sim_extras.py`

| Piece | Behaviour |
|---|---|
| `SimRenderer(sim, width=640, height=480, fps=12)` | Owns one MuJoCo `Renderer` in its own thread. Copies `MjData` under the sim lock, renders outside it, and keeps the latest JPEG for named cameras: `head_camera` (FPV), `front`, `side`, `free` (orbit). `latest(cam)` returns JPEG bytes. Uses EGL when there is no display. |
| LEDs | Chest and eye LED markers (sites or geoms added via `MjSpec` at load, no rebuild). Colour and mode (on / off / blink / breath) are animated in the renderer. |
| Sim built-ins | Approximate keyframe files in `motions/builtin/` for the parametric built-ins: `raise`, `wave`, `stretch`, `come on` (left/right/both), `bend` (left/right), `head` (forward/left/right), `bow`, `crouch`, `reset`. Each is tested to keep the robot upright. File name: `NAME_DIRECTION.json` with spaces as `_`, e.g. `come_on_left.json`. |
| Sim gait | A stepping generator (`SimGait`) driven by `speed_v`/`speed_h`/`steps`/`period`, with ankle balance feedback from the simulated IMU; its constants are tuned by `tools/tune_gait.py`. It must not make the robot fall in sim; if real walking is not stable, step in place and say so in the activity text. |
| Built-ins with no sim version | In `sim` mode: "not available in simulation". In `both`: twin-follow. |
| Sim battery | Starts at 100% and drains slowly with actuator effort. `charge()` resets it. |

### `yanshee_sim/controller.py`: `Controller`

```python
ctl = Controller(start_mode="sim", sim_viewer=False, fixed_base=False)
ctl.mode                      # "sim" | "robot" | "both"
ctl.set_mode(mode)            # raises ModeError if a robot is needed but not connected
ctl.connect(ip) / ctl.disconnect()
ctl.state()                   # JSON-serialisable snapshot (see section 4)
ctl.stop_all(); ctl.reset()
ctl.set_servos(angles, runtime=500)        # goes through the safety gate; returns {"ok", "sent_to", "warnings", "blocked"}
ctl.get_servos()                           # {"sim": {...}, "robot": {...}|None}
ctl.set_servo_mode(mode, names)            # PRP limp/work
ctl.play_action(name, direction="", speed="normal", repeat=1, version="v1")
ctl.list_actions()                         # [{name, directions, category, sim:bool, robot:bool, music:bool}]
ctl.gait(speed_v, speed_h, steps=0, period=1, wave=False, lease_s=None); ctl.gait_keepalive(); ctl.gait_stop()
ctl.pause_action(); ctl.resume_action()   # robot built-ins (motion_control pause/resume); activity.detail "paused"
ctl.music_play(name); ctl.music_stop(); ctl.music_list()
ctl.motion_files(); ctl.load_motion(name); ctl.save_motion(name, motion); ctl.delete_motion(name)
ctl.check_motion(motion)                   # {"errors", "warnings", "collisions", "duration_s"}; see below
ctl.play_motion(name_or_dict, speed=1.0)   # async in a thread; one playback at a time; cancellable
ctl.record_frame(runtime, hold)            # PRP: capture the current angles (robot readback in robot/both, sim otherwise)
ctl.set_led(type, color, mode); ctl.set_volume(v); ctl.set_language(l); ctl.set_fall_management(enable)
ctl.info()                                 # battery, versions, flags per target
ctl.sensors()                              # gyro (+euler) per target, add-on sensors (robot), "not connected" otherwise
ctl.tts(text)                              # robot: robot TTS; sim: returns {"speak_in_browser": text}
ctl.tts_stop()                             # robot TTS only (stop_voice_tts) when a robot is connected
ctl.vision(task, timeout=8)                # robot: real; sim: "colour" from the rendered head camera, otherwise not available
ctl.photo(resolution="640x480")           # JPEG bytes (robot photo at 320x240..1920x1080, or sim head-camera render)
ctl.events                                 # thread-safe deque of log events {id, t, level, text}
ctl.events_since(last_id)                  # events with id > last_id (the server pushes these over the WebSocket)
ctl.face_entry(name)                       # robot only
ctl.charge()                               # sim battery back to 100%
ctl.open_robot_stream(resolution)          # {"ok", "url"}: the robot's head-camera MJPEG
ctl.close()                                # stop the activity, renderer and twin threads (also a context manager)
```

Return details: every method returns a dict with `"ok"`, except `list_actions()` and `motion_files()` (lists) and
`photo()` (JPEG bytes, or `{"ok": False, "error"}`). `gait`: `speed_h > 0` steps to the robot's left; `period=0` stops.
`set_servos` refusals carry `blocked: [[body, body], ...]`; `play_motion` refusals carry `errors`, `warnings`, `collisions`.
`gait_stop()` sends `exit_motion_gait` to a connected robot whatever the mode (like STOP). `set_servo_mode("program")`
on robot leg servos adds a warning (the robot cannot stand). In `check_motion`, `collisions[].frame` is 0-based
(`len(frames)` = the `end_in_reset` move), `collisions[].label` names it ("frame 1", "return to reset") and `at` is the
fraction of that move; `validate` messages count frames from 1 too.

Rules:
- All public methods are thread-safe (the server calls them from a thread pool).
- Long operations (motion playback, built-ins, gait) run in **one** cancellable activity thread. Starting a new activity cancels the previous one first.
- Robot errors are caught and reported as events plus a `{"ok": False, "error": ...}` result. They never crash the server.
  Only `set_mode` raises (`ModeError`).
- A robot that disconnects while a command is being checked gives `{"ok": False, "error": "... robot disconnected"}`
  (the gate is read under its lock and must belong to the robot the command was aimed at).
- A robot activity (built-in, dance, gait) ends with an error event when the robot has not answered for 5 s
  (`ROBOT_LOST_S`); the twin marks the robot offline after 3 failed servo reads.

## 3. Web server: `yanshee_sim/web/server.py`

Started with `python3 scripts/web_gui.py [--host 0.0.0.0] [--port 8080] [--mode sim|robot|both] [--ip IP] [--fixed] [--viewer]
[--motions-dir DIR] [--log-level info]`. `--mode robot|both` needs `--ip`, and in those modes a failed connection exits
with code 2 (with `--mode sim` a failed `--ip` only warns and the GUI starts in sim mode). A busy port always exits 2.

The server is FastAPI plus uvicorn, and serves `yanshee_sim/web/static/` at `/`. All bodies are JSON. Every response is `{"ok": bool, ...}`; errors are `{"ok": false, "error": "message"}` with HTTP 200 (or 400 for malformed input).
Also: 404 for unknown paths, 405 for a wrong method on an existing path (e.g. `POST /api/nonexistent` matches the
static mount and gives 405), and 500 for an unexpected server error (all in the same JSON form; a 500 is logged as an
event), 403 for a write request whose `Origin` does not match the host (another web page cannot drive the robot), and
413 for bodies over 2 MB (by `Content-Length`, and counted as they arrive for chunked bodies). Controller calls run in
a thread pool, never on the event loop.

| Method | Path | Body / query | Purpose |
|---|---|---|---|
| GET | `/api/state` | | Full snapshot (section 4) |
| WS | `/ws` | | Server pushes `{"type":"state", ...}` about 5 times a second and `{"type":"event", ...}` as events happen |
| POST | `/api/mode` | `{mode}` | Switch the relay |
| POST | `/api/connect` | `{ip}` | Connect the real robot |
| POST | `/api/disconnect` | | |
| POST | `/api/stop` | | Stop everything |
| POST | `/api/reset` | | Reset pose |
| GET / POST | `/api/servos` | `{angles, runtime}` | Read / set servos |
| POST | `/api/servos/mode` | `{mode, servos}` | PRP limp/work |
| GET | `/api/actions` | | Action list |
| POST | `/api/action` | `{name, direction, speed, repeat, version}` | Play a built-in |
| POST | `/api/action/pause` / `/api/action/resume` | | Pause / resume the robot built-in that is playing |
| POST | `/api/gait` | `{speed_v, speed_h, steps, period, wave, lease}` | Walk. `steps=0` is a dead-man lease: it stops unless `/api/gait` or `/api/gait/keepalive` arrives within `lease` s (default 1.5, 0.5-10) |
| POST | `/api/gait/keepalive` | | Refresh the walk's lease; returns `{walking}` |
| POST | `/api/gait/stop` | | |
| GET | `/api/music` | | Music list |
| POST | `/api/music` | `{name}` | |
| POST | `/api/music/stop` | | |
| GET | `/api/motions` | | Motion files |
| GET / PUT / DELETE | `/api/motions/{name}` | motion JSON | Motion file CRUD (names `[A-Za-z0-9_-]+`, no paths) |
| POST | `/api/motions/{name}/play` | `{speed}` | |
| POST | `/api/motions/check` | motion JSON | Validation report |
| POST | `/api/record` | `{runtime, hold}` | Capture a PRP frame; returns `{frame}` |
| POST | `/api/led` | `{type, color, mode}` | |
| POST | `/api/volume` | `{volume}` | |
| POST | `/api/language` | `{language}` | |
| POST | `/api/fall` | `{enable}` | |
| GET | `/api/info` | | |
| GET | `/api/sensors` | | |
| POST | `/api/tts` | `{text}` | |
| POST | `/api/tts/stop` | | Stop the robot's speech only (STOP stops everything) |
| POST | `/api/vision` | `{task, timeout}` | `task` is one of `face:recognition`, `face:quantity`, `face:age`, `face:age_group`, `face:gender`, `face:expression`, `face:mask`, `face:glass`, `gesture`, `color`, `object`, `qr` |
| POST | `/api/face_entry` | `{name}` | |
| GET | `/api/photo` | `?resolution=320x240\|640x480\|1280x720\|1920x1080` | `image/jpeg` (resolution: robot camera; default 640x480) |
| GET | `/api/stream/{cam}.mjpg` | | MJPEG (`multipart/x-mixed-replace`). `cam` is `head`, `front`, `side` or `free`. In `robot` mode, `head` proxies the robot's `http://IP:8000/stream.mjpg` (opened on demand) and the others show the twin. |
| POST | `/api/sim/charge` | | Reset the sim battery |
| POST | `/api/motions/play` | `{motion, speed}` | Play an unsaved motion (the recorder's keyframes) through the same checks as a file |
| GET | `/api/docs` | | OpenAPI UI |

Details:
- `GET /api/actions` returns `{ok, actions: [...]}` and `GET /api/motions` returns `{ok, motions: [...]}`; other routes
  pass the controller's dict through.
- `POST /api/motions/check` also accepts `{"name": "saved_name"}`, and `?speed=`. `POST /api/record` and
  `POST /api/motions/{name}/play` may be sent without a body (defaults: runtime 500, hold 0; speed 1).
- `GET /api/stream/{cam}.mjpg?source=auto|robot|sim&frames=N`: `auto` proxies the robot's head camera only in `robot`
  mode, `robot` also in `both`; `frames=N` ends the stream after N frames. If the robot camera cannot be reached the
  stream falls back to the twin's head camera (one warning). The robot has one camera: the proxy releases it for
  `/api/vision`, `/api/photo` and `/api/face_entry`, and 2 s after the last viewer leaves.
- WS `/ws`: sending the text `ping` gets `{"type": "pong"}`. When the last WebSocket client disconnects, a running walk
  is stopped (the lease would end it too).
- The robot camera's stream open and `exclusive()` (vision, photo, face entry) are serialised: an exclusive call that
  arrives while a proxy is opening the stream waits for it and then closes it.
- Ctrl+C / SIGTERM end open streams and WebSockets at once, close the controller and exit with code 0.

## 4. State snapshot (`/api/state` and WS `state`)

```json
{"mode": "sim", "robot": {"connected": false, "ip": null, "battery": null, "charging": null, "versions": null},
 "sim": {"battery": 100, "tilt_deg": 1.2, "fallen": false, "fixed_base": false},
 "servos": {"sim": {"NeckLR": 90, "...": 0}, "robot": null},
 "activity": {"kind": "idle|motion|action|gait|music|tts|vision", "name": "", "target": "", "started": 0.0, "progress": 0.0},
 "leds": {"button": {"color": "blue", "mode": "breath"}, "camera": {"color": "green", "mode": "on"}},
 "volume": 50, "language": "en", "fall_management": false,
 "limits": {"NeckLR": [15, 165], "...": [0, 0]}, "reset_pose": {"NeckLR": 90},
 "events": [{"t": 0.0, "level": "info|warn|error", "text": "..."}]}
```

Additions to the shape above: `robot.online` (false after 3 failed servo reads), `sim.renderer_error` (null, or why the
sim camera is not rendering), `activity.detail` (e.g. "sim follows the robot"), and an `id` on every event. Activity
kinds: `idle`, `action`, `motion`, `gait`, `music`, `vision`. In `sim` mode music has no audio and no end: it shows as
the activity (when nothing else runs) until Stop music or STOP.

## 5. GUI (`yanshee_sim/web/static/`)

A single page with no build step (plain HTML/CSS/JS, no CDN dependencies, so it works offline at an exhibition). It works on a laptop and on a tablet. It has a top bar plus tabs.

**Top bar** (always visible):
- the **Sim / Robot / Both** relay toggle
- robot IP field with Connect / Disconnect
- battery for the sim and the robot
- the current activity
- a big **STOP** button (Escape key, also while a confirm dialog is open; dialogs have their own STOP button)
- Pause / Resume next to the activity while a robot built-in plays
- **Reset** button

**Live view** (always visible, left or top): the MJPEG sim view with a camera selector (front / side / free / head FPV), the robot's head stream in robot mode, and LED indicators.

| Tab | Contents |
|---|---|
| **Motion control** | Walking D-pad/joystick (hold to walk, release to stop; speed slider 1–5, side steps, arm swing), turn left/right; built-in action grid built from `/api/actions` with direction, speed and repeat controls, and categories (gestures, locomotion, fight, football, get-up, dances, v2 layered, user); dances with their music; get-up buttons. |
| **Servos & recorder** (PRP) | 17 sliders grouped by body part, with live readback per target; limp/work toggle per limb group (robot; limping leg servos in robot/both mode asks for confirmation first); "capture frame" with runtime and hold; editable keyframe list (reorder, delete, edit); preview in sim, play on the target; check report; save / load / delete motion files. |
| **Settings & sensors** | Volume, chest/eye LED colour + mode, language, fall management, versions/SN, IMU numbers and a small rolling chart (euler/gyro), add-on sensors (IR, ultrasonic, touch, pressure, environment), sim battery charge. |
| **Voice & vision** | TTS box: robot speaks it; in sim the browser speaks it with `speechSynthesis`; "Stop speaking" stops only speech (`/api/tts/stop`). Vision task buttons with result cards; face entry (name + capture); photo snapshot with a resolution choice. A note that ASR is Chinese-only on the robot. |
| **Log** | Event stream. |

Tabs have URL hashes (`#motion`, `#servos`, `#settings`, `#voice`, `#log`). Keys: Esc = STOP (anywhere); arrows or WASD
walk while held (Motion tab). Walking always stops on release, pointer leave/cancel, window blur and page hide; while a
direction is held the page sends `/api/gait/keepalive` every 0.4 s, so a lost tablet stops the walk within 1.5 s. A gait
request that got no answer is followed by a stop, and release sends a stop whenever a gait request went out without a
confirmed stop since. While the WebSocket is down the page polls `/api/state` every second and shows a banner 1.5 s
after the connection dropped (once per outage); while the server is unreachable the live view says the video is paused. Per-viewer settings live in
`localStorage` under the `yanshee.` prefix.

Safety UX:
- Every robot-bound refusal (collision, limits, battery) is shown as a visible toast with its reason.
- Switching to `robot` or `both` asks for confirmation ("robot on a flat surface, emergency stop within reach").

## 6. Testing without hardware

`tests/fake_robot.py` is a fake Yanshee REST server: `python3 tests/fake_robot.py --port 9191`.
- **Endpoints:** every endpoint `YansheeRest` uses, with plausible state.
- **Servos:** move linearly over their runtime.
- **Motion list:** taken from the captured real list (`tests/data/motion_list_real.json`).
- **Battery:** configurable.
- **Vision:** canned results.
- **Error injection:** `--battery 20`, `--refuse`, `--slow 5`; from Python also `srv.state.dead = True` (drops every
  connection but keeps the port, so a test's "dead robot" cannot be replaced by another process binding the port).
- Tests start fakes on OS-assigned ports (`fake_robot.start(0)`); clients that time out do not print tracebacks.

`tests/` gains:
- `test_rest.py`: client against the fake robot
- `test_controller.py`: all modes and the safety gate
- `test_server.py`: FastAPI `TestClient` against every endpoint, the WebSocket, and MJPEG headers

Everything must pass headless with `MUJOCO_GL=egl`: `python3 -m pytest -q tests`, or each `python3 tests/test_*.py`.
`tests/pytest.ini` blocks the unrelated `dash` pytest plugin (installed in the user's site-packages, it needs flask and
stopped pytest at startup).
