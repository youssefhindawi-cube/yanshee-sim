# yanshee_sim

A MuJoCo simulator, a ROS 2 description and sim-to-real tools for the **UBTECH Yanshee** humanoid
(17 servos, Raspberry Pi 3B + STM32, YanAPI REST on port 9090).

Design a movement in the simulator, check it is safe, then run the same file on the real robot.
You can also mirror the two live: move the simulator and the robot follows, or move the robot and
the simulator follows. For exhibitions there is a **web GUI** (laptop or tablet browser) that drives
the simulator, the real robot, or both at once through a safety gate:
[`scripts/web_gui.py`](#scriptsweb_guipy-web-gui-simulator--robot-relay), designed in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

![Sample motions in the simulator](docs/motions.png)

---

## Contents

1. [Quick start (5 minutes)](#1-quick-start-5-minutes)
2. [Installation](#2-installation)
3. [How it works: the one rule](#3-how-it-works-the-one-rule)
4. [Command reference](#4-command-reference)
5. [Live mirroring: simulator ↔ robot](#5-live-mirroring-simulator--robot)
6. [Python API reference](#6-python-api-reference)
7. [Motion files](#7-motion-files)
8. [Servo reference](#8-servo-reference)
9. [From simulation to the real robot: step by step](#9-from-simulation-to-the-real-robot-step-by-step)
10. [ROS 2](#10-ros-2)
11. [Rebuilding the model](#11-rebuilding-the-model)
12. [Tests](#12-tests)
13. [Accuracy and limits](#13-accuracy-and-limits)
14. [Troubleshooting](#14-troubleshooting)
15. [Repository layout](#15-repository-layout)

---

## 1. Quick start (5 minutes)

```bash
git lfs install                       # once per machine (the Webots source model is stored with Git LFS)
git clone git@github.com:youssefhindawi-cube/yanshee-sim.git
cd yanshee-sim
pip install -r requirements.txt                           # MuJoCo >= 3.3, numpy, the web GUI's FastAPI stack

python3 scripts/web_gui.py --host 127.0.0.1               # web GUI at http://localhost:8080/ (simulator)
python3 scripts/view.py --fixed                           # look at the robot, drag joints by hand
python3 scripts/play_motion.py motions/wave_right.json    # watch a motion
python3 scripts/pose_editor.py --out motions/hello.json   # build your own motion
```

With the robot on the same Wi-Fi (find its IP in the Yanshee app); nothing extra to install:

```bash
python3 scripts/web_gui.py --ip 192.168.3.13                                    # GUI; switch to Robot/Both there
python3 scripts/mirror.py from-robot --ip 192.168.3.13                          # simulator copies the robot
python3 scripts/play_motion.py motions/wave_right.json --real --ip 192.168.3.13 --speed 0.5
```

---

## 2. Installation

| Need | Install |
|---|---|
| Everything | `pip install -r requirements.txt` |
| Simulator (required) | `mujoco>=3.3` (the code uses the `MjSpec` editing API, new in 3.3) and `numpy`; tested with MuJoCo 3.8 and Python 3.10. |
| Web GUI | `fastapi`, `uvicorn`, `websockets`, `Pillow` (`httpx` only for the server tests). |
| Real robot | Nothing extra: `RealRobot` talks to the robot's REST API with the standard library (`yanshee_sim/rest.py`). |
| `RealRobot.api` (optional, advanced) | The official `YanAPI.py` module, for calls the package does not wrap (`bot.api.sync_do_face_recognition(...)`). Needs `requests`, `nest_asyncio`, `opencv-python` and the YanAPI folder (below). |
| ROS 2 (optional) | ROS 2 Humble with `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`. |
| The model file | `git lfs install` before cloning, or `git lfs pull` afterwards. Without it, `source_model/robot.json` is a tiny pointer file. You only need that file to rebuild the model; `model/` already contains the generated files. |

**Where YanAPI comes from (only for `RealRobot.api`).** The first access to `bot.api` imports the
official `YanAPI.py`. By default it looks in `../yanshee robot/yanshee/yanshee_examples/yanshee_examples/`.
Point it elsewhere with:

```bash
export YANAPI_DIR=/path/to/folder/containing/YanAPI.py
```

All commands are run from the repository root.

---

## 3. How it works: the one rule

> **Simulated joint angle (radians) = real servo angle (degrees) converted to radians.**

- **Joint names:** the YanAPI servo names (`RightShoulderRoll` … `NeckLR`).
- **Joint limits:** the YanAPI servo limits.
- **Units in files and code:** servo **degrees**, the exact numbers `YanAPI.set_servos_angles()` takes. Nothing is converted between simulation and hardware.
- **Why it holds:** the model comes from UBTECH's Webots model of Yanshee. Its saved joint positions are the servo angles in radians, and they reproduce the Yanshee Course's default standing pose.

The simulator runs physics **in real time in a background thread**, so it behaves like the robot:
`set_servos_angles(..., runtime)` returns at once while the servos move over `runtime` ms, and
`time.sleep()` in your scripts works the same on both.

---

## 4. Command reference

### `scripts/web_gui.py`: web GUI (simulator / robot relay)

```bash
python3 scripts/web_gui.py                                  # simulator only, http://localhost:8080/ (and the LAN)
python3 scripts/web_gui.py --host 127.0.0.1                 # reachable from this machine only
python3 scripts/web_gui.py --ip 192.168.3.13                # connect the robot; switch modes in the GUI
python3 scripts/web_gui.py --mode both --ip 192.168.3.13    # start relaying to the simulator and the robot
python3 tests/fake_robot.py --port 9191 &                   # no robot? a fake one ...
python3 scripts/web_gui.py --mode robot --ip 127.0.0.1:9191 # ... and the GUI driving it
```

| Option | Meaning |
|---|---|
| `--host` | Address to listen on. Default `0.0.0.0`: **anyone on the network can drive the robot**; use `127.0.0.1` to keep it to this machine. |
| `--port` | HTTP port (default 8080). |
| `--mode sim\|robot\|both` | Starting relay mode (default `sim`). `robot` and `both` need `--ip`. |
| `--ip IP[:PORT]` | Robot to connect at start (port 9090 by default). |
| `--fixed` | Hold the simulated torso in the air. |
| `--viewer` | Also open the MuJoCo viewer window. |
| `--motions-dir DIR` | Where the GUI's motion files live (default `motions/`). |
| `--log-level` | Server log level (default `info`). |

Exit code 2 when the port is busy, or when `--mode robot|both` cannot connect to `--ip` (with `--mode sim` a
failed `--ip` only warns). Ctrl+C stops everything and exits.

Open the printed URL in a browser (a tablet on the same Wi-Fi works). The page has a **Sim / Robot / Both**
switch, a big **STOP** button (Esc), walking (hold the d-pad), the robot's built-in actions and dances, servo
sliders with a keyframe recorder, LEDs/volume/sensors, speech and vision. In **Robot** and **Both** mode every
command passes the safety gate first: servo limits, self-collision, a 90 °/s speed cap (also for motion files) and
a 30% battery floor. Walking has a dead-man: if the page loses the server for 1.5 s, the robot stops. The REST API
behind the page is documented at `/api/docs` and in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

### `scripts/view.py`: look at the robot

```bash
python3 scripts/view.py            # free-standing robot on the floor
python3 scripts/view.py --fixed    # torso held 10 cm in the air: joints can move without falling
```

In the viewer:
- **Drag a servo:** open **Control** in the right panel. Values are radians; degrees = rad × 57.3.
- **Push the robot:** hold Ctrl and right-drag.
- **Reset the scene:** press Backspace.
- **See collision shapes:** press 3. To toggle visual meshes, press 2.

### `scripts/play_motion.py`: check and play a motion file

```bash
python3 scripts/play_motion.py MOTION.json [options]
```

| Option | Meaning |
|---|---|
| *(none)* | Check the motion, then play it in the simulator with a viewer. |
| `--fixed` | Simulator: hold the torso in the air (good for arm-only motions). |
| `--headless` | Simulator: no window (fast check only). |
| `--loop` | Simulator: repeat until the window is closed. |
| `--speed 0.5` | Play at half speed (also on the robot; recommended for first tries). |
| `--real --ip IP` | Run on the real robot, but only if every check passes. |
| `--yes` | Real robot: skip the "is it safe?" confirmation. |

Checks run every time:
- **Errors** (block real playback): servo limits, and runtime outside YanAPI's 200–4000 ms window.
- **Warnings**: joints moving faster than 300 °/s.
- **Self-collision sweep** (blocks real playback): the motion is interpolated eight steps per frame, and any two parts of the robot that meet are reported.

On the real robot, it also needs battery ≥ 30% and returns to the reset pose at the end, or on Ctrl+C.

### `scripts/pose_editor.py`: build a motion while watching it

```bash
python3 scripts/pose_editor.py --out motions/hello.json [--fixed]
```

Type commands at the `pose>` prompt:

| Command | What it does |
|---|---|
| `set NAME DEG [NAME DEG ...]` | Move servos, e.g. `set RightShoulderFlex 30 RightElbowFlex 90` |
| `time MS` | Runtime for the next moves and keyframes (default 600) |
| `add [HOLD_MS]` | Save the current pose as a keyframe (only the servos that changed) |
| `undo` | Remove the last keyframe |
| `play` | Replay all keyframes from the reset pose |
| `reset` | Go back to the reset pose |
| `show` | Print the current targets and all keyframes |
| `save` | Check the motion (limits, speed, self-collision) and write the file |
| `quit` | Exit |

An existing file given with `--out` is loaded, so you can extend it.

### `scripts/mirror.py`: live mirroring

```bash
python3 scripts/mirror.py from-robot --ip IP [--rate 4] [--limp-arms]
python3 scripts/mirror.py to-robot [--ip IP] [--rate 4] [--max-speed 90] [--no-reset]
```

See [section 5](#5-live-mirroring-simulator--robot).

### `scripts/run_in_sim.py`: run an existing YanAPI script in the simulator, unchanged

```bash
python3 scripts/run_in_sim.py path/to/script.py [script args]
YANSHEE_SIM_FIXED=1 python3 scripts/run_in_sim.py script.py      # torso held in the air
YANSHEE_SIM_HEADLESS=1 python3 scripts/run_in_sim.py script.py   # no window
```

`import YanAPI` and `from YansheeRobot import YanAPI` are redirected to the simulator.

| YanAPI call | In simulation |
|---|---|
| `yan_api_init`, `set_servos_angles`, `sync_set_servo_rotate`, `get_servos_angles`, `get_servo_angle_value` | Simulated exactly |
| `start_play_motion`, `sync_play_motion`, `stop_play_motion` | `reset` / `Stop` work. Other names play the first file found: `NAME_DIRECTION.json`, then `NAME.json`, in `motions/recorded/`, then `motions/builtin/` (approximate sim versions of `wave`, `raise`, `bow`, ...), then `motions/`. |
| `get_motion_list_value` | Lists available motion files (including `motions/builtin/`) |
| `get_robot_battery_value`, `get_robot_battery_info` | 100% |
| `get_sensors_gyro` | Simulated IMU (frame differs from the real robot; see [section 13](#13-accuracy-and-limits)) |
| `sync_do_tts`, `start_voice_tts`, `set_robot_led`, `set_robot_volume`, `start_play_music`, `stop_play_music` | Printed to the terminal |
| Everything else (vision, speech recognition, other sensors) | Not simulated: returns a failure value of the call's type, with a warning printed once: `-1` for `*_value` calls, `""`, `False`, `[]` or `None` where YanAPI returns those, otherwise `{"code": -1, "data": {}, "msg": "not simulated"}` |

### `scripts/record_real_motion.py`: copy a built-in motion from the robot

```bash
python3 scripts/record_real_motion.py --ip IP --name wave --direction both
python3 scripts/record_real_motion.py --ip IP --name WakaWaka --hz 4 --max-seconds 90
```

The robot plays the motion while all 17 servo angles are sampled, `--hz` times per second
(default 5). The result goes to `motions/recorded/NAME[_DIRECTION].json`, which `play_motion.py`
and `run_in_sim.py` can then replay. Fast moves come out smoothed, because each REST read takes
tens of milliseconds.

### `tools/build_models.py`: regenerate the models

```bash
python3 tools/build_models.py
```

See [section 11](#11-rebuilding-the-model).

### Tests

```bash
python3 -m pytest -q tests          # all of them (headless: MUJOCO_GL=egl)
python3 tests/test_controller.py    # or one file at a time
```

See [section 12](#12-tests).

---

## 5. Live mirroring: simulator ↔ robot

**Yes: connect the robot, move the simulator, and the robot makes the same movement.** It also
works the other way round.

### Robot → simulator: always do this first on new hardware

```bash
python3 scripts/mirror.py from-robot --ip 192.168.3.13
python3 scripts/mirror.py from-robot --ip 192.168.3.13 --limp-arms
```

- The simulator, with its torso held in the air, copies the robot's 17 servo angles 4 times per second.
- Play a built-in motion from the app, or with `--limp-arms`, move the arms and head by hand. `--limp-arms` puts those 7 servos into YanAPI's limp `program` mode and restores `work` mode on exit.
- **Check that every joint turns the same way in both.** This is how you confirm the model matches your robot before sending it any commands.

### Simulator → robot

```bash
python3 scripts/mirror.py to-robot                        # dry run: prints what would be sent
python3 scripts/mirror.py to-robot --ip 192.168.3.13      # the robot follows
```

1. At start, the simulator jumps to the robot's current pose, so nothing moves suddenly.
2. Open **Control** in the viewer's right panel and drag a servo slider.
3. Several times per second (`--rate`, default 4), servos that changed by ≥ 1° are checked:
   - **Limits:** clamped to each servo's range.
   - **Self-collision:** the whole new pose is tested. If two parts of the robot would meet, the command is **not sent** and the terminal shows `NOT sent (self-collision)`.
   - **Speed cap:** the runtime is stretched so no joint moves faster than `--max-speed` (default 90 °/s).
4. Safe changes are sent with `set_servos_angles`. On exit the robot returns to reset (`--no-reset` keeps the pose).

The torso is held in the air in the simulator, so this mode is for **upper-body and posture**
work. A leg move that is safe on a hanging model can still tip a standing robot over. Test leg
motions with `play_motion.py` (free-standing physics) before sending them.

In code:

```python
from yanshee_sim import SimRobot, RealRobot
from yanshee_sim.mirror import SimToRobot

sim = SimRobot(fixed_base=True, manual=True)     # sliders drive the targets
link = SimToRobot(sim, RealRobot("192.168.3.13"), max_speed_deg_s=60)
link.start()
while sim.is_running():
    print(link.step())                           # ("sent", ...) / ("blocked", ...) / ("idle", None)
    time.sleep(0.25)
```

---

## 6. Python API reference

```python
from yanshee_sim import connect, SimRobot, RealRobot, motion, mirror, NAMES, LIMITS, RESET_POSE
```

### `connect(target="sim", ip=None, **kwargs)`

Returns a robot. `connect("sim", viewer=True, fixed_base=False)` or `connect("real", ip="192.168.3.13")`.

### Methods on both robots (same names and return shapes as YanAPI)

| Method | Description |
|---|---|
| `set_servos_angles(angles, runtime=200)` | Move `{name: degrees}` over `runtime` ms (clamped to 200–4000). Returns immediately. Raises `ServoCommandError` for unknown names or out-of-range angles. |
| `sync_set_servo_rotate(angles, runtime=200)` | Same, then waits until the move is done. |
| `get_servos_angles(names=None)` | `{"code": 0, "data": {name: deg}, "msg": "success"}` with measured angles (all 17 if `names` is omitted). |
| `get_servo_angle_value(name)` | One angle as an int. |
| `reset(runtime=1000)` | Move to the standing reset pose and wait. |
| `wait(seconds)` | Sleep. With `SimRobot(realtime=False)` it steps the physics instead. |
| `close()` | Stop the simulator thread and viewer (no-op for the real robot). Robots also work as context managers: `with connect("sim") as bot:`. |

### `SimRobot(viewer=True, realtime=True, fixed_base=False, manual=False, scene=SCENE)`

| Argument | Meaning |
|---|---|
| `viewer` | Open a MuJoCo window. |
| `realtime` | Run physics in a background thread at wall-clock speed. `False` = advance only inside `wait()`, which is fast and good for tests and batch checks. |
| `fixed_base` | Hold the torso 10 cm above the floor. |
| `manual` | Servo targets come from the viewer's Control sliders instead of `set_servos_angles`. |

Extra methods:

| Method | Description |
|---|---|
| `torso_tilt_deg()` | Angle between the torso and vertical; above about 45° means the robot has fallen. |
| `get_sensors_gyro()` | Simulated IMU in YanAPI's shape (gyro rad/s, accel m/s²). |
| `get_target_angles()` | Current servo targets in degrees (the slider positions in manual mode). |
| `jump_to(angles)` | Teleport joints and targets to a pose, with no motion. |
| `is_running()` | False once the viewer window is closed. |

Attributes: `model` and `data` (raw MuJoCo objects), and `strict` (False = clamp out-of-range angles instead of raising).

Also: `set_servos_mode(mode, names)` (`"program"` = limp, kp 0 on those actuators; `"work"` = hold),
`get_servos_mode(names=None)`, `stop_all()` (cancel moves and hold the current pose), `respawn()` (stand a fallen
robot up), `set_led(type, color, mode)` / `get_led()` and the `led` attribute (chest `button` and eye `camera` LEDs).
`SimRobot(spec_hook=fn)` lets `fn(MjSpec)` edit the model before it is compiled.

### `RealRobot(ip, yanapi_dir=None, min_battery=30, timeout=4.0)`

- **On connect:** checks the robot is reachable on port 9090 (3 s timeout) and that battery ≥ `min_battery`.
- **Transport:** `rest`, a `YansheeRest` client (`yanshee_sim/rest.py`, standard library only) with a `timeout` on every request. All the methods above go through it.
- **Attributes:** `battery`, the starting percentage, and `api`, the official YanAPI module for anything else (`bot.api.sync_do_face_recognition("age")`, ...). It is imported on first use and needs the extra libraries in [section 2](#2-installation).
- **`battery_info()`:** `{"percent", "charging", "voltage"}` read now.
- **`set_servos_mode(mode, names)` / `get_servos_mode(names=None)`:** `"program"` (limp, pose by hand) or `"work"`.
- **`play_builtin(name, direction="", speed="normal", repeat=1, version="v1")`:** start a built-in motion (`"wave"`, `"WakaWaka"`, ...); the reply has `data.total_time` in ms. System motions (`Shutdown`, `calibration`, `ActionAging`, ...) are refused unless `allow_system=True`.
- **`stop()`:** stops any built-in motion. **`stop_all()`:** motion v1 and v2, gait, music and speech at once; never raises.

### `yanshee_sim.controller.Controller` (what the web GUI drives)

```python
from yanshee_sim.controller import Controller
with Controller() as ctl:                        # mode "sim"
    ctl.play_action("wave", "right")
    ctl.connect("192.168.3.13"); ctl.set_mode("both")   # simulator and robot together
    ctl.set_servos({"NeckLR": 60}, runtime=800)          # through the safety gate
    ctl.stop_all()
```

Every method returns `{"ok": ...}` and never raises on robot errors (only `set_mode` raises `ModeError`). The full
method list, the relay modes and the safety rules are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) section 2.

### `yanshee_sim.motion`

| Function | Description |
|---|---|
| `load(path)` / `save(motion, path)` | Read or write a motion JSON file. |
| `validate(motion, start_pose=None)` | Returns `(errors, warnings)`: limits, runtimes, speed. |
| `self_collisions(motion, samples_per_frame=8)` | List of `(frame, fraction, (body, body))` contacts during the motion. |
| `play(robot, motion, speed=1.0, on_frame=None)` | Run a motion on any robot, with an optional callback per frame. |
| `duration_s(motion)` | Total playing time in seconds. |
| `CollisionChecker()` | Reusable checker: `.contacts({name: deg})` returns new body pairs in contact (about 0.3 ms). |
| `MAX_SPEED_DEG_S` | Speed warning threshold (300). |

### `yanshee_sim.mirror`

| Class | Description |
|---|---|
| `SimToRobot(sim, robot, max_speed_deg_s=90, deadband_deg=1.0, min_runtime_ms=250)` | `.start()` aligns the sim to the robot. `.step()` sends safe changes and returns `("sent", (angles, runtime))`, `("blocked", pairs)` or `("idle", None)`. |
| `RobotToSim(sim, robot, period_s=0.25)` | `.step()` reads the robot and drives the simulator; returns the angles. |
| `DryRunRobot()` | Prints commands instead of sending them. |

### `yanshee_sim.servos`

`SERVOS` (id, name, min, max), `NAMES`, `IDS`, `LIMITS`, `RESET_POSE`, `MIN_RUNTIME_MS`,
`MAX_RUNTIME_MS`, `check_angles(angles)` (returns a list of problems) and `clamp_angles(angles)`.

### Example

```python
import time
from yanshee_sim import connect, motion

bot = connect("sim")                                   # change to connect("real", ip="...") later
bot.set_servos_angles({"NeckLR": 60, "RightShoulderFlex": 30}, runtime=800)
time.sleep(1.0)
print(bot.get_servos_angles(["NeckLR", "RightShoulderFlex"]))

wave = motion.load("motions/wave_right.json")
errors, warnings = motion.validate(wave)
assert not errors and not motion.self_collisions(wave)
motion.play(bot, wave)
bot.close()
```

---

## 7. Motion files

```json
{
  "name": "wave_right",
  "description": "Raise the right arm and wave",
  "frames": [
    {"angles": {"RightShoulderFlex": 30, "RightElbowFlex": 90}, "runtime": 800, "hold": 100},
    {"angles": {"RightElbowFlex": 130}, "runtime": 300},
    {"angles": {"RightElbowFlex": 80}, "runtime": 300}
  ],
  "end_in_reset": true
}
```

- Each frame is one `set_servos_angles(angles, runtime)`, then a pause of `runtime + hold` ms.
- Servos not named in a frame keep their previous target.
- `runtime` must be 200–4000 ms (YanAPI's range). `hold` is optional.
- `end_in_reset: true` returns to the reset pose at the end.

Included: `wave_right`, `look_around`, `cheer`, `squat`. Recorded built-ins go in `motions/recorded/`.

---

## 8. Servo reference

| ID | Name | Range (deg) | Reset | What it does (in the model) |
|---|---|---|---|---|
| 1 | RightShoulderRoll | 0–180 | 90 | Arm forward/back: 0 = forward, 180 = back |
| 2 | RightShoulderFlex | 0–180 | 140 | Arm out to the side: 140 ≈ down, 90 = horizontal, 0 = overhead |
| 3 | RightElbowFlex | 0–180 | 164 | Elbow |
| 4 | LeftShoulderRoll | 0–180 | 90 | Mirror of 1: 180 = forward, 0 = back |
| 5 | LeftShoulderFlex | 0–180 | 40 | Mirror of 2: 40 ≈ down, 90 = horizontal, 180 = overhead |
| 6 | LeftElbowFlex | 0–180 | 15 | Elbow |
| 7 | RightHipLR | 0–120 | 90 | Leg sideways (roll) |
| 8 | RightHipFB | 10–180 | 60 | Hip forward/back (pitch) |
| 9 | RightKneeFlex | 0–180 | 76 | Knee |
| 10 | RightAnkleFB | 0–180 | 110 | Ankle pitch |
| 11 | RightAnkleUD | 65–180 | 90 | Ankle roll |
| 12 | LeftHipLR | 60–180 | 90 | Leg sideways (roll) |
| 13 | LeftHipFB | 0–170 | 120 | Hip pitch |
| 14 | LeftKneeFlex | 0–180 | 104 | Knee |
| 15 | LeftAnkleFB | 0–180 | 70 | Ankle pitch |
| 16 | LeftAnkleUD | 0–115 | 90 | Ankle roll |
| 17 | NeckLR | 15–165 | 90 | Head turn: below 90 = robot's left, above 90 = robot's right |

- **Frame:** the robot faces +y, its right side is +x, and z is up.
- **Mirroring:** left and right servos are mirror images, so the same movement on the other side is usually `180 − angle`.
- **Squat**, tested upright: right leg `HipFB −d, KneeFlex −2d, AnkleFB +d`, left leg the mirror image. `motions/squat.json` uses d = 20.

---

## 9. From simulation to the real robot: step by step

1. **Safety setup.** Flat floor or table, away from edges, battery > 30%, and someone at the emergency stop on top of the head. Stay within about 1 hour of continuous use (servo life).
2. **Connect.** Robot and laptop on the same 2.4 GHz Wi-Fi. Check with `curl http://IP:9090/v1/devices/battery`.
3. **Verify the mapping once** (new robot, or after recalibration):
   - Run the robot's own reset, from the app or with `bot.api.sync_play_motion("reset")`.
   - `python3 scripts/mirror.py from-robot --ip IP --limp-arms`, then move each arm and the head by hand. The simulator must move the same way.
   - Compare the angles printed after reset with `RESET_POSE` in `yanshee_sim/servos.py`. Update that file if they differ.
4. **Design** with `pose_editor.py`, or write a motion JSON.
5. **Check** in the simulator: `python3 scripts/play_motion.py motions/X.json`. Watch for falls and self-collisions.
6. **First real run, slowly:** `python3 scripts/play_motion.py motions/X.json --real --ip IP --speed 0.5`.
7. **Full speed** once it looks right.

---

## 10. ROS 2

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch yanshee_description display.launch.py                 # RViz and sliders, starting in reset
ros2 launch yanshee_description display.launch.py gui:=false      # another node publishes /joint_states
ros2 launch yanshee_description display.launch.py rviz:=false
```

- **Frames:** `base_link`, then `torso_link` (fixed), then one `<ServoName>_link` per servo, plus `head_camera_optical_frame`.
- **`/joint_states`:** uses the servo names, with positions in **servo radians**. Degrees = rad × 57.2958.
- **Mirroring the real robot in RViz:** publish `get_servos_angles()` converted to radians as `/joint_states`, and launch with `gui:=false`.

---

## 11. Rebuilding the model

`tools/build_models.py` reads `source_model/robot.json` (needs Git LFS) and writes:

- `model/yanshee.xml`, `model/scene.xml`, `model/meshes/*.stl` (MuJoCo)
- `ros2_ws/src/yanshee_description/urdf/yanshee.urdf` and `meshes/` (ROS 2)

Edit and rebuild when you want to change:

| What | Where |
|---|---|
| Servo limits or reset pose | `yanshee_sim/servos.py` |
| Servo strength, damping or armature | `SERVO_KP`, `SERVO_TORQUE`, `SERVO_DAMPING`, `SERVO_ARMATURE` in `tools/build_models.py` |
| Floor, lights or cameras | `SCENE` in `tools/build_models.py` |

Then run `python3 tools/build_models.py && python3 tests/test_models.py`. Don't edit the generated files by hand.

---

## 12. Tests

| Test | Checks |
|---|---|
| `tests/test_models.py` | Reset pose inside limits; URDF and MuJoCo forward kinematics agree (< 1 µm over random poses); reset keyframe; the robot stands for 3 s (< 5° tilt, < 6 mm sag) |
| `tests/test_mirror.py` | Sim → robot sends changes with the speed cap, blocks self-collisions, starts aligned with the robot; robot → sim follows and clamps |
| `tests/test_motion.py` | Motion validation (limits, runtimes, speed, the return to reset), playback and cancelling, the self-collision sweep, `play_motion.py` (headless and against the fake robot) |
| `tests/test_rest.py` | `YansheeRest` against the fake robot: every endpoint, timeouts, refusals, bad replies, the same paths and payloads as the official `YanAPI.py` |
| `tests/test_controller.py` | The relay controller in sim, robot and both modes: safety gate, STOP and mode-switch races, walking dead-man, robot that stops answering, concurrency and a soak (`YANSHEE_SOAK_S` shortens it) |
| `tests/test_server.py` | The web server with FastAPI's `TestClient`: every endpoint, bad input, body limits, the WebSocket, MJPEG streams and the robot-camera proxy, a real uvicorn shutdown |
| `tests/test_yanapi_sim.py` | `run_in_sim.py`'s simulated YanAPI (official example scripts run unchanged, stop/sync playback, failure values), `record_real_motion.py`, `pose_editor.py`, `view.py` |

Run them all with `python3 -m pytest -q tests` (about 3 minutes, headless with `MUJOCO_GL=egl`), or each file with
`python3 tests/test_NAME.py`. `tests/fake_robot.py` is a fake Yanshee REST server used by the robot tests; run it by
hand for GUI work without hardware: `python3 tests/fake_robot.py --port 9191` (`--battery 20`, `--refuse`,
`--slow 5` inject errors).

---

## 13. Accuracy and limits

High fidelity:
- **Kinematics:** joint axes, link lengths, meshes and head camera pose come from UBTECH's CAD-derived Webots model.
- **Matching models:** the URDF and MuJoCo models agree to within 1 µm.
- **Mass:** 2.10 kg (the real robot is about 2.05 kg).
- **Head camera:** 640×480, field of view from a real calibration.

Approximate, and worth tuning against the real robot:
- **Servo model:** gain 8 N·m/rad, 2 N·m limit, damping and armature are assumptions.
  - Arm choreography and slow poses transfer well.
  - Walking, kicks and fast balance-critical moves need tuning first.
  - A laptop cannot do closed-loop balance over the REST API anyway (tens of ms per call).
- **Inertia data:** one link's Webots inertia values were physically invalid. MuJoCo's `balanceinertia` repairs them.
- **IMU:** the simulated IMU uses the model frame. The real robot's raw IMU frame is left-back-up.
- **Not simulated:** vision (except colour detection from the rendered head camera in the web GUI), voice, and the built-in dances. The web GUI's sim LEDs, sim built-ins (`motions/builtin/`) and sim gait are approximations. Built-in motions can be recorded with `record_real_motion.py`.
- **Not yet confirmed on hardware:**
  - the right-arm reset values (mirrored from the left)
  - the joint directions in [section 8](#8-servo-reference)
  - Step 3 of section 9 checks both.

---

## 14. Troubleshooting

| Problem | Fix |
|---|---|
| `cannot reach IP:9090` | Robot off, wrong IP, or a different Wi-Fi. Find the IP in the Yanshee app; `curl http://IP:9090/v1/devices/battery`. |
| `battery N% is below 30%` | Charge. The robot refuses motions near 30% anyway (`lowbattery`). |
| `ServoCommandError: NAME=X outside A-B` | The angle is outside that servo's range ([section 8](#8-servo-reference)). |
| `refusing to send this motion` | Fix the reported errors or self-collisions first. |
| `No module named 'YanAPI'` (only when using `bot.api`) | Set `YANAPI_DIR` to the folder that contains `YanAPI.py`. |
| `No module named 'cv2'` / `nest_asyncio` (only when using `bot.api`) | `pip install opencv-python nest_asyncio requests` |
| `module 'mujoco' has no attribute 'MjSpec'` | MuJoCo is older than 3.3: `pip install -r requirements.txt`. |
| `web_gui.py`: `port 8080 ... is in use` | Another GUI is running; stop it or use `--port 8081`. |
| GUI says "refused: ... cancelled by STOP" | A command was still waiting when STOP or a mode switch came; send it again. |
| `robot.json` is ~130 bytes / the converter fails to parse it | Git LFS pointer: `git lfs install && git lfs pull`. |
| No window appears | You're on a machine without a display. Use `--headless`, or `MUJOCO_GL=egl` for rendering. |
| The robot falls in the simulator | That's the point: the motion is unbalanced. Try `--fixed` to work on the arms, or adjust the legs. |
| Simulator runs slower than real time | It re-syncs automatically. Close other GPU-heavy apps. |
| `mirror.py to-robot` says `NOT sent (self-collision)` | The slider pose would make two parts of the robot touch. Move the slider back. |

---

## 15. Repository layout

| Path | Contents |
|---|---|
| `yanshee_sim/` | Python package: `robot.py` (sim and real backends), `rest.py` (REST client for the robot), `controller.py` (relay controller and safety gate), `sim_extras.py` (renderer, LEDs, sim built-ins and gait, sim battery), `motion.py`, `mirror.py`, `servos.py`, `yanapi_sim.py` (fake YanAPI) |
| `yanshee_sim/web/` | Web GUI: `server.py` (FastAPI) and `static/` (the page: `index.html`, `app.js`, `style.css`) |
| `scripts/` | `web_gui.py`, `view.py`, `play_motion.py`, `pose_editor.py`, `mirror.py`, `run_in_sim.py`, `record_real_motion.py` |
| `model/` | Generated MuJoCo model (`scene.xml` is the entry point) |
| `ros2_ws/src/yanshee_description/` | ROS 2 package: URDF, meshes, launch, RViz config |
| `motions/` | Motion files; `recorded/` holds motions copied from the robot, `builtin/` the approximate sim versions of the robot's parametric built-ins |
| `requirements.txt` | Python dependencies |
| `tools/build_models.py` | Webots JSON → MJCF + URDF + STL converter |
| `source_model/robot.json` | Webots Yanshee model (Git LFS) from [Yanshee-Serenade/Yanshee-RL-Unity](https://github.com/Yanshee-Serenade/Yanshee-RL-Unity). That repository has no license file, so ask its authors before redistributing. |
| `tests/` | Tests (see [section 12](#12-tests)), `fake_robot.py` (fake Yanshee REST server) and `data/` (the motion list captured from a real robot) |
| `knowledge/` | Yanshee knowledge base, course digest and online resources |
| `docs/` | `ARCHITECTURE.md` (design of the relay controller, REST client, web server and GUI) and images used in this README |
| `GIT_GUIDE.md` | How to commit and push to this repo (multi-account setup, helper commands) |
