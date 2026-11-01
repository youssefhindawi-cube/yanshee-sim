# Plan for the first session with the robot on the network

Everything in the simulator was built from documents and from UBTECH's model; nothing has been measured on a
Yanshee yet. This is the order to do it in, what each step settles, and what to change afterwards. Budget
about two hours of robot time; the servos are rated for about an hour of continuous use per session, so take
breaks.

## 0. Before touching the robot

- Charge it above 50 % (motions are refused under 30 %).
- Laptop and robot on the same 2.4 GHz Wi-Fi: the first connection goes through the Yanshee app over Bluetooth
  (the published field card "Yanshee Wi-Fi Setup" has the steps, the IP methods and the failure table; bring your
  own router or hotspot, keep it offline during the show). The app shows the robot's IP; or scan from the laptop:
  `python3 -c "import socket;[print(f'172.20.10.{i}') for i in range(1,15) if socket.socket().connect_ex((f'172.20.10.{i}',9090))==0]"`
  (replace the prefix with your network's).
- `python3 tools/robot_checks.py latency --ip IP` — confirms REST works and tells how slow it is (ms per
  call). Write the numbers into README section 13 ("tens of ms per call" is a guess).
- Keep a hand near the red button on the robot's head: it is the emergency stop.

## 1. Read the Raspberry Pi (read-only, 10 minutes)

`python3 tools/robot_checks.py inspect --ip IP` (ssh `pi`/`raspberry`; nothing is written on the robot).

It saves `robot_dump/inspect_<time>/inspect.txt` and copies every `.hts` file it finds. What to look for:

| In the dump | Why |
|---|---|
| `/etc/ubt_version.txt`, `dpkg -l` | firmware version; the docs describe up to v2.9 |
| the `.hts` files (system folder `/mnt/1xrobot/res/hts` and user folder `/home/pi/Documents/motions`) | the robot's own built-in motions: play them in the sim with `python3 scripts/play_motion.py robot_dump/.../FILE.hts`; copy the useful ones into `motions/recorded/` so the GUI offers them |
| anything named `gait` / `motion_planning` | the closed-source gait controller: its config files may give the period/speed mapping |
| anything named `mpu` / `imu` / `gyro` | IMU driver and its axis configuration |
| `rosservice list` / `rostopic list` | confirms `/hal_angles_set` (50 Hz servo input) and `hal_gyro_report` exist for a faster path than REST |

## 2. Joint directions and the reset pose (README section 5)

```bash
python3 scripts/mirror.py from-robot --ip IP --limp-arms
```
Move each limb by hand and watch the simulator copy it. Every joint must turn the same way; the ankle pitch
(`RightAnkleFB`/`LeftAnkleFB`) matters most, because UBTECH's self-balancing sample moves it the opposite way
to what the sim needs (README section 13). Then run the robot's built-in `reset` and read the servos:
`python3 -c "from yanshee_sim import RealRobot; print(RealRobot('IP').get_servos_angles())"` after
`play_builtin('reset')`. Update `RESET_POSE` in `yanshee_sim/servos.py` if it differs (the right arm 140/165
and the ankle rolls are the least certain), then `python3 tools/build_models.py && python3 tests/test_models.py`.

## 3. IMU conventions (5 minutes)

`python3 tools/robot_checks.py imu --ip IP` — it asks for five postures (upright, leaning back, leaning right,
face down, on the back) and prints the mean readings and what the sim assumes. Change `IMU_UPRIGHT_EULER_X`
and, if a sign differs, the axis maps in `yanshee_sim/robot.py` (`get_sensors_gyro`). The test
`test_sim_imu_fall_convention` in `tests/test_models.py` encodes the current assumptions; update it with the
real numbers.

## 4. Servo speed and lag (5 minutes)

`python3 tools/robot_checks.py servo --ip IP` — steps the right shoulder from 140 to 90 degrees in the minimum
200 ms and reads it back as fast as REST allows. It prints the command latency, the first-change time, the
time to arrive and the resulting deg/s, plus how many readbacks per second you get. Then:

- set `MAX_DEG_S` in `SimGait` (`yanshee_sim/sim_extras.py`) to the measured speed and rerun
  `python3 tools/tune_gait.py` (the current 400 deg/s is an assumption; at 300 the fastest cycle falls);
- if the servo clearly lags or overshoots, adjust `SERVO_KP` / `SERVO_DAMPING` in `tools/build_models.py` so
  the sim's `set_servos_angles(..., 200)` step looks the same, rebuild, and rerun UBTECH's 2018 walk frames
  (they walk in the sim at 400 ms per frame but not at 240 ms: a faster real servo would explain that).

## 5. The built-in gait (15 minutes)

`python3 tools/robot_checks.py gait --ip IP` — for each `speed_v, period` setting it starts
`control_motion_gait` for 10 steps, times it, and asks for the distance you measured with a tape. This gives
seconds per step and cm per step per setting, which nobody has published. Then make the sim's walk match:
`CYCLE`, `CYCLE_PER_PERIOD` and `STRIDE` in `SimGait` (two steps per cycle; `speed_v 5` should give the measured
cm/s), keeping `python3 tools/tune_gait.py --check` passing.

Also note how the robot walks (does it sway, does it lift the feet, step length) and film it from the side
and front: the sim gait is a guess and can be reshaped to look like the real one.

### 5a. How the mobile app drives the gait (captured 29 Oct 2026, `robot_dump/app_capture/`)

The app uses the same REST API: walking is `PUT /v1/motions/gait` with `speed_v` +-2 (always 2 in the capture),
`period` 1 and `wave` true, sent once per stick movement; it stops with the same call and every value 0. Left/right on
its joystick does not side-step: it turns in place with `PUT /v1/motions` (TurnLeft/TurnRight, `operation`
"start_async", `repeat` 0), resent about every 0.4 s while held, and the robot then plays its own ResetSwitch.
`tools/joystick.py` drives the robot the same way (side steps behind a checkbox). Capture method: ssh to the robot,
`rostopic echo -p /gait/goal` and `/rosout` (every motion played is logged), and `sudo tcpdump` on port 9090.

### 5b. Stopping without tipping over

A walk that is frozen mid-step at speed can tip the robot: in the sim, a 0.4 s fade-out at full speed fell in
half the cases and a 1 s fade-out never did (`SimGait.STOP_S` is now 1 s). Nobody knows how the firmware
decelerates on `exit_motion_gait`, so the software stops the robot the gentle way and the session checks it:

- The gentle stop is `rest.gait_decelerate()`: the speeds are stepped down (`5 → 3 → 1`, 0.5 s apart) with
  `control_motion_gait` updates, then `exit_motion_gait`. Releasing the GUI's D-pad, `gait_stop()` and a lost
  dead-man lease all use it. **STOP (the red button in the GUI and the one on the robot's head) exits at
  once** — use it for emergencies and expect the robot may need catching.
- Test it before any fast walking: `python3 tools/robot_checks.py gait --ip IP --stop-test` walks 3 s at
  `speed_v 3`, stops gently, then repeats with a sudden exit; you answer whether it stayed up and the answers
  are saved. Only move on to `--stop-speed 5` if speed 3 survived both. Stand behind the robot with a hand
  ready at its shoulders: a stop at speed tips it forward.
- If the sudden exit is fine on the real robot, the firmware already decelerates and `stage_s` in
  `rest.gait_decelerate` can be shortened; if even the gentle stop tips it, add a stage (`2`) or lengthen
  `stage_s` and re-test.
- When the gait is recorded with `record_real_motion.py`, record the stop too: the sim's `STOP_S` should
  match how long the real robot takes to settle.

### 5c. Turning

The robot's gait cannot turn; it turns with keyframe motions (TurnLeft/TurnRight, TurnLeftSlowly, TurnL_tiny,
OneStepTurnLeft, "turn around"), which UBTECH's own files show to be a stance-foot pivot with the rear leg swept and
the sole rolled onto its edge. Play each one N times from a floor mark, read the IMU's euler-z before and after,
log `total_time`, and note the floor. That gives degrees per run and per second (expect roughly 5–9 °/s); the
sim's pivot does 10–11 °/s (`SimGait.TURN_DEG`, `TURN_LIFT`), so match it to the real figure and pick the motion
the Rotate buttons should map to.

### 5d. Arms while walking (layers)

Today a gesture during a walk cancels the walk (one activity at a time); only the gait's own arm-swing flag works
mid-walk. The firmware's "layers" engine (v2 motions with head/hand/foot tracks, firmware ≥ 2.4;
`set_servos_angles_layers`) is the intended way to combine them, but what happens on conflicts is undocumented.
Measure, with the gait running at speed 3: a plain `set_servos_angles` on two arm servos (per-servo true/false,
readback 0.5 s later, gait state); the same with `set_servos_angles_layers`; a `H_*` motion with `version="v2"`
and `GET /v1/motions/all`; a head move during a v1 "wave"; two v2 motions at once and `stop_play_motion("",
version="v2")`; and, with the robot held, one leg servo command during the gait. The results decide the controller's
servo-group lanes (design in docs/ARCHITECTURE.md).

The full checkpoint protocol, including every sensor and peripheral (camera, microphone, speaker, LEDs, buttons,
add-on sensors, Bluetooth gamepad, HDMI/USB, energy saving, OTA) and the endurance session, is the published page
"Yanshee Hardware Test Plan".

## 6. Motions both ways

- Robot → sim: the `.hts` files from step 1 are exact. For dances or motions that only exist as names, record
  them: `python3 scripts/record_real_motion.py --ip IP --name WakaWaka` (5 Hz readback, smoothed).
- Sim → robot, through the safety gate: `python3 scripts/play_motion.py motions/wave_right.json --ip IP`
  (it checks limits, self-collision and the speed cap first). Start with arm motions, then `squat`, then a
  gait made in the pose editor.
- Web GUI with both: `python3 scripts/web_gui.py --mode both --ip IP`, then the Servos & recorder tab with the
  arms limp to pose by hand.

## 7. Afterwards (laptop only)

- `python3 -m pytest -q tests` and fix what the new constants broke.
- Put the measured numbers into README section 13 and `docs/ARCHITECTURE.md`, and commit the `robot_dump/`
  JSON files (not the copied `.hts` files unless you want them in the repo).
- Record the firmware version, the robot's IP and anything surprising in the memory notes for the next session.

## To do

Done on 29 Oct 2026 from the walking session's logs (`docs/SIM2REAL_FINDINGS.md`), still to check on the robot:

- **Settle gate (AppWalk)**: a walk start, a turn start and a walk<->turn switch now wait until the robot has settled
  after the last stop (at least 1.2 s, IMU within 3 deg of standing for 0.5 s, at most 2.5 s, no ResetSwitch playing).
  Check: walk, release, press again at once, and switch walk -> turn -> walk, 10 times each; `tools/walk_stats.py`
  should show no falls around the switches and the robot log no 11-38 deg lean after restarts.
- **Auto get-up waits for the firmware's get-up**: with fall management on, hal stands the robot up by itself; ours now
  waits (up to 30 s) and only acts if it is still down. Check: Auto get-up on, a fall on its back and one face down
  (smooth floor), both with fall management on and off.
- **Decide fall management for the show**: on, hal plays GetupFront face down (fails on mats, works on a smooth hard
  floor) and GetupRear on the back; off, our auto get-up plays Base_GetupF + reset / GetupRear. Pick after the check.
- **Encoder reads pause while hal plays a motion**: check the robot log shows `enc_pause` during a get-up and no
  "buffer is full" warnings.
- **Shadow turns and timing**: the shadow now plays the robot's .hts motions (turns, ResetSwitch) and schedules the
  firmware's frames at the robot's own times; check `tools/sim2real_report.py` turns show a heading in the shadow.
- **Model refit**: joint damping 0.12, gear friction 0.08, contact friction 0.8 (`tools/build_models.py`), fitted by
  replaying the 17 AppWalk walks (`tools/sim2real_replay.py`: falls agree 13 -> 16 of 17, pitch 7.6 -> 5.9 deg, roll
  4.2 -> 3.1 deg) with the get-up fits kept. One session on one floor: validate on new sessions, and on the show floor.

- **Copy the demo's robot log** (29 Oct 2026 19:38, 1 min: 4 AppWalk walks, 2 turns; the robot left the Wi-Fi before
  the GUI could copy it): `python3 tools/fetch_robot_logs.py --ip 192.168.89.74` (`robot_20261029_193840.jsonl`).
- **The robot drops off the Wi-Fi**: twice on 29 Oct, about a minute after the link started (once idle, once after 4
  walks); its own log kept running through the first drop, so the Pi stayed up and only the Wi-Fi went. Check on the
  Pi: Wi-Fi power saving (`iw dev wlan0 get power_save`), signal (`iwconfig wlan0`), which saved networks it can join
  (`wpa_cli list_networks`). For the show: a dedicated access point next to the stand.
- **Latency baseline, then each fix** (`tools/latency_test.py`, docs/SIM2REAL_FINDINGS.md "Link latency"):
  `--seconds 300 --label baseline`; then `--power-save off --label "power save off"` (lasts until the robot
  restarts; make it permanent if it helps); then on the travel router with the laptop wired; once, with the GUI
  disconnected and the robot standing, `--move` (command -> arm servo) and `--relay` (REST through the link: the
  REST server's own time). Compare: `--compare robot_dump/latency_test_*.json`.
- **Get up ends in the reset pose** (1 Nov 2026, tested in the simulator and against the fake robot): after the
  get-up motion (and Base_GetupF's reset) the Reset button's own move; a leg left splayed (hip roll more than 4 deg
  out, not stall-locked) is stepped in: weight onto the other leg with both ankle rolls (10 deg), the free leg
  lifted (hip pitch 14 deg), its hip roll back, put down, reset pose, every move slow (2.2x); it stops if the IMU
  says it tipped. Check on a smooth hard floor, face down and on its back, a hand near the robot: it ends standing
  straight (encoders within 3 deg of the reset pose); a stepping-in event if a leg was out.
- **Robot watchdog and link reconnect** (1 Nov 2026, tested only against a fake ROS): with the GUI in Robot mode,
  walk forward slowly on a clear floor, one person ready to catch it, and turn the laptop's Wi-Fi off: the robot
  should stop by itself 1.5 s later (the app's stop, then its usual wind-down); the robot's log gets a `watchdog`
  row. The same holding TurnLeft. Wi-Fi back on: the page shows "link lost, reconnecting", then the event "robot
  link is back" within about 10 s. Also check the walk commands now go through the link (`relay` rows in the
  robot's log) and the walk still starts and stops as before.

Still open:

- **Record the AppWalk directions the simulator is missing** (Sim mode refuses them until then): with the GUI in Both
  mode, walk backward, right and the four diagonals for 5 s each, a few times, stopping at different moments; then
  `python3 tools/extract_appwalk.py` (the GUI now logs every walk command with its speed and direction). Also walk at
  the speed used for the show, and measure how far 5 s forward takes the robot (the simulator: 65 cm).
- **Turning in place in the simulator**: settled for TurnLeft on 1 Nov 2026 (contact friction 0.8 -> 0.7: it no
  longer falls, leans up to 21 deg as the robot does, docs/SIM2REAL_FINDINGS.md "Turning and IMU balance"); still to
  compare the turn rate on more robot turns (5 recorded so far, 15-25 deg/s TurnLeft).
- **Carry-over between walks in the replay**: each walk is replayed from rest, so a walk started while the robot was
  still swinging (walk 12) cannot match; replay a whole session continuously.
- **IMU balance on the robot** (built 1 Nov 2026, `yanshee_sim/balance.py`, GUI "Balance"; tested only in the
  simulator and against a fake ROS): first `python3 tools/robot_checks.py balance --ip IP` (robot standing: two 3 deg
  ankle moves; the robot's balance stays off unless both lean the torso the way they do in the simulator, which also
  settles B3). Then, Robot mode, Balance off and on, the same forward walks (5 s, a restart 1 s after a stop, a gentle
  push from behind with a finger): `tools/sim2real_report.py` tilt per walk, `bal` rows (the offsets) and `frame` rows
  with src `balance` in the robot's log; watch for hal warnings ("Read servo angle status timeout") with the extra
  frames. Turns with the balance on are played by the node (frames with src `turn`, `turn` rows): first hold
  TurnLeft 2 s with Balance off (hal plays it) and on (the node plays it), the robot on a smooth floor and someone
  ready to catch it; compare yaw and tilt (robot log IMU), then longer turns (4-6 s, where TurnLeft fell once).
  Check the release plays ResetSwitch and STOP halts a node turn at once.
- **Face-down get-up on a mat** (parked 29 Oct 2026): findings, attempts and leads in `docs/GETUP_ON_MATS.md`.
  Until it is solved the stand needs a smooth hard surface.
- **More sim-to-real data**: every session with the robot connected logs on board (robot clock,
  `/home/pi/yanshee_logs/`, copied to `robot_dump/robot_logs/`; `tools/fetch_robot_logs.py`) and the shadow on the
  laptop (`robot_dump/sim2real_<session>.jsonl`); collect AppWalk sessions on battery (the charger adds about 0.25 V)
  and on the show floor, then rerun `tools/sim2real_replay.py --sweep` on all of them. Known: hal ignores the head on
  `/hal_angles_set`; encoders read whole degrees and settle about 1 deg short of a target.
- **SwayStep** is archived on the robot (29 Oct 2026) and in the simulator (1 Nov 2026,
  `docs/archive/SWAYSTEP_ON_ROBOT.md`); AppWalk is the walk to build on, and the simulator plays the robot's.
- One get-up log (Base_GetupF 13:20, on the mat) still disagrees: the simulator stands, the robot did not.

## What this still cannot settle

- The gait generator stays closed source: the sim's walk only matches its speeds, not its servo trajectories,
  unless you record one with `record_real_motion.py` during a `gait` run (it polls at 5 Hz, enough for the
  shape of a 0.8 s cycle).
- Servo torque: only the stall figures of the servo family are known (8-12 kg·cm). The sim's 2 N·m limit is
  a guess; a robot that fails to stand up from a squat would show it is too high.
