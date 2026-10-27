# Plan for the first session with the robot on the network

Everything in the simulator was built from documents and from UBTECH's model; nothing has been measured on a
Yanshee yet. This is the order to do it in, what each step settles, and what to change afterwards. Budget
about two hours of robot time; the servos are rated for about an hour of continuous use per session, so take
breaks.

## 0. Before touching the robot

- Charge it above 50 % (motions are refused under 30 %).
- Laptop and robot on the same Wi-Fi. The app shows the robot's IP; or scan from the laptop:
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

## What this still cannot settle

- The gait generator stays closed source: the sim's walk only matches its speeds, not its servo trajectories,
  unless you record one with `record_real_motion.py` during a `gait` run (it polls at 5 Hz, enough for the
  shape of a 0.8 s cycle).
- Servo torque: only the stall figures of the servo family are known (8-12 kg·cm). The sim's 2 N·m limit is
  a guess; a robot that fails to stand up from a squat would show it is too high.
