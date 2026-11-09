# Handover: yanshee_sim

Everything a new engineer needs to pick this repository up: what it is for, what to install, how to run it in
simulation and on the real robot, where the knowledge lives, what is known to bite, and what is still open.
Written 9 Nov 2026. The companion repository (speech, Arabic recognition, Q's brain) has its own
`llm_model/HANDOVER.md`; `PROJECT_HANDOVER.md` here is the project-wide index (milestones, folders, rules).

---

## 1. What this is and why it exists

A MuJoCo simulator, a web GUI and sim-to-real tools for the **UBTECH Yanshee** humanoid (17 servos, Raspberry Pi 3B,
YanAPI REST on port 9090). It was built to prepare the robot for a **company exhibition on Monday 7 December 2026**
(Cube Consultants). The milestones agreed on 28 Oct 2026:

| Milestone | Date | Content |
|---|---|---|
| M1 (protected) | Fri 20 Nov 2026 | Introduction presentation, basic movements, interaction, face recognition of enrolled team members |
| M2 | Fri 27 Nov 2026 | M1 + the company AI ("Q") answering spoken or typed questions through the robot (see `llm_model`) |
| M3 (cut first) | Fri 4 Dec 2026 | M2 + replies shown as visuals on a screen next to the robot |

The repository lets you do everything on the simulator first, then run the same command on the robot through a
safety gate. The one rule of the codebase: **servo names, angles and limits are the same as YanAPI**, so a motion
file or script runs unchanged in sim and on the robot.

Planning pages (private artifacts, share from the page if needed):

- Hardware test plan, ticked live during robot sessions: https://claude.ai/artifact/PMgMatZwBzEmuRWtiMM3sC
- High-level exhibition plan: https://claude.ai/artifact/X2cmA5CbkPJYvduR9iYaiz
- Technical plan: https://claude.ai/artifact/377NDYzeH5PsaXg2iiTnCW
- Wi-Fi setup card, handling guide, fall-recovery test: https://claude.ai/artifact/BqDEeddY2eqashYtfKQ4Yr ,
  https://claude.ai/artifact/DneKcUR3E36bps9Fqiim8j , https://claude.ai/artifact/WCCUDnLxZ5tTT2AT7L3kjk

---

## 2. Requirements

### Laptop

| Need | Details |
|---|---|
| OS / Python | Linux (developed on Ubuntu, Python 3.10.12). Windows is untested. |
| Python packages | `pip install -r requirements.txt`: `mujoco>=3.3,<4` (uses the `MjSpec` API; tested with 3.8), `numpy`, `fastapi`, `uvicorn`, `websockets`, `Pillow`, `httpx` (tests only), `requests`, `nest_asyncio`, `opencv-python` (only for the official `YanAPI.py`). |
| Git LFS | `git lfs install` before cloning: `source_model/robot.json` (the Webots model) is stored in LFS. Only needed to rebuild `model/`, which is already generated. |
| Headless machine | `MUJOCO_GL=egl` for rendering without a display. |
| Optional | `paramiko` (`tools/robot_checks.py inspect`), `sshpass` (the robot tools ssh into the Pi), RealVNC Connect Viewer (`tools/find_robot.py --vnc`, package `realvnc-rvncconnect-viewer`), ROS 2 Humble (`ros2_ws/`). |
| Hardware | Any laptop runs the sim. A GPU is not needed. |

`pytest` here must be run as `python3 -m pytest -q tests`; `tests/pytest.ini` disables two unrelated plugins that
live in the user's site-packages (`-p no:dash -p no:anyio`). Keep that line if you move machines and see pytest die at
startup.

### The robot

| Item | Value |
|---|---|
| Model | UBTECH Yanshee, SN AAH001UBT60000030, core firmware 2.8.0.55, servo firmware dc.24.4.12 |
| Computer | Raspberry Pi 3B, Raspbian 9 (Stretch), kernel 4.14, **Python 3.5 and 2.7**, ROS Kinetic, STM32 for the servos |
| Services | REST (YanAPI) on `:9090` (Swagger at `/v1/ui`), Jupyter `:8888`, MJPEG camera `:8000`, RealVNC `:5900`, ssh |
| Login | ssh `pi` / `raspberry`, passwordless sudo. Scripts use `SSHPASS=raspberry sshpass -e ssh pi@IP`. |
| Network | 2.4 GHz Wi-Fi only, DHCP, the IP changes per network. MAC `b8:27:eb:20:d3:af`. Office network "CubeConsultants": it has been .74, .137 and .99 on 192.168.89.x. |
| Battery | Motions are refused below about 30 % (`lowbattery`, code 6). Charger does not block servos. |
| Motions on board | `.hts` files; user motions in `/home/pi/Documents/motions/`. No REST download of motions (use `tools/robot_checks.py inspect`). |
| Boot settings we installed | `tools/robot_setup.py --ip IP` installed a service that turns Wi-Fi power saving off and syncs the clock with NTP at boot (`--status`, `--remove`). |
| Our node on the robot | `yanshee_sim/robot_stream.py` and `balance.py` are copied to the Pi and run under its Python 2 ROS when the GUI connects. Anything that runs on the robot must stay Python 2/3 compatible. |

---

## 3. How to run it

All commands from the repository root.

### Simulator only (no robot)

```bash
pip install -r requirements.txt
python3 scripts/web_gui.py --host 127.0.0.1        # http://localhost:8080/ (this machine only)
python3 scripts/web_gui.py --viewer                # also the MuJoCo 3D window on the laptop
python3 scripts/view.py --fixed                    # look at the model, drag joints, torso held in the air
python3 scripts/play_motion.py motions/wave_right.json       # validate a motion (limits, speed, self-collision) and play it
python3 scripts/pose_editor.py --out motions/hello.json      # build a motion keyframe by keyframe
python3 tests/fake_robot.py                        # a fake Yanshee REST server, then web_gui.py --ip 127.0.0.1:9191 --mode robot
python3 -m pytest -q tests                         # 201 tests, about 7 minutes headless (MUJOCO_GL=egl)
```

The GUI has Sim, Robot and Both modes, walking with speed ramp, the robot's built-in motions and dances, a pose
panel, keyframe recording, IMU display, speech, camera streams, Get up, Free stalled servos and a Balance toggle.
Every robot command passes the safety gate (limits, self-collision, speed cap, battery, gentle stops).

### With the robot

```bash
python3 tools/find_robot.py                        # finds the robot by MAC, confirms REST + ssh, remembers ~/.yanshee_robot_ip
python3 tools/find_robot.py --vnc                  # also opens its desktop in RealVNC Viewer (login pi)
python3 tools/robot_setup.py --ip IP --status      # boot service present? (power saving off, clock synced)
python3 scripts/mirror.py from-robot --ip IP       # FIRST on any new or recalibrated robot: the sim copies the robot
python3 scripts/web_gui.py --ip IP                 # GUI connected; switch to Robot or Both in the page
python3 scripts/play_motion.py motions/wave_right.json --real --ip IP --speed 0.5   # a checked motion, slowly
python3 tools/latency_test.py --ip IP              # ping, link, REST round trips, Wi-Fi details
python3 tools/robot_checks.py latency --ip IP      # the measurements in docs/ROBOT_SESSION_PLAN.md
python3 tools/fetch_robot_logs.py --ip IP          # copy /home/pi/yanshee_logs/ to robot_dump/robot_logs/
python3 tools/sim2real_report.py                   # compare the robot's log with the sim shadow of the session
```

Desktop launchers on the development laptop: `~/Desktop/Find Yanshee Robot.desktop` and `Yanshee Robot Screen.desktop`.

### A robot session, step by step

1. Power the robot on a smooth hard floor (face-down get-up fails on soft or grippy mats). Battery above 40 %.
2. `tools/find_robot.py`, then `robot_setup.py --status`. If the service is missing, install it again.
3. `mirror.py from-robot` once, to check the sim and robot agree on the reset pose.
4. Start the GUI with `--ip`. In Robot mode, test a wave before any walk.
5. Walking and turning: **someone stands ready to catch the robot**, always. Do not move the robot while it is charging
   or when nobody is next to it.
6. After the session: `fetch_robot_logs.py`, run `sim2real_report.py`, tick the test plan, write findings into
   `docs/SIM2REAL_FINDINGS.md`.
7. The robot does not return to a known state by itself after a stall. If a servo locks (every command returns
   false, reset fails with code 1), hold the robot and press **Free stalled servos** in the GUI.

Environment flags you may need: `YANSHEE_ROBOT_BALANCE=1` (re-enable the parked IMU balance on the robot),
`YANSHEE_ALLOW_SWAY=1` (the archived SwayStep gait), `YANSHEE_SIM_HEADLESS=1`, `YANSHEE_SIM_FIXED=1`,
`YANSHEE_DUMP_DIR`, `YANSHEE_WALK_STATS`, `YANSHEE_SOAK_S` (shorter controller soak test), `YANAPI_DIR`.

---

## 4. Where to look

Read in this order:

1. `README.md`: the complete user manual (16 sections: GUI, every script, Python API, motion files, servo table,
   ROS 2, model rebuild, tests, troubleshooting, repository layout).
2. `docs/ARCHITECTURE.md`: design of the relay controller, REST client, robot link node, web server and page.
3. `docs/SIM2REAL_FINDINGS.md`: every measurement on the real robot and what changed in the sim because of it
   (link latency, Wi-Fi, walking tests of 2 Nov, the balance fall, the delay model, the hip stall, the speed ramp).
4. `docs/ROBOT_SESSION_PLAN.md`: the measurement protocol and the **To do** section (what is done, what is open).
5. `docs/GETUP_ON_MATS.md`, `docs/archive/SWAYSTEP_ON_ROBOT.md`: parked problems with their evidence.
6. `knowledge/`: digest of the Yanshee manual and the 411-page course, online resources (which public repos are real).
7. `GIT_GUIDE.md`: the three-GitHub-account setup of the development laptop. Not needed on your own machine.

Code map (package `yanshee_sim/`):

| File | Role |
|---|---|
| `robot.py` | `SimRobot` and `RealRobot` with the same methods as YanAPI; emulated IMU; servo delay 60 ms and IMU latency 20 ms modelled from the robot |
| `rest.py` | REST client (standard library only), gentle gait stop, relay of selected calls through the ssh link |
| `controller.py` | The relay controller the GUI drives: safety gate, modes, walking with speed ramp (`RAMP_*`), get-up (stand wide then step in), `free_stalled_servos`, link keeper and reconnect, balance switch |
| `robot_stream.py` | The node that runs **on the robot** (Python 2): queued telemetry, REST relay, heartbeat, 1.5 s watchdog that stops a walk when the laptop goes quiet, plays node-side turns and balance frames |
| `balance.py` | IMU ankle balance (shared by sim and robot node); gains retuned on the delay model; parked on the robot |
| `appwalk.py` | Plays the robot's own recorded firmware walk in the sim (`model/appwalk.json`), per-speed tables, ramp |
| `recovery.py` | Fall detection, get-up sequences, `recenter_steps` |
| `motion.py`, `hts.py` | JSON motions and the robot's `.hts` format |
| `mirror.py`, `servos.py`, `sim_extras.py`, `yanapi_sim.py` | Live mirroring, servo table, renderer/LEDs/sim built-ins, fake YanAPI for `run_in_sim.py` |
| `web/` | FastAPI server and the single-page GUI (`static/index.html`, `app.js`, `style.css`) |

Tools worth knowing: `tools/extract_appwalk.py` (robot log to walk tables), `tools/sim2real_replay.py` (replay robot
walks in the sim, `--sweep` for model fitting), `tools/build_models.py` (Webots JSON to MJCF, URDF, STL),
`tools/getup_lab.py`, `tools/record_gait_topics.py` (ROS topic recorder on the robot, parked task).

Data: `robot_dump/` holds every measurement and robot log (24 MB, **git-ignored on purpose**: it contains UBTECH
vision source and office photos). Ask the previous owner for a copy; nothing in the code requires it.

---

## 5. Things that bite (read before touching the robot)

- **Never call the hal write services** on the robot: `hal_write_sn`, `hal_upgrade`, `hal_servo_calibration`,
  `hal_servo_offset_write`, `hal_shutdown_set`. They change the firmware's identity or calibration.
- **Never start face or object tracking over REST during a show.** It deadlocked the REST server once; recovery was
  `sudo systemctl restart ubtedu-ros` with the robot held.
- **The REST server is single-threaded Python 2** and costs 56 to 77 ms per call on the Pi. Fast things (walk frames,
  the watchdog) go through our ssh link node, not REST.
- **`pkill -f PATTERN` matched its own command line twice** and killed the launcher. Use pid files (`launch_command`
  in `robot_stream.py`) or anchor the pattern (`pgrep -f "^python3 scripts/web_gui.py"`).
- **Fast motions must be played by the robot's engine** (`.hts` by name). Frame-by-frame REST replay starts each
  frame about 0.2 s late and changes the outcome.
- **Servo stall lock**: driving a servo into an obstacle latches it (wrong readback, commands return false, reset fails).
  Recover with Free stalled servos (program mode, work mode, reset). The hips lock when the reset drags loaded feet
  together after a face-down get-up, which is why the get-up now stands wide and steps in.
- **Speed 5 from standstill** made the robot lurch 15 to 20 degrees back and fall 4 times out of 9. The ramp (start 2,
  +1 every 0.5 s, ramp down before stop) fixed it. Keep the ramp.
- **IMU balance on the robot is parked** (it fell on its back in 1.6 s: the correction lagged the servos by 76 to
  124 ms plus the IMU's 53 ms and amplified the sway). The sim now has those delays and the gains are a quarter of the
  old ones. Next step is a standing push test with `YANSHEE_ROBOT_BALANCE=1`, not a walk.
- **Wi-Fi drops are the channel**, not the Pi: channel 6 is crowded in the office (12 % loss, 2 s stalls). For the
  show use a dedicated router on channel 1 or 11, the laptop wired, **no internet uplink** (four updater processes on
  the robot phone home over HTTPS). Bluetooth or Wi-Fi 6/7 dongles will not help: the Pi 3B's radio is the limit.
- **Encoders read whole degrees** and settle about 1 degree short; hal ignores the head on `/hal_angles_set`.
- **Built-in speech recognition is Chinese-only**; English TTS works. That is why speech lives in `llm_model`.
- **Face-down get-up fails on mats** (pikes forward). Smooth hard floor for the stand.
- The app-made `.hts` files contain `LeftHipFB = 171`, one past the documented limit; `motion["clamped"]` tells you.
- Keep sessions under about an hour of continuous servo use (manual's recommendation).

---

## 6. Open work, in priority order for the show

From `docs/ROBOT_SESSION_PLAN.md` (To do) and the test plan artifact:

1. **Show network**: travel router on channel 1 or 11, laptop wired, repeat `tools/latency_test.py --seconds 300`.
2. **Face-down get-ups**: more trials watching the hip-roll encoders after `Base_GetupF`; confirm the stand-wide and
   step-in sequence never locks.
3. **Standing push test of the retuned balance** (`YANSHEE_ROBOT_BALANCE=1`), then a 3 s balanced walk with a catcher.
4. **More AppWalk recordings**: backward-left never settled into a cycle; speeds 3 and 4 missing; more stops per
   direction. Then `python3 tools/extract_appwalk.py`.
5. **Sim speed**: the sim walks 1.5 times too fast (11 vs 7.4 cm/s) and tips replaying two diagonals; next model
   refit leans toward higher joint damping (`tools/sim2real_replay.py --sweep`).
6. **Walk commands without the REST server** (parked task 5): run `tools/record_gait_topics.py --ip IP --walk 2`
   with the robot on the floor and someone next to it, then let the link node publish `/motion_planning_speed`
   directly and re-measure latency.
7. Remaining test-plan rows: D1, D4 to D6, D8, B7, E2 to E4, sections F (arms while walking), G, H, I (endurance).
8. Decide fall management for the show (firmware get-up vs ours) after the checks above.
9. Exhibition content itself (presentation script, face recognition of team members, gestures while Q talks) is not in
   this repository yet; the technical plan artifact lists the tasks.

---

## 7. Git and conventions

- Remote: `git@github-cube:youssefhindawi-cube/yanshee-sim.git` (GitHub `youssefhindawi-cube/yanshee-sim`). Ask the
  owner to add you as a collaborator, then clone with a plain `git@github.com:` URL on your machine.
- Last commit at handover: `7265aaa` (tools/record_gait_topics.py). Working tree clean except `robot_dump/`.
- Never commit `robot_dump/`, `external/` or anything with office photos. Force pushes are not used.
- Every behaviour change comes with a test in `tests/` and a line in `README.md` section 13; sim-to-real findings go to
  `docs/SIM2REAL_FINDINGS.md` with the date and the `robot_dump` file they came from.
- Commit messages describe the behaviour in one line (see `git log`).
- The hardware test plan artifact is the running status board; tick it after every robot task.
