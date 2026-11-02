# Sim-to-real findings (29 Oct 2026)

From the GUI session of 29 Oct (26 min, 23 walks: 19 AppWalk, 4 SwayStep), with the robot's readings logged on the
robot (robot clock) and the shadow simulator (physics fed the robot's own servo frames) logged on the laptop:
`robot_dump/sim2real_20261029_165523.jsonl` + `robot_dump/robot_logs/robot_20261029_165523.jsonl`.

```bash
python3 tools/sim2real_report.py robot_dump/sim2real_20261029_165523.jsonl --joints   # the live comparison
python3 tools/sim2real_replay.py robot_dump/sim2real_20261029_165523.jsonl            # offline replay, current model
python3 tools/sim2real_replay.py SESSION --sweep                                     # model knobs
```

## Offline replay: the fair comparison

The live shadow got the firmware's frames over Wi-Fi, late and bunched; `tools/sim2real_replay.py` feeds the same
frames from the robot's own log at the robot's timestamps (and plays the robot's .hts motions, turns and their
ResetSwitch, at the times hal logged them), starting from the robot's encoder pose. Bunched frames alone made the
live shadow fall in two forward walks that the replay and the robot both finished.

| model | falls agree (17 AppWalk walks) | pitch RMS | roll RMS | SwayStep falls agree (4, not fitted) |
|---|---|---|---|---|
| before (damping 0.08, gear friction 0.01, contact friction 1.0) | 13 | 7.6 deg | 4.2 deg | 2 |
| **now** (damping 0.12, gear friction 0.08, contact friction 0.8) | **16** | **5.9 deg** | **3.1 deg** | **3** |

- **Joint damping / gear friction** were the levers for walking (servo gain and torque barely mattered; the current
  gain is the best). Damping x3 (0.24) or gear friction 0.15 fitted the walks as well but broke the get-up fit
  (`tools/getup_compare.py`: the simulator's GetupFront no longer stood up on a smooth floor, the robot's did): the
  geared servos resist small sways more than fast moves. 0.12 / 0.08 keeps every get-up result.
- **Contact friction 0.8** (robot surfaces and the default floor; the mat stays 1.4): with 1.0 the simulator fell in
  2 of the 4 turns in place that the robot finished; with 0.8 it agrees on all 4. Torsional friction has no effect
  (the contacts are condim 3).
- The 17th walk (walk 12) disagrees because it started 1.1 s after a stop with the robot still swinging; the replay
  starts from rest. The robot fell, the replay did not.
- Still off: **turning in place**, the simulator turns 40-75 % as far as the robot for the same TurnLeft/TurnRight.

## The robot's walk in the simulator (1 Nov 2026)

The biggest gap left was that Sim mode walked with SwayStep, our own gait, while the robot walks with the firmware's
(AppWalk). The robot's log shows the firmware walk is open-loop and repeats: in 11 forward walks the leg frames match
within 0.3 deg once the walk is going; the first second differs with the pose the robot walked off from (3 different
starts); one step cycle is 20 frames (0.4 s); after a stop it keeps walking to a step boundary (about 12 frames or
more after the command), winds down in about 11 frames and stands up with the same five 100 ms frames every time;
the arm swing starts from wherever the arms were. So the simulator now plays these frames itself (Sim mode,
`yanshee_sim/appwalk.py`, table built by `tools/extract_appwalk.py`), and SwayStep is archived in the simulator too.

- Recorded: forward (11 walks) and left (2; logged without its sign, the simulator told it apart). Not yet:
  backward, right, the diagonals; and the speed of each walk was not logged (the GUI now logs every command).
- With the robot's frames the simulator walks 65 cm forward in 5 s and drifts 8 cm left; the robot's distance has not
  been measured (a tape measure next time).
- TurnLeft is fragile in the simulator: from some standing poses (the crouch AppWalk ends in, the end of ResetSwitch)
  it tips within 2 s, from others not; TurnRight stays up. On the robot one 6 s TurnLeft ended face down during
  ResetSwitch. Part of the open "turning in place" gap.

## Servos and IMU

- Frames to encoders: median error about 1 deg (whole-degree encoders), delay 20-80 ms.
- Over the first 2 s of a walk the robot and the simulator sway alike (pitch SD about 5 deg, roll about 3 deg).
- The IMU runs at 19 per second; axes and signs as emulated (milestone C).

## AppWalk falls: restarts, not stops

The robot's log shows the stop itself is benign: left alone after a forward stop the robot is back within 3 deg of
standing in 0.25-1.5 s (pitch at most 4.5 deg). Walks restarted 0.9-1.5 s after a stop caught it still swinging and
leaned it back 11-38 deg; the one forward-walk fall came 2 s after such a restart. The walking statistics blamed
"walk+->stop" for 4 falls because the stop was sent (by hand or by the fall handling) while the robot was already
going over; falls are now attributed to what the robot was doing when it was last upright. One fall came during
ResetSwitch after a 6 s turn.

**Fixed:** a settle gate in the controller (`Controller._robot_settled`): a walk start, a turn start and a
walk<->turn switch wait until at least 1.2 s after the last stop or turn release and until the IMU has been within
3 deg of the standing lean for 0.5 s (at most 2.5 s), and until no motion (ResetSwitch) is playing.
`tools/joystick.py` waits a fixed 1.5 s.

## The firmware's fall manager

With fall management on, hal now detects every fall ("fall detection, and ready to raise up") and plays
GetupFront/GetupRear itself (GetupRear about 8 s). Our auto get-up then asked for a get-up too and was refused
(motion engine rc 5 = robot fell; the GUI showed "code 1 Failure"). **Fixed:** the auto get-up first waits for a
motion the robot is already playing (REST motion state, hal's fall flag via the robot link), up to 30 s, and plays its
own only if the robot is still down afterwards; the Get up button says so instead of failing.

## Other fixes from this data

- **Encoder polling** during GetupRear filled hal's buffers (154 "buffer is full" warnings, 7 servo-read timeouts):
  the robot link now skips encoder reads while hal plays a motion.
- **Live shadow timing**: the firmware's frames are scheduled at the robot's own timestamps plus 0.15 s instead of
  applied as they arrive; turns and other .hts motions hal plays are played in the shadow too (local copies of the
  robot's files, fetched once when missing).
- **Walking statistics** logged each stop twice (two stops arriving at once): locked, and the report skips the old
  duplicates.
- The link carried our frames 3 ms late when idle and 31 ms late while streaming at 50 per second. Corrected 1 Nov
  2026: the 31 ms was the robot's clock drifting (57 ms over the 24 min session; the robot's clock is not time-synced, NTP off, seen 2 Nov 2026)
  against one clock offset for the whole session; the report now follows the drift and the frames were 3 ms late
  (95% 10 ms, max 34 ms). With it the recorded AppWalk stops come 12 frames after the command at least (was 13).
- Most of the session ran on battery (7.95-8.38 V); the charger came off just before the first walk.

## Link latency (1 Nov 2026, from the 29 Oct logs)

The link's round trip (laptop -> robot -> laptop, ssh over Wi-Fi) has a median of 7 ms and a 95th percentile of
13-23 ms; one way our frames took 3 ms (95% 10 ms). Of the 7 ms the radio is 1-2 ms: the rest is ssh and Python on
the Pi 3. The problem is the tail, not the average: stalls up to 2.5 s, 10% of the demo's pings over 170 ms, and
two drops off the Wi-Fi. With AppWalk the walk runs on the robot, so the link only delays its start and stop (the
firmware itself starts walking 0.3 s after the command) and the mirror. The robot's own readings come every 53 ms
(IMU, hal's rate) and every 200 ms (encoders, our setting: a reading takes about 35 ms). The REST server (port
9090, where AppWalk's walk, stop and turn commands go) took a median 64 ms per battery call on 29 Oct
(tools/robot_checks.py latency), so a walk command spends more time there than on the network.

The robot is a Raspberry Pi 3 Model B (revision a02082): Wi-Fi 2.4 GHz only (BCM43438, Bluetooth on the same chip
and antenna), Raspbian 9 with Linux 4.14. Bluetooth would be slower (tens of ms) and would share the Wi-Fi's radio;
Wi-Fi 6/7 USB adapters need Linux 5.x/6.x drivers, and upgrading the robot's system risks UBTECH's software. In
order: the robot's Wi-Fi power saving off, a dedicated travel router on a quiet channel with the laptop wired to it,
measure (`tools/latency_test.py`), and only if it still stalls a 5 GHz Wi-Fi 5 USB adapter with a driver for 4.14.

## Turning and IMU balance (1 Nov 2026)

**Why the simulator tipped in TurnLeft.** TurnLeft and TurnRight (the .hts hal loops while the button is held) step
between a deep crouch and a pose with one leg lifted, in 260-540 ms moves; there is no hip yaw joint, so the robot
turns by pivoting on the stance sole. The robot's log (5 turns, 29 Oct) shows TurnLeft is marginal on the robot too:
15-25 deg/s, the torso leaning 16-22 deg, and the one 6 s TurnLeft ended in a fall during ResetSwitch. In the
simulator, looping TurnLeft 6 s from three standing poses (reset, the crouch AppWalk ends in, the end of ResetSwitch),
two speeds and three floors:

| contact friction | TurnLeft | TurnRight |
|---|---|---|
| 0.8 (the model until today) | fell every time, at about 4 s (the 5th lift) | up, 14-16 deg |
| 0.7 | up, leaning up to 21 deg (the robot: 16-22), 14 deg/s | up, 15 deg, 26 deg/s |
| 0.6 / 0.4 | up, 6-8 deg | up |

Slowing the frames down did not help. The stance sole gripped too hard to pivot, so the leg's twist went into tipping
the body. Replaying the robot's 17 walks at 0.7 agrees on 17 of 17 (0.8: 15, 0.6: 15, with the report's clock-drift
fix), and the get-up comparisons are unchanged (same outcomes, euler-x within 0.2 deg), so the model's contact
friction is now 0.7 (`tools/build_models.py` CONTACT_FRICTION; the mat stays 1.4).

**IMU balance** (`yanshee_sim/balance.py`): ankle offsets added to whatever AppWalk commands, from the torso's lean
beyond a deadband around the standing lean (2 deg pitch, 4 deg roll: the walk sways on purpose) and the lean rate,
at most 8 deg, at most 120 deg/s; pitch on the two AnkleFB, roll on the two AnkleUD (signs measured in the
simulator). The simulator runs it on its emulated IMU (19 per second, with noise) every 20 ms. Peak tilt in degrees
(F = fell), friction 0.7:

| scenario | off | balance |
|---|---|---|
| forward 4 s | 18.7 | 8.9 |
| forward, restarted while still swinging | 14.3 | 9.5 |
| push from behind / front / side (0.3 N s, 0.1 s on the torso, mid-walk) | F / F / F | 11.4 / 8.9 / 13.4 |
| side push 1.2 N s | F | 18.3 |
| push from behind 0.5 N s | F | F |
| floor friction 0.5 / 1.0 | 23.6 / 13.9 | 9.1 / 8.3 |
| left walk | 7.9 | 7.3 |
| TurnLeft 6 s (friction 0.8: F) | 21.0 | 6.4 |
| TurnLeft after a walk | 17.1 | 8.9 |
| TurnRight | 15.4 | 9.2 |

The cost: about 7 % less distance walking forward (58 -> 54 cm in 4 s). Turns get faster with it (TurnLeft 84 ->
139 deg in 6 s, TurnRight 158 -> 225): the steadier stance foot pivots further. Gains tried: half and double
(double fell on a side push), pitch only (fell turning after a walk), roll only (fell on a side push and in TurnLeft),
more rate damping (kd 0.06 kept; 0.08 fell turning after a walk).

On the robot the same code runs in the robot link's node: each firmware gait frame (50 per second) is followed by an
ankle-only frame with the offsets, from the robot's own IMU, so the Wi-Fi is not in the loop; after the walk the
offsets fade back to the gait's angles. Gated by `tools/robot_checks.py balance` (the ankle and IMU signs must match
the simulator's). Turns: hal plays TurnLeft/TurnRight from its .hts itself, so nothing can be added to its frames;
with the balance on the robot the node plays them instead, from the same keyframes (sent from model/appwalk.json at
link start): each keyframe a linear move over its runtime in 20 ms frames, then its hold, the first one from the
encoders' pose, looped until released, then ResetSwitch, every frame with the ankle offsets; it stops by itself
(ResetSwitch) if the laptop goes silent, at once on STOP. As in the simulator's own turn playback, where the balance
removed the TurnLeft falls.

## Link and Wi-Fi measured on the robot (2 Nov 2026, office, robot on the charger)

Robot on the office network (2.4 GHz channel 6, signal -50 dBm, 72 Mbit/s); laptop on 5 GHz at -67 dBm.
`tools/latency_test.py`, 5 minutes each (robot_dump/latency_test_20261102_*.json):

| | ping median / 95% / max | link (ssh) median / 95% / max | REST new / kept open (median) | replies over 100 ms (ping) |
|---|---|---|---|---|
| Wi-Fi power saving on (as booted) | 14 / 587 / 2059 ms | 25 / 694 / 2261 ms | 129 / 76 ms | 28 % |
| power saving off | 14 / 291 / 1868 ms | 20 / 348 / 1915 ms | 129 / 86 ms | 20 % |

- **Power saving was on** (a boot script turns it off at 9 s, something turns it back on), and the Wi-Fi took about
  3 minutes after boot to join. Off, the 95th percentiles halve; the 1-2 s stalls remain.
- **The stalls are on the robot's leg**: pinging the router at the same time, the laptop got median 5 / 95 % 20 /
  max 41 ms, the robot 11 / 153 / 504 ms (8 % over 100 ms). The robot's channel 6 is overlapped by strong networks on
  5, 7 and 8; channel 1 (one weak network) and 11 are the quiet ones here: the show router goes there.
- **The REST server is the cost of a command**: it takes 56-77 ms per call on the robot itself (through the link,
  timed on the robot); a call over the Wi-Fi with a kept-open connection 76-89 ms, a new connection 117-129 ms
  (rest.py's way), through the link 87 ms. Below that only by skipping the REST server: `/gait_node` listens on
  `/motion_planning_speed` (ubt_msgs: int32 speed_v, speed_h) and `/teleopkey` (Twist); nothing publishes there
  while idle. What the REST server sends there during a walk is still to capture (robot walking).
- The robot's clock is not time-synced (NTP off): the drift in the logs is the crystal's, the report follows it.
  Fixed the same day: `tools/robot_setup.py` syncs it at boot (and keeps Wi-Fi power saving off).
- **Command -> servo through the link** (`latency_test.py --move`, robot standing, timed on the robot's own log):
  LeftShoulderFlex starts moving a median 76 ms after our frame (worst 92), is settled by 124 ms (worst 156), 1 deg
  short of the command. Over REST (29 Oct, B4) the first movement came 310 ms after the call: the link reaches the
  servos four times sooner, which is why the balance and the node's turns run there.
- The new robot node starts on the robot in 5 s and does everything that does not move the robot: REST through the
  link, IMU 19/s, encoders, the turn frames loaded, balance on/off while idle (no frames sent), no warnings. Its
  first start failed: `pkill -f` matched its own launch command; the node now leaves a pid file instead.
- The demo session of 29 Oct (copied today): the robot stayed up in its 4 walks; replayed on the current model the
  simulator falls at 6.4 s in the 12 s forward walk (3 of 4 agree); with joint damping x1.5, servo gain x1.25 or
  friction 0.6 all 4 agree (damping best: pitch RMS 4.6 deg). A lead for the next refit (damping x2 broke the
  get-up fit before). Its two turns: the simulator turned half as far (-32 / -41 deg against -63 / -96).
