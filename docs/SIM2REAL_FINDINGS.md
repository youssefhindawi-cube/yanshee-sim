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

## The robot's first walks through the link, and the balance fall (2 Nov 2026)

Office floor (smooth), battery 60 %, GUI in Robot mode, every command through the robot link's relay (13 of 13 in
the first walk: the walk, the stop, the gait-state polls; the REST server took 28-305 ms each on the robot).

| walk | balance | frames, s | lean (pitch / roll) | result |
|---|---|---|---|---|
| 1, speed 2 | off | 143, 2.9 s | -12..+18 / -5..+7 deg | up |
| 2, speed 2 | off | 274, 5.5 s | -5..+9 / -6..+9 deg | up |
| 3, speed 2 | **on** | 384, 7.7 s | +85 back at 1.6 s | **fell on its back** |

**Why the balance felled it.** The robot's own log, every 0.2 s: at 0.2-0.3 s the walk's first step leaned the
torso 5 deg forward and the balance answered with -2.8 deg at the ankles (lean back); the walk's sway then swung
back on its own, the correction added to it (+5.8, +9, +12 back by 0.7 s), the balance reversed to its +8 deg limit
(lean forward) and stayed there, but the lean went +18, +24, +32, +39, +55, +85: over in 1.6 s. Nothing like it
without the balance (walk 2 right before it: +9 at most). The cause is delay: the robot's servos start moving 76 ms
after a frame and settle at 124 ms, the IMU report adds up to 53 ms, so at the walk's 2.5 Hz sway (a 400 ms cycle) the
correction lands a third of a cycle late, in phase with the next swing instead of against it. The simulator's servos
follow a frame at once and its IMU has no delay, so there the same gains halved the sway. No servo warnings from the
100 frames a second (gait plus ankle frames).

So: **the balance on the robot is parked** (`Controller.ROBOT_BALANCE_ENV`: `YANSHEE_ROBOT_BALANCE=1` to try it
again); the GUI's Balance stays on for the simulator only and says so. Before another try: put the measured servo
delay (first motion 76 ms, settled 124 ms) and the IMU's 53 ms into the simulator's servo model, re-tune there
(expect: a much smaller gain, phase lead from the rate term, or a correction that only acts on the slow part of the
lean, below 1 Hz), and first test on the robot standing still with pushes, not walking.

**Get up on its back (E8)** right after: GetupRear, then the reset move; upright in 11 s, every encoder within 2 deg
of the reset pose, nothing stepped in.

## Walking tests of 2 Nov 2026 (office floor, GUI in Robot mode, every command through the link)

| test | result |
|---|---|
| E6 watchdog, walking | laptop silenced 2 s into a speed-1 walk: the robot stopped itself 1.59 s later (the app's stop, then its wind-down and stand frames), stood still; link back by itself about 12 s later |
| E6 watchdog, turning | holding TurnLeft: ResetSwitch 1.74 s after the turn began, +24 deg turned, 7 deg lean at most |
| E8 get up, on its back | GetupRear + the reset move: 11 s, upright, every encoder within 2 deg of the reset pose |
| E8 get up, face down | Base_GetupF + its reset + the reset move: 18 s, upright, within 2 deg, no leg stepped in |
| D3 directions | backward, right, left, forward-left, backward-right, forward-right, backward-left: one 5-6 s walk each at speed 2, all upright; speed_h < 0 is the robot's right (watched). All but backward-left in the table: that walk never settled into a repeating cycle (re-record it) |
| D2 distance | 45 cm in 6.10 s of 20 ms frames at speed 2 = 7.4 cm/s (tape measure); heading +5 deg over the walk |
| E7 balance | parked after the fall (above) |

**The simulator walks too fast.** Playing the same recorded frames, the simulator's forward walk covers 56 cm in
5 s (11.2 cm/s) against the robot's 7.4 cm/s: the robot's feet slip and its servos lag more than the model's. Until
the model is refitted, Sim mode distances are about 1.5x the robot's. Replaying today's diagonals the simulator
also falls in backward_right and forward_right where the robot stayed up.

**The Wi-Fi drop, seen live.** Two minutes off the network right after the E6 tests, power saving off. The Pi's
under-voltage flag stayed clear and the kernel log shows no carrier loss, so neither the battery nor the radio
reset; the association stayed up and packets stopped getting through. The robot's data rate fell 72 -> 14 Mbit/s
over the session (182 excessive retries, rx errors 171k), and a 20 s ping from the robot to the router lost 12 %
with 2.3 s stalls while the laptop's lost nothing. Channel 6 with strong networks on 5, 7 and 8, an antenna inside
a moving body: the show router on channel 1 or 11, close to the stand, is the fix; the watchdog makes a drop
mid-walk harmless meanwhile.

## The robot's delays in the simulator, and the balance retuned (2 Nov 2026, evening)

**Delay model.** `SimRobot` now applies every servo command, balance offsets included, `SERVO_DELAY_S` (60 ms) after
it is sent, and serves IMU samples `IMU_LATENCY_S` (20 ms, assumed) old on top of the 19 Hz hold. 60 ms reproduces
the robot's step response measured through the link (a 15 deg move, 40 ms encoder reads: first motion median 76 ms,
settled 124 ms). With it the simulator falls with the old balance gains in every scenario (walk, restart, push,
turn) where it stayed up without them, as the robot did; with the balance off nothing changes but a slightly
slower turn.

**What the torso does.** The walk rocks the torso forward and back at about 1.2 Hz (robot logs, steady forward
walks: pitch period 0.42-0.89 s, 8-15 deg peak to peak; the simulator the same). A correction that lands a quarter
of that period late feeds the rocking. A low-pass on the lean error (tau 0.4 s) made things worse, falling even
without delays: more lag. Rate-only damping fell too.

**Retune** (`balance.Balance.GAINS`): a quarter of the old gains and half the authority: kp 0.15, kd 0.04 (pitch),
kr 0.15, krd 0.015 (roll), deadband 4 deg, at most 4 deg of ankle. Peak torso tilt in the simulator with the delays,
friction 0.7 (F = fell):

| scenario | off | old gains | retuned |
|---|---|---|---|
| forward 4 s | 13.1 | F | 9.0 |
| forward, restarted while swinging | 12.8 | F | 9.2 |
| push from behind / front / side, 0.3 N s | F / F / 16.1 | F | 16.8 / 11.9 / 14.2 |
| harder pushes (0.5 N s behind, 1.2 N s side) | F | F | F |
| floor friction 0.5 / 1.0 | 13.0 / 11.1 | F | 10.1 / 8.7 |
| left walk, left after a walk | 7.5, 12.1 | F | 8.9, 9.0 |
| TurnLeft, TurnLeft after a walk | F, F | F | F, 17.7 |
| TurnRight, TurnRight after a walk | 14.7, 15.7 | F | 13.9, 13.3 |

Next on the robot, in this order: standing still with finger pushes (Balance on, no walking; the robot's log shows
the offsets), then a 3 s forward walk with someone ready to catch it, then the rest of E7. The robot side stays
parked (`YANSHEE_ROBOT_BALANCE=1`) until the standing test passes.

## The hip stall in a face-down get-up (2 Nov 2026, afternoon)

Pressing Get up with the robot face down (battery 8.0 V, smooth office floor): Base_GetupF ran its 7 s, the robot's
reset followed, and the robot tipped onto its right side with **LeftHipLR, RightHipLR and RightShoulderRoll
stall-locked** (they refused every command; freed by hand: limp 2 s, work, the robot's reset: all 17 back within
2 deg). The robot's log against the morning's successful face-down get-up (battery 7.2 V, same floor):

- The two IMU traces are identical for 6 s. Then the morning's rose to upright (euler-x 109 -> 100, roll within
  4 deg); the afternoon's stopped at 94-98 and **rolled right: 7, 14, 20 deg** over the last second of the motion.
- At the end of Base_GetupF the morning's hip rolls read 89 / 92 (closed, as the motion commands); the afternoon's
  read **29 / 117**, 60 and 27 deg short: the hips were already stuck during the motion's own close, with the
  body leaning on the right leg. The reset that followed asked the loaded, stuck hips to drag the feet in: the
  servos stalled (their readings went to 340 / 76 / 4: locked) and the robot went over at 8.5 s.
- No hal warnings in that window; the encoders are not read while hal plays a motion (they filled its buffers on
  29 Oct), so the moment the hips jammed inside the motion is not in the log. Battery 7.9-8.1 V throughout.
- The same pair (RightHipLR + RightShoulderRoll) locked on 29 Oct on the mat (robot_dump/getup_Base_GetupF_132903).

**What the simulator says about the torque.** After its Base_GetupF the simulated crouch has the hips at -41 / +28
deg (the real motion ends closed when it goes well: the simulator's hips cannot close it during the motion either).
Closing them with the robot's reset holds both hip-roll servos at their torque limit (1.15 N m) for the whole move
and leaves the legs 41 / 14 deg out, at any speed (1 s or 3 s). So the hips are not given more torque than needed:
they are asked for a move the planted feet will not allow, draw stall current, and the real servo's protection
locks them. Standing up wide-legged first and stepping each leg in with the foot lifted (the step-in built the day
before) ends with the legs within 4 deg of the reset pose, but holding the wide stance also keeps the hips at the
limit (5-7 s with the quick steps, 11 s with the slow ones), so it has to be quick.

**Changed:** when Base_GetupF ends with a hip roll more than 10 deg out (`Controller.HIP_WIDE_DEG`) the get-up no
longer plays the robot's reset: it stands up wide-legged (reset pose, hips left where they are, 1.2 s) and steps
the legs in with the quick steps (up to 3 rounds), then the reset move; a stalled servo stops the get-up with a
message. A **Free stalled servos** button (POST /api/servos/free) does the limp / work / reset cycle for the locked
servos; someone must hold the robot. Still to test on the robot: a face-down get-up that ends wide (hard to provoke
on purpose), and whether the real hips object to a few seconds standing wide (the log's `enc` rows and the locked
check before and after will say).

Logs: robot_dump/robot_logs/robot_20261102_134849.jsonl (robot clock), robot_dump/sim2real_20261102_134849.jsonl,
robot_dump/getup_stall_2026-11-02_gui_events.json.

## Speed ramp, per-speed tables and the replay comparison (2 Nov 2026, afternoon)

**Why speed 5 fell.** Every un-ramped walk started at speed 5 (9 that afternoon, battery 8.0-8.4 V) showed the same
start: a dip forward in the first 0.3 s, then a backward lurch of 15-20 deg by 0.8 s; 4 of the 9 went over
backwards within 1.5 s, 5 recovered. Walks at speeds 1 and 2 (7) showed no lurch at all (pitch within 3 deg). The
spacing to the previous movement was the same for the falls and the survivors (1.6-5 s), so it is the start itself.

**The ramp** (`Controller.RAMP_START/RAMP_STEP_S/RAMP_DOWN_S`): a robot walk asked above speed 2 begins at 2 and goes
up by 1 every 0.5 s (the firmware takes speed changes in place: the gentle stop used that already); a release steps
down to 2 before the app's stop. Three ramped starts at speed 5: no lurch (pitch within 3 deg in the first second),
peak lean 9-13 deg at the speed steps, 0 falls. The simulator ramps the same way (`AppWalkSim(ramp=...)`).

**Per-speed tables.** `tools/extract_appwalk.py` now keeps one table per speed: `forward` (speed 2, 19 walks),
`forward@1` (8), `forward@5` (10, the un-ramped afternoon starts). The simulator plays the exact speed's table, else
the nearest recorded, and switches tables in place at the same cycle phase when the speed changes (the ramp). Played
in the simulator, the speed-5 table **falls** in its 5 s playback where the speed-2 table walks 51 cm: the
simulator reproduces the lurch. But it also tips over when ramped (2 -> 5 in place, as the robot was driven), where
the robot stayed up 3 of 3: the simulator's speed-5 cycle is less stable than the robot's, a gap for the next model
fit (tests/test_appwalk.py records both outcomes).

**Replay of the afternoon's walks** (`tools/sim2real_replay.py`, the robot's own frames, current model): the session
with the un-ramped speed-5 falls agrees on 22 of 28 walks; of the robot's 5 falls the simulator reproduces 3 and
misses 2, and it falls in 4 walks the robot survived (two of them turns). The ramped session: 2 of 4 (the simulator
fell in one ramped walk the robot survived; one "walk" is the later stall). Joint damping x1.5 was the best lead
for the next model fit on the morning's data; worth re-checking on these.

Still open: backward-left never settled into a cycle (re-record); speeds 3 and 4 not recorded; the Wi-Fi stalls of
the office channel interrupted two of the ramped walks (the robot's watchdog stopped them, as designed).
