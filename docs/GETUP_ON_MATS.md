# Getting up face down on a mat (parked, 29 Oct 2026)

**To do (later, option 3):** find a face-down get-up that works on the soft, grippy mat by iterating on the robot
itself (frame by frame, `tools/getup_lab.py --step`), using the simulator only for hints. Until then the stand needs a
smooth hard surface, where everything below already works.

## What works today

| Lying | Smooth hard floor | Soft mat |
|---|---|---|
| face down: GetupFront (firmware's) | stands, 13.8 s | pikes to -48 deg and falls forward |
| face down: Base_GetupF, then reset (GUI's Get up) | stands, 6.8 s + reset | stands, then a hip roll locks while it closes its wide crouch |
| on its back: GetupRear | stands (all runs on the mat) | stands, 5.5-6.7 s |

Fall management (the firmware's automatic get-up) stays off: its on/off setting never reached the hal node on our
robot, it ignores falls it blames on people, and face down it plays GetupFront.

## Why the mat fails

- **GetupFront**: the knees grip the mat and cannot slide under the body, so unfolding the legs lifts the hips into a
  pike and the robot falls forward. All servos reach their targets.
- **Base_GetupF**: it rises (using its momentum) into a crouch with the legs spread about 55 deg each side, then pulls
  both loaded feet in. On the mat a hip roll cannot drag its foot, the load pushes it out to its end stop (0 or 180)
  and the servo stall-locks (it refuses commands until set to program mode and reset). Seen 4 times out of 4.

## What was tried (all failed on the robot, files removed from the robot)

| Version | Idea | Result on the mat |
|---|---|---|
| fix 2 | close the crouch one foot at a time, shifting weight over the other | a bug jolted an ankle 24 deg, and even in the sim the weight never left the foot; hip locked |
| fix 3 | slide the feet in while rising, all joints 10 deg inside their limits | rose like the sim predicted, hip locked at the final close |
| fix 5 | fix 3 searched in a calibrated sim: slower (2.75x), deeper rise, foot lifted 11 deg | **no lock**, but the slow deep rise sank into the mat and never got up |
| blend | Base_GetupF's fast rise with fix 5's slide | not run: does not rise in the calibrated sim |

## Simulator calibration (tools/getup_research/calib.py)

The default model's servos are about twice too strong. With the **hip rolls limited to about 1.2 N m** (model:
2.0; UBTECH quotes 8-12 kg cm for the servo family), the **soles at friction 1.0 on the mat** (0.2 on the smooth
floor) and the **knees and body at 1.5**, the sim reproduces the robot's hip-roll shortfall at the end of the closing
frames: smooth floor 6 deg (robot under 20, stands), Base_GetupF on the mat 49-50 deg (robot 53-92, locks), fix 3 41
deg (robot 62-86, locks). A shortfall under about 15-20 deg is what worked on the robot. The sim does not model a
servo's stall lock, nor the mat giving under the knees and hands: it predicted fix 5 would rise and the fast blend
would not, the robot showed the reverse.

## Leads for later

1. Start from fix 5's slide (it avoided the lock) with Base_GetupF's fast rise, and tune on the robot frame by frame.
2. Slicker soles (PTFE or smooth tape): the calibrated sim says Base_GetupF then closes without locking; check walking.
3. Feed the 1.2 N m hip-roll figure into `tools/build_models.py` (`SERVO_TORQUE`) once measured more directly.

## Files

- `tools/getup_lab.py`: plays a get-up on the robot frame by frame or with its own engine (`--engine`), records the IMU
  and every servo, checks for locked servos (`--finish-reset` resets only when none is locked). Fast motions must use
  `--engine`: a REST replay starts every frame about 0.2 s late and changed GetupRear's outcome.
- `tools/getup_research/`: the searches (`fix2.py` .. `fix5.py`), the calibration (`calib.py`, `split_friction.py`),
  the sim player (`sim_getup.py`) and fix 5's parameters (`fix5_params.json`). Run from the repo root with
  `MUJOCO_GL=egl`.
- `robot_dump/getup_*.json`: every robot run (per-frame IMU and joint readings); `robot_dump/fall_*.json`: the fall
  management trials.
- User motions go in `/home/pi/Documents/motions/NAME.hts` on the robot and play by name (`hts.build` writes them byte
  for byte like the robot's own files).
