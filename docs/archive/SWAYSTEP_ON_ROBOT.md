# SwayStep on the real robot (archived 29 Oct 2026)

**Decision:** the robot walks with **AppWalk** (UBTECH's firmware gait, driven the way the mobile app drives it) and
we build on top of it. SwayStep, our own gait (`yanshee_sim/sim_extras.py`, `SimGait`), is no longer offered for the
robot. **Parked in the simulator too (1 Nov 2026):** it needs a lot of work before it is useful, and a simulator that
walks a different gait from the robot's rehearses nothing; Sim mode now plays the robot's own AppWalk frames
(`yanshee_sim/appwalk.py`). The code, its tuning tool (`tools/tune_gait.py`) and its tests stay.

## What was tried

SwayStep ran on the laptop and streamed its servo frames to the robot 50 times a second over the robot link
(`yanshee_sim/robot_stream.py`: a Python 2 ROS node started over ssh publishes them on `/hal_angles_set`, the topic the
firmware gait uses). Its ankle balance used the robot's IMU (`RobotBody.torso_lean`) instead of the simulator's;
the capture-point brake and foot placement were off (the robot gives no body velocity).

## What happened (session `robot_dump/sim2real_20261029_165523.jsonl` + `robot_dump/robot_logs/robot_20261029_165523.jsonl`)

| | runs | fell | how |
|---|---|---|---|
| SwayStep on the robot | 4 (forward x3, turn x1) | 2 | both on its back, 4.1 s and 4.4 s in |
| AppWalk, same session | 19 | 2 | on its back walking forward, face down while turning |

- The crouch, the joint tracking and the link were fine: joints within 1-3 deg of the frames, frames 31 ms late
  (median, 95 % 38 ms) while streaming.
- From the first step the robot pitched back and forth and the swing grew every step: -3, +11, -6, +13, +18,
  +32 deg (+ = leaning back), then it fell backward. The ankle-balance command swung with it (ankle pitch 110-148
  deg), so the IMU feedback fed the oscillation instead of damping it.
- The second fall came while standing up out of the walking crouch after the walk stopped: still pitching back,
  the robot went over backward when the knees straightened.
- The shadow simulator, fed the same frames, fell **forward** in the first run and stayed up in the second: the
  simulator does not predict how the real robot responds to these frames.

## Why it works in the simulator and not on the robot

- **Delay in the balance loop.** In the simulator the ankle balance reads the body's exact lean with no delay. On the
  robot the lean comes from the IMU's fused euler angles (19 per second, filtered), then crosses the link and the
  servos (frame to encoder 20-80 ms): the correction arrives late and pushes the swing it was meant to stop. The gains
  were tuned in the simulator, without that delay.
- **Different body dynamics.** With identical joint commands the simulator and the robot fall at different times and
  in different directions (`docs/SIM2REAL_FINDINGS.md`), so gains tuned in the simulator do not carry over.
- **No momentum check on the robot.** The capture-point brake and foot placement need the body's velocity, which the
  robot does not measure.

## Re-enabling it for an experiment

```bash
YANSHEE_ALLOW_SWAY=1 python3 scripts/web_gui.py --mode both --ip ROBOT_IP --viewer
curl -X POST http://127.0.0.1:8080/api/walkmethod -H 'content-type: application/json' -d '{"method": "sway"}'
```

The walk method applies to both: with `sway` the simulator walks with SwayStep in Sim mode and the robot gets
SwayStep's frames in Robot and Both modes.

What would have to change first: balance gains retuned with the IMU and servo delays in the simulator's loop (or the
ankle feedback off), the gait loop on the robot itself instead of over the link, and a simulator that falls when and
where the robot falls (the sim-to-real to-do in `docs/ROBOT_SESSION_PLAN.md`).
