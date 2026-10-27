# yanshee_sim

MuJoCo simulator, ROS 2 description and sim-to-real tools for the UBTECH Yanshee humanoid.
Design a motion in simulation, check it, then send the same file to the real robot.

```
python3 scripts/play_motion.py motions/wave_right.json                        # watch it in the simulator
python3 scripts/play_motion.py motions/wave_right.json --real --ip 192.168.3.13 --speed 0.5
```

![Sample motions in the simulator](docs/motions.png)

## The key convention

**Simulated joint angle (rad) = real servo angle (deg) in radians.** Joint names are the YanAPI
servo names (`RightShoulderRoll` … `NeckLR`) and limits are the YanAPI servo limits. Motion
files and code use servo degrees, exactly the numbers `YanAPI.set_servos_angles` takes, so
nothing is converted between simulation and hardware.

This comes from the Webots model the converter starts from: its saved joint positions are the
servo angles in radians, and they reproduce the Yanshee Course's default pose
(legs 90/60/76/110/90 and 90/120/104/70/90, neck 90, left arm 90/40/15).

## What is in here

| Path | What it is |
|---|---|
| `source_model/robot.json` | Webots Yanshee model exported to JSON, taken from [Yanshee-Serenade/Yanshee-RL-Unity](https://github.com/Yanshee-Serenade/Yanshee-RL-Unity). That repo has no license file, so ask the authors before redistributing. |
| `tools/build_models.py` | Converts the source into STL meshes, `model/yanshee.xml` (MuJoCo) and the ROS 2 URDF. Re-run it after changing servo limits or physics parameters. |
| `model/` | Generated MuJoCo model. `scene.xml` adds the floor and lights. 17 position-controlled servos, IMU, head camera (640×480), 2 IR rangefinders. |
| `ros2_ws/src/yanshee_description` | ROS 2 (Humble) package: URDF, meshes, RViz launch with one slider per servo. |
| `yanshee_sim/` | Python package: `connect("sim" or "real")`, motion files, safety checks, fake YanAPI. |
| `scripts/` | Viewer, motion player, pose editor, YanAPI script runner, motion recorder. |
| `motions/` | Example motions: `wave_right`, `look_around`, `cheer`, `squat`. `motions/recorded/` holds motions copied from the real robot. |
| `tests/test_models.py` | URDF vs MuJoCo agreement, reset keyframe, standing stability. |

## Setup

```
pip install mujoco numpy                      # simulator (MuJoCo 3.x; tested with 3.8)
pip install requests nest_asyncio opencv-python   # only for the real robot (YanAPI imports these)
```

The real backend loads the official `YanAPI.py` from
`../yanshee robot/yanshee/yanshee_examples/yanshee_examples/`. Set `YANAPI_DIR` to use another copy.

## Everyday use

**Look at the robot and move joints by hand**

```
python3 scripts/view.py --fixed        # torso held in the air; open "Control" on the right to drag servos
```

**Write a motion while watching it**

```
python3 scripts/pose_editor.py --out motions/hello.json
pose> set RightShoulderFlex 30 RightElbowFlex 90
pose> add 200
pose> set RightElbowFlex 130
pose> add
pose> play
pose> save
```

**Check and play a motion**

```
python3 scripts/play_motion.py motions/hello.json            # simulator
python3 scripts/play_motion.py motions/hello.json --fixed    # no falling, good for arm-only work
python3 scripts/play_motion.py motions/hello.json --real --ip <robot-ip> --speed 0.5
```

Before anything reaches the robot, `play_motion.py`:
- checks every angle against the servo limits and every runtime against YanAPI's 200–4000 ms window
- warns about joints moving faster than 300 °/s
- sweeps the motion for **self-collisions** (two parts of the robot meeting)
- refuses `--real` when any of those checks fail
- needs ≥30% battery and a confirmation before the robot moves

**In your own code**

```python
from yanshee_sim import connect, motion

bot = connect("sim")                              # connect("real", ip="192.168.3.13") for the robot
bot.set_servos_angles({"NeckLR": 60}, runtime=800)   # same call and return shape as YanAPI
bot.wait(0.8)
print(bot.get_servos_angles(["NeckLR"]))          # {'code': 0, 'data': {'NeckLR': 60}, ...}
motion.play(bot, motion.load("motions/cheer.json"))
bot.close()
```

The simulator runs physics in real time in a background thread, so it behaves like the robot:
commands return at once while the servos move over `runtime`, and `time.sleep()` works the same.

**Run an existing YanAPI script in the simulator, unchanged**

```
python3 scripts/run_in_sim.py my_show.py
```

`import YanAPI` and `from YansheeRobot import YanAPI` are redirected:
- **Simulated:** servo calls, `reset`, and motion files.
- **Printed:** speech, LEDs and music.
- **Return `code: -1`:** vision and voice recognition. Your script should already handle that case for the real robot.

Built-in motions such as `wave` or `WakaWaka` run on the robot itself. Copy one into the
simulator by recording it:

```
python3 scripts/record_real_motion.py --ip <robot-ip> --name wave --direction both
```

## ROS 2

```
cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
ros2 launch yanshee_description display.launch.py           # RViz + sliders, starting in the reset pose
```

- **Frames:** `base_link`, a fixed joint to `torso_link`, then one `<ServoName>_link` per servo, plus `head_camera_optical_frame`.
- **Units:** `/joint_states` positions are servo radians. Multiply by 57.2958 to get the YanAPI degrees.
- **Live view:** with `gui:=false`, any node that publishes the real robot's `get_servos_angles()` as `/joint_states` mirrors the robot in RViz.

## Directions (from the model; confirm on your robot)

The robot faces +y, its right side is +x, and z is up.

| Servo | Lower value | Higher value |
|---|---|---|
| `LeftShoulderFlex` | 40 = arm down | 90 = straight out to the side, 180 = overhead |
| `RightShoulderFlex` | 140 = arm down (mirror image: 180 − left) | 90 = out to the side, 0 = overhead |
| `LeftShoulderRoll` | 0 = arm swings back | 90 = neutral, 180 = arm swings forward |
| `RightShoulderRoll` | 0 = arm swings forward | 90 = neutral, 180 = arm swings back |
| `NeckLR` | below 90 = head turns to the robot's left | above 90 = to its right |

**Squat**, tested upright in simulation: right leg `HipFB −d, KneeFlex −2d, AnkleFB +d`, left leg the mirror image.
The other sign combinations make the robot fall over in simulation.

## Accuracy and limits

High-fidelity:
- **Kinematics:** joint axes, link lengths, meshes and the head camera pose come from UBTECH's CAD-derived Webots model.
- **Matching models:** the URDF and the MuJoCo model agree to within 1 µm (test).
- **Mass:** 2.10 kg, against about 2.05 kg for the real robot.

Approximate, and worth tuning against the real robot:
- **Servo model:** gain 8 N·m/rad, 2 N·m torque limit, damping and armature are assumptions (`SERVO_*` in `tools/build_models.py`).
  - Arm choreography and slow whole-body poses transfer well.
  - Balance-critical or fast motions (walking, kicks, reinforcement learning) need these tuned first.
  - The YanAPI REST loop is also too slow (tens of ms per call) for closed-loop balance from a laptop.
- **Webots inertia data:** one link's values were physically invalid. MuJoCo's `balanceinertia` repairs them.
- **IMU frame:** the simulated IMU uses the model frame. The real raw IMU frame is left-back-up, per the Yanshee-Serenade notes.
- **Reset pose:** taken from the Webots model. Compare `get_servos_angles()` after the robot's own `reset` motion and edit `RESET_POSE` in `yanshee_sim/servos.py` if they differ.

### First test on the real robot

1. Run the robot's `reset` and read all 17 angles. Compare them with `RESET_POSE`.
2. Play `motions/look_around.json` with `--speed 0.5`. Check that the head turns left first.
3. Play `motions/wave_right.json` the same way. Check that the right arm rises out to the side and waves.
4. If a joint moves the wrong way, report it. The fix is a sign or offset in one place in the converter.
