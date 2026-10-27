# Yanshee online resources (checked 2026-10-27)

Checked 2026-10-27 (cloned and inspected, not just searched):
- **No official ROS 2 package or official simulation from UBTECH.** `UBTEDU/yanshee-ros` and `wisionhuang/ROS2-yanshee` contain only a README. `UBTEDU/Yanshee-Raspi-SDK` is deprecated.
- **github.com/ChancesSon/Yanshee_ROS** (2022, ROS1 catkin, no license): 17-DOF URDF with STL meshes, RViz display, and a Gazebo + ros_control package. Only the gazebo xacro has inertials (total about 2.2 kg). Joint names are generic (`left_arm1_joint`…) and every limit is ±1.58 rad.
- **github.com/Yanshee-Serenade** (2026 Chinese undergraduate thesis, no licenses):
  - Yanshee-RL-Unity: Unity ML-Agents PPO walking sim. Its `Assets/Texts/robot.json` (64 MB) is a Webots `YansheeRobot` model converted to JSON. The original Webots model was not found online.
  - Yanshee-IK-ROS2: C++ leg IK library (ament package, Apache-2.0).
  - ROS1-Nodes: ships `ubt_msgs` msg/srv definitions plus a ROS-TCP bridge.
  - ROS2-Nodes and Serenade-ROS2: Docker setup with YOLO-World, Depth-Anything-3, a Qwen VLM chatbot and a walker. Their ASR is still the built-in Chinese one.
  - The org's `.github/profile/EXPERIENCE.md` has practical notes:
    - camera intrinsics: 640x480, fx 503.6, fy 502.2, cx 312.6, cy 244.4
    - IMU raw frame is left-back-up
    - services `dm_deviceapp` and `dm_devmanagementapp` (IMU/speaker), plus `ubtedu-ros`
    - stream URL `:8000/stream.mjpg`; debug port `:2017`
    - the robot's ROS Kinetic talks to ROS Noetic in Docker, and to ROS 2 through ros1_bridge
- **Calibration:** done only in the Yanshee app (sidebar → Servo), per a now-404 SIT library FAQ. No API exists for it.
