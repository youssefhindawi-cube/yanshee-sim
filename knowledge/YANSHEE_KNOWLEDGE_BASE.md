# Yanshee Robot: Consolidated Knowledge Base

Compiled 2026-10-27 from every file in this folder: the User Manual, the YanShee-EN-20180228 slides, the Instructor Guide, the 411-page Yanshee Course (full digest in `YANSHEE_COURSE_DIGEST.md`), the YanAPI.py source, all examples, the Cairo University demo .rar and the UBTalk video (visual content only).
**Status:** this is what the documents say. None of it has been checked on the live robot yet. Items marked ⚠ conflict between sources or are known bugs.

---

## 1. Hardware

| Item | Value |
|---|---|
| Brain | Raspberry Pi 3B (BCM2837, 1.2 GHz quad-core A53, 1 GB RAM, 16 GB storage) plus an STM32F103 co-processor (servos and sensors) |
| OS | Raspbian **Stretch**, Python **3.5**, armv7l, ROS **Kinetic** |
| DOF | 17 serial-bus digital servos with angle readback, IDs, and self-protection |
| Size / weight | 370×192×106 mm, about 2.05–2.09 kg |
| Power | DC 9.6 V, 4.5–38.4 W. Battery 2750 mAh (manual) vs 3100 mAh (slides, v1.1) ⚠. Charger output 9.6 V / 4 A |
| Camera | 8 MP fixed-focus camera in the head |
| Audio | 1 microphone, 2 stereo speakers |
| LEDs | 2 RGB eye LEDs ("camera"), 3 chest LEDs ("button"), mic indicator, charge indicator |
| Sensors | Built-in 9-axis IMU (MPU9250) and board temperature sensor. 6 magnetic POGO 4-pin ports for add-on IR, ultrasonic, touch, pressure and environment modules (I2C via the STM32) |
| Ports | HDMI, 2× USB, 40-pin GPIO (WiringPi numbering). Pins 3/5 are I2C and pins 12/35/38/40 are I2S audio. **Do not use these 6 pins.** |
| Buttons | Chest ON/OFF: hold 2–3 s until blue, wait for the boot sound. A short press while running stops a motion and starts voice interaction. The **emergency stop on top of the head cuts power.** |
| Battery | Behind the cover under the left arm: turn 90° clockwise to unlock. It charges in the robot or on the dock. |
| Network | 2.4 GHz Wi-Fi only (802.11 b/g/n), Bluetooth 4.1 |

### Servos (name → ID → safe range from the YanAPI source)
| ID | Name | Range | | ID | Name | Range |
|---|---|---|---|---|---|---|
| 1 | RightShoulderRoll | 0–180 | | 10 | RightAnkleFB | 0–180 |
| 2 | RightShoulderFlex | 0–180 | | 11 | RightAnkleUD | **65–180** |
| 3 | RightElbowFlex | 0–180 | | 12 | LeftHipLR | **60–180** |
| 4 | LeftShoulderRoll | 0–180 | | 13 | LeftHipFB | **0–170** |
| 5 | LeftShoulderFlex | 0–180 | | 14 | LeftKneeFlex | 0–180 |
| 6 | LeftElbowFlex | 0–180 | | 15 | LeftAnkleFB | 0–180 |
| 7 | RightHipLR | **0–120** | | 16 | LeftAnkleUD | **0–115** |
| 8 | RightHipFB | **10–180** | | 17 | NeckLR | **15–165** (docstring says 45–135; course says 30–150 ⚠). Use 45–135 to be safe. |
| 9 | RightKneeFlex | 0–180 | | | | |

The course gives this default standing pose (raw ROS order, index 16 = head): `[90,90,90,90,40,15,90,60,76,110,90,90,120,104,70,90,90]`. The left/right ordering is inconsistent in the course ⚠.

## 2. Software access points (on the robot)

| What | Where |
|---|---|
| REST API (all of YanAPI wraps this) | `http://<IP>:9090/v1/...`, with Swagger UI at `http://<IP>:9090/v1/ui` |
| JupyterLab (20 built-in examples) | `http://<IP>:8888` |
| MJPEG camera stream (`open_vision_stream`) | `http://<IP>:8000` |
| SSH / VNC | user `pi`, password `raspberry`. Hostname looks like `Yan_XXXX` (last 4 MAC digits) |
| Core service | `ubtedu-ros` (`sudo service ubtedu-ros stop/start`) |
| ROS | master on the robot at :11311. `source /opt/ros/kinetic/setup.bash` + `/opt/yanshee/setup.bash` (or `/mnt/yanshee/setup.bash` per the course). Messages are `ubt_msgs`. Nodes: `/hal /motion_centre /servo_centre /sysconfig /ubt_blockly_agent /ubt_crp /upgrade_node /vision /voice /voice_tuling` |
| Raw UDP (from the course) | single servo: `127.0.0.1:20001` `{"cmd":"servo","type":"write","time":35,"angle":"<17 hex bytes, FF=skip>"}`. All robots on the LAN: broadcast to `:20005` `{"cmd":"action","type":"start","para":{"name":"Waka_waka","repeat":1}}`, with `name:"stop"` to stop |
| Chest-button events | ROS topic `hal_button_info` (Int32: 1 = single press, 2 = double press). Useful as an operator trigger. |
| User files | music `/home/pi/Documents/music` (wav/mp3), motions `/home/pi/Documents/motions`, photos `/tmp/photo`. The course puts hand-made `.hts` files in `/mnt/1xrobot/res/hts` |
| Other front-ends | Yanshee mobile app (iOS/Android: Wi-Fi setup, joystick, PRP pose recording, Blockly, servo calibration, sensor view, fall-management toggle), uCode, PC Blockly, ROS SDK |

**Finding the IP:** the official way is the mobile app (Bluetooth, then Wi-Fi provisioning; the robot says "Connected"). Alternatives: plug HDMI, keyboard and mouse into the robot and run `hostname -I`, or check the router's DHCP list. IPs used in past sessions: 192.168.3.13 (`IP Address.txt`), 192.168.0.26 (Cairo), 10.146.138.108.

## 3. YanAPI (Python SDK running on the laptop)
- File: `yanshee/yanshee_examples/yanshee_examples/YanAPI.py` (official, 5870 lines; all 7 copies are identical). `YanAPI_en.py` is the same code with English comments.
- Laptop dependencies: `requests`, `nest_asyncio`, `opencv-python` (cv2 is imported at the top, so it is required). The bundled `site-packages/cv2` is a **Windows** build (OpenCV 4.5.5, .pyd) and does not work on Linux.
- On the robot the module is `from YansheeRobot import YanAPI`. On a PC with YanAPI.py next to your script, use `import YanAPI`.
- Every call returns `{"code":0,"data":...,"msg":...}`. Known codes: `6 lowbattery` (motion refused, roughly below 30%) and `20003` (task already running / stream reused).

### Key calls
- **Init:** `yan_api_init(ip)`
- **Device:** `get_robot_battery_info()` → percent / charging / voltage(mV); `set_robot_volume(0-100)`; `set_robot_language('en'|'zh')`; `get/set_robot_fall_management_state(bool)` (auto get-up, off by default); `get_robot_version_info('core'|'servo'|'sn')`; `get_robot_mode()` (energy_saving / calibration)
- **LEDs:** `set_robot_led(type, color, mode)`. `button` (chest) takes white/red/green/blue/yellow/purple/cyan and on/off/blink/breath. `camera` (eyes) takes red/green/blue and on/off/blink.
- **Motions:** `start_play_motion / sync_play_motion(name, direction, speed, repeat 1-100, version 'v1'(hts)|'v2'(layers))`, plus `pause_/resume_/stop_play_motion()`, `get_motion_list()`, `upload_motion(path)` and `get_current_motion_play_state()`.
  - Built-in poses: `reset`, `raise`, `crouch`, `stretch`, `come on`, `wave` (left/right/both), `bend`, `turn around` (left/right), `walk` (forward/backward/left/right), `head` (forward/left/right), `bow`. Get-up motions: `GetupFront`, `GetupRear`.
  - Dances (each with music): `WakaWaka`, `LittleApple`, `GangnamStyle`, `HappyBirthday`, `SorrySorry`, `WeAreTakingOff`. `start_play_motion` returns `total_time` in ms (WakaWaka = 83150).
  - v2 layer examples: `H_WaveRH`, `F_MoveF_A`.
  - Speeds: `very slow`, `slow`, `normal`, `fast`, `very fast`.
- **Gait:** `control_motion_gait(speed_v -5..5, speed_h -5..5, steps (0 = endless!), period 1-5, wave)`, `exit_motion_gait()`, `get_motion_gait_state()` (status 0–8; 8 = idle).
- **Servos:** `set_servos_angles({name: deg}, runtime 200-4000 ms)`, `sync_set_servo_rotate`, `get_servos_angles([names])`, `set_servos_angles_layers({name:{angle,isNeedBessel,runtime}})`, `set_servos_mode('work'|'program', [names])`. `program` mode makes a servo limp so you can pose it by hand and read the angle back.
- **Music:** `start_play_music(name)`, `stop_play_music()`, `get_media_music_list()`, `upload_media_music(path)`
- **Voice:**
  - TTS: `start_voice_tts(text, interrupt)` / `sync_do_tts(text)`. English works.
  - `sync_do_voice_asr()` (endpoint `voice/asr`) is semantic Q&A (Tuling). `sync_do_voice_iat()` (endpoint `voice/iat`) is dictation. **Both are Chinese cloud services** ⚠.
  - Offline command grammar: `create/update_voice_asr_offline_syntax` (Chinese examples).
  - Naming trap: inside `Voice()`, the methods called `asr` actually hit `voice/iat`, and the methods called `nlp` hit `voice/asr`.
- **Vision:**
  - Face: `sync_do_face_recognition(type)` with `recognition | quantity | age | age_group | gender | expression | mask | glass`. Results come back in `data.analysis.*`, `data.recognition.name` or `data.quantity`.
  - Other: `sync_do_gesture_recognition()` (ok/good/bad/victory/heart_hand_sign/digital_6_gesture), `sync_do_color_recognition()`, `sync_do_object_recognition()` (44 classes), `sync_do_QR_code_recognition(timeOut)`, `start_aprilTag_recognition`, `start_object_tracking(name)`, `start_face_recognition('tracking')` (head follows a face).
  - Photos and face database: `take_vision_photo(res)` → `get_vision_photo(name, dir)`; face enrolment with `do_face_entry(name)` (photo → `upload_vision_photo_sample` → `set_vision_tag`); `get_vision_tags`, `delete_vision_tag`.
  - Annotated stream: `show_visions_result('face_recognition_remote'|...)`.
- **Sensors:**
  - Gyro/IMU: `get_sensors_gyro()` (gyro / accel / compass / euler).
  - Add-on modules: `get_sensors_infrared()` (mm), `get_sensors_ultrasonic()` (mm), `get_sensors_touch()` (0/1/2/3), `get_sensors_pressure()`, `get_sensors_environment()` (°C / %RH / hPa) and `get_sensors_list()`.
  - Factory I2C ID ranges: ultrasonic 17–22, IR 23–28, touch 29–34, pressure 35–40. A missing sensor returns an empty list.
- **Push subscriptions (robot POSTs to your URL):** `start_subscribe_sensor / vision / motion / voice_*`
- **Gamepad:** `get/set_gamepad_keymap` maps Bluetooth gamepad keys to motion files.

## 4. Known bugs and pitfalls (important for an unattended exhibition)
1. ⚠ **The official `sync_*` waits have no timeout.** `__wait_result*` loops until the robot reports `idle` with the matching timestamp. If the timestamp never matches, the loop spins **without sleeping**, so it can hang and burn CPU. Wrap vision and voice calls in your own timeout (the Cairo `patch_face.py`/`patch_object.py` show one approach: poll, 8 s timeout, explicit stop).
2. Vision `data` sometimes comes back as the string `"{}"`. Parse it defensively.
3. `sync_do_face_recognition_value` checks `if not startSuccess` on a dict, so the check never fails and errors are not caught.
4. `get_ip_address()` uses `fcntl`, but that import is commented out. `ukit_controller` uses `lib_send`, which is also commented out. Both are broken.
5. Vision tasks use timestamps in seconds; motion and gait use milliseconds.
6. The camera is a single resource. Opening a stream while recognition runs gives "Resource is not available".
7. `control_motion_gait(steps=0)` walks practically forever, so always pass a step count or call `exit_motion_gait()`.
8. Example script bugs:
   - `0custom0/dance_.py`: `DANCE_SPEED` is undefined, so it raises NameError.
   - `servos_samples/servos_samples.py`: has an indented module-level docstring, so it raises IndentationError.
   - `sensors_samples/sensor_samples.py`: has an infinite IR loop and the rest of the file is commented out.
   - `dance_samples.py`: only WakaWaka is active.
   - More bugs in the course scripts are listed in section 12 of `YANSHEE_COURSE_DIGEST.md`.
9. Cairo's `robot_speak()` sends a nested payload (`{"tts": "<json string with languageType EnUs>"}`), while YanAPI sends `{"tts": text, "interrupt", "timestamp"}`. Which format the firmware expects is unverified, so test both.
10. The Instructor Guide's claim that "YanAPI must be patched" only applied to their truncated 2843-line copy. The full YanAPI.py here already contains the vision and sensor functions.

## 5. Safety and operational limits
- **Emergency stop** is on top of the head (cuts power). To interrupt a motion, press the chest button or call `stop_play_motion()` / `start_play_motion("Stop")`.
- Use the robot only on flat, level surfaces away from edges. Do not run it while charging. Do not force joints or the head.
- **Run it for at most about 1 hour at a time** (servo life). Servos overheat under load: the servo LED flashes and the joint stops responding. Recover by powering off and letting it cool, and **schedule rest periods**.
- Motions are refused below roughly 30% battery (`lowbattery`).
- Auto get-up (fall management) is off by default. It can pinch hands while someone is picking the robot up. The course's fall-detection thresholds look wrong (the robot reads about −179.5° while upright), so **measure the real Euler angles yourself before relying on them.**
- Auto-update runs when the robot is online. Do not power off during an update, and consider keeping the exhibition network offline so no update starts mid-show.
- To pick the robot up, hold it firmly by the back. Ages 14+.

## 6. Upgrades and recovery
- The robot updates itself (OTA) when it has internet. `manual_upgrade_step3_*.jpg` shows the Pi 3 inside the opened chest with an arrow at the **microSD slot**, so a manual upgrade means re-flashing the SD card. Back up the SD card before the exhibition.
- To free CPU for custom on-robot work (for example NCNN models), stop `ubtedu-ros`. The NCNN guide uses the cp35 armv7l wheels and `onnx-simplifier==0.2.18`.

## 7. Online resources (as referenced in the docs)
- SDK: https://github.com/UBTEDU/Yanshee-Raspi-SDK
- Developer site / API docs: http://yandev.ubtrobot.com/#/en
