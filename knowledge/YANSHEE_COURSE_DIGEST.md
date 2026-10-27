# Yanshee Course — Factual Digest

Source: "Yanshee Course" (GateIn Technology / UBTECH branding, 411 pages; footer "www.gateintech.com / www.ubtrobot.com").
Read in full from both text extractions (`CourseDocx.txt`, `Yanshee Course.txt`) and cross-checked. Key figures were rendered from the PDF and inspected (pp. 52, 64, 74, 76, 83–85, 160–163, 169–170, 176–184, 189–200).
Page numbers ("p.N") refer to the PDF page footer.

Legend:
- **[VERBATIM]** marks text copied exactly from the course.
- **[AMBIG]** marks places where the course is unclear, contradicts itself, or has a bug.
- **[ZH]** marks content that is specific to Chinese or written in Chinese.
- **[NOT IN COURSE]** marks things you asked about that the course does not cover. I flag these so you know to look in other documents. I have not filled them in from memory.

---

## 0. Read this first: what this course is and is not

- This is an **educational curriculum**: Python, Linux, ROS, kinematics, sensors, ML/DL, speech and vision. It is **not** an operations, setup or bring-up manual.
- **The course does NOT cover any of the following [NOT IN COURSE]:**
  - JupyterLab or port 8888
  - SwaggerUI, or the REST API on port 9090 (no endpoint paths appear anywhere)
  - first boot
  - Wi-Fi provisioning steps (app, AP/hotspot mode, wpa_supplicant)
  - default passwords (the string "raspberry" never appears as a password)
  - SSH or VNC setup steps (it only *mentions* using VNC/SSH/HDMI)
  - OTA/update procedure
  - factory reset
  - servo calibration or zeroing
  - battery capacity, charging or run time
  - LED colour tables beyond 1/2/3
  - the music list
  - the full built-in motion list
  - `.layers` files
  - POGO pins, GPIO pinouts or I2C addresses
  - systemd service names
  
  A grep for `8888`, `9090`, `swagger`, `password` (only a generic Wi-Fi sentence), `calibrat` (only OpenCV camera calibration), `battery` (only the ROS service name) and `charg` (only capacitor physics) confirms this.
- **YanAPI usage in the course is limited to:**
  - `yan_api_init`
  - `set_servos_angles`
  - `sync_play_motion`
  - `get_sensors_infrared` / `_ultrasonic` / `_touch` / `_pressure` / `_environment` / `_gyro`
  - `start_voice_tts`
  
  YanAPI is a wrapper over the robot's HTTP REST API. The course says the HTTP REST API exists ("RestfulAPI based on the http protocol", p.~103) but never documents it.
- The course is the only source in it for:
  - the **internal ROS graph** (nodes, services, topics, `.srv` layouts)
  - **raw UDP JSON command ports** 20001 and 20005
  - the `hal_angles_set` raw servo topic and its 0–2048 angle encoding
  - the default 17-servo pose array
  - the fall-detection Euler-angle thresholds
  - the motion file directory `/mnt/1xrobot/res/hts`
  - the ROS environment file `/mnt/yanshee/setup.bash`
- Language and runtime: almost all examples are **Python 2.7** (`print x`, `raw_input`, `except X, e`). The text says "all the examples in this document are implemented and interpreted based on the Python 2.7 programming language" (p.2). The newer YanAPI snippets use `print(...)`, and one lesson says to run `python3 ubtReadSensorValue.py` (p.163). Mixed.
- Hostname and user evidence from screenshots: prompts show `pi@Yan_0A7E` and `pi@Yan_672C`. So the user is `pi` and the hostname pattern is `Yan_XXXX` (4 hex characters, probably from the MAC). The home directory is `/home/pi` ("the default port path of Yanshee is /home/pi" [sic], p.~18). In the ROS lesson the text instead says the default hostname is `raspberrypi` (p.~66) **[AMBIG: depends on image version]**.
- An old SDK path appears in screenshots: `~/yanshee-raspi-sdk/output/python/example` (p.163, p.170). This is the old "RobotApi" C/Python SDK; see §9.

---

## 1. Chapter / lesson map (course's own structure)

| # | Chapter | Lesson | PDF pages | What it teaches |
|---|---|---|---|---|
| 1 | Basic Application Course | Python Basis of AI | 2–16 | Python 2.7 basics, OOP, threads, sockets, Tkinter calculator |
| | | Frequently Used Linux Basic Commands for Robot Terminals | 17–25 | ls/pwd/cd/touch/mkdir/cat/nano/rm/man/cp/mv/tail/tar/dpkg on the robot |
| | | Desktop Blockly Instructions | 26–34 | On-robot "pc-blockly" desktop app, block categories, face-recognition demo |
| | | Introduction to OpenCV | 35–41 | grayscale, motion detection with PiCamera + OpenCV |
| 2 | Multi-Robot Communication System | Centralized Control of Robots and Network Communication | 42–47 | Network theory; **raw UDP control (ports 20001, 20005)**; multi-robot synchronized dance |
| | | Standard ROS and Basic Robotic Communication | 48–62 | ROS basics; **Yanshee ROS node/service list**; servo/LED/TTS/vision/button via ROS |
| | | Use ROS to Communicate with Robots | 63–70 | PC↔robot, robot↔robot, one→many ROS; recommends UDP broadcast instead |
| 3 | Robot Kinematics | Getting started with Servo Rotations | 71–77 | 17× 180° servos; `set_servos_angles({"NeckLR":..})`; head range 30–150° |
| | | Servo Readback and Continuous Action Execution | 78–87 | Digital bus servos, power-off/"softened", self-protection; **app PRP readback programming**; `sync_play_motion`; `.hts` location |
| | | Gazebo-Based Robot Motion Simulation | 88–103 | URDF, rviz, Gazebo, ros_control controllers for head/arms |
| | | Forward Kinematics | 104–121 | D-H parameters, 3-DoF arm FK in Gazebo |
| | | Inverse Kinematics | 122–139 | 3-DoF arm IK on the **real robot** via ROS topic `hal_angles_set` |
| | | Gait Control | 140–156 | ZMP, static walking, simple IK gait on the real robot (`Yanshee_Gait`) |
| 4 | Robot Sensoring System | Infrared | 157–164 | IR distance add-on sensor; `get_sensors_infrared` |
| | | Ultrasonic | 165–170 | `get_sensors_ultrasonic` |
| | | Capacitive Touch | 171–178 | `get_sensors_touch` (0/1/2/3) |
| | | Pressure | 179–185 | thin-film 2–20 kg; `get_sensors_pressure` |
| | | Temperature & Humidity | 186–192 | `get_sensors_environment` (temp/humidity/pressure) |
| | | Built-in Motion Sensors | 193–202 | MPU9250 IMU; `get_sensors_gyro`; **fall detection + get-up script** |
| 5 | Machine Learning | Application of ML | 203–214 | sklearn decision tree, KNN, linear regression, SVM |
| | | Math Foundations & Tools | 215–237 | calculus/linear algebra/Bayes; sklearn, numpy, scipy, matplotlib, pandas |
| | | ML Algorithm Advanced | 238–250 | locally-weighted LR, logistic regression, K-means |
| | | Perceptron | 251–255 | 2-layer numpy net |
| | | CNN | 256–266 | LeNet on MNIST (Keras) |
| | | RNN | 267–277 | LSTM poetry generator (TF1) |
| | | Open Source Frameworks | 278–286 | Caffe five-class classifier (Ubuntu, CPU) |
| | | Deploy Model to Robot | 287–301 | Train on Ubuntu VM → **install TF 1.11 / Keras on the robot Pi** → run h5 models |
| 6 | Robot Speech System | Principles of Sound Propagation | 302–311 | PCM, bitrate; record with pyaudio, play with sox |
| | | Robotic Speech Recognition | 312–319 | **PocketSphinx (offline, English)** build and keyword model |
| | | Robot Speech Functions | 320–324 | **eSpeak TTS (offline, English)**; read sensor values aloud |
| | | Simple Application of a Speech | 325–333 | Keras 2-word ("seven"/"stop") classifier |
| | | Machine Translation Model | 334–340 | seq2seq English→Chinese **[ZH data]** |
| 7 | Robot Vision Recognition | Image & Video recording | 341–352 | raspistill, picamera photo/video/OpenCV stream |
| | | Color recognition in Camera | 353–363 | HSV thresholds red/green/blue/yellow; traffic-light demo |
| | | The colorful world of AI | 364–379 | AI overview; TF cat-vs-dog CNN (trained on PC) |
| | | Face detection | 380–394 | OpenCV Haar cascade on PiCamera |
| | | Face recognition | 395–411 | PCA/LDA (eigenface/fisherface) on ORL att_faces |

There are no appendices in the extracted text. Page ~51 says "For details, please refer to: Appendix Yanshee_ROS message summary", but **that appendix is not in this document [AMBIG/missing]**.

---

## 2. Chapter 1 — Basic Application Course

### 2.1 Python Basis of AI (p.2–16)
- Recommends Jupyter Notebook (generic link `http://jupyter.org/install.html`). There is **no mention of on-robot JupyterLab or port 8888**.
- "The Yanshee robot command line is installed with Python by default... open the terminal python xxx.py".
- Edit with `nano hello.py` and run with `python hello.py`. Covers Python 2 syntax, a UTF-8 header for Chinese text **[ZH]**, modules, packages, classes, `threading`, and TCP sockets (`server.py`/`client.py`, port 12345, "On a Yanshee robot, open two command line terminals").
- Covers Tkinter (`import Tkinter`, with a note that it is `tkinter` in Py3) and a calculator ("Yanshee's calculator").
- No robot-specific API is used here.

### 2.2 Linux Basic Commands (p.17–25)
- "Connect to an HDMI monitor or VNC to log in to the robot". The terminal icon is at the top-left of the Raspberry Pi desktop, or press `Ctrl + Alt + T`.
- `pwd` shows `/home/pi`.
- Generic usage of `ls, pwd, cd, touch, mkdir, cat, nano` (keys: Ctrl+O save, Ctrl+X exit, Ctrl+K cut, Ctrl+U paste, Alt+6 copy, Ctrl+W search), plus `rm, man, cp, mv, tail -f, tar, dpkg -i/-r/-P/-L/-l/-c/-S/--unpack/--configure`.
- Note: the docs call the OS "Yanshee's Debian system", and elsewhere "Raspberry Pi Raspbian system".

### 2.3 Desktop Blockly Instructions (p.26–34)
- **"pc-blockly"** is a desktop icon on the robot's own desktop. Reach it via HDMI or VNC and double-click. The UI shows a landing page, then the main page:
  - left: block palette
  - top: settings bar (Language, File, Edit, Tool, More)
  - right: generated **Python** code
  - bottom-right: **Run** button
- Language: **Simplified Chinese / English** switch.
- Tool menu: "connect the robot to the local PC by default, or read the sensor data of the current robot from the sensor reading window."
- More menu: version number and feedback email.
- Block categories (verbatim): **Event, Action, Show, Sensors, Logic, AI, Loops, Math, List, Variables, Functions**.
  - **Event**: "detecting the touch of the left or right side of the touch sensor with the specific ID", and "detecting colors with the camera".
  - **Action**: "Special action, Arm action, Knee action, Combat action, Movement, Head action, robot camera and steering gear setting". These are category names only; individual motion names are not listed **[NOT IN COURSE]**.
  - **Show**: "robot TTS function, traffic sound, and chest button light color setting".
  - **Sensors**: ultrasonic, IR, temperature and humidity, pressure.
  - **AI**:
    - Face collection: take a photo and enter a name into the face database. The generated block "indicates whether the person who entered in the last 20 seconds is recognized".
    - Detected age: "infants, children, adolescents, middle-aged and elderly".
    - Detected gender: male/female.
    - Detected expressions: "happiness, anger, surprise, calmness, sadness, disgust, and fear" (7 expressions).
    - Number of faces detected.
    - "Text heard: The robot detects if a certain keyword is heard and performs the corresponding action." Built-in ASR language is not stated here; see §8.
- Working principle (verbatim): "Internal modules use the **RobotApi** library function in the format of python scripts to complete the integrated application of the five functions of steering gear, action, sensing, voice and vision. **The SDK and function modules inside a robot communicate in UDP mode.**"
- Exercises:
  - Basic: read the obstacle sensor, broadcast obstacle info, change the chest light colour as an alarm.
  - Advanced: face-count + face-recognition + gender + TTS + action + touch combined program. This is a good exhibition template.
- "Steering gear" is the course's translation of 舵机 = **servo** throughout **[ZH translation artefact]**.

### 2.4 OpenCV intro (p.35–41)
- `gray.py` (`cv2.imread/cvtColor/imshow/imwrite`), run with `cd Desktop/opencv_test; python gray.py`. Needs a display (HDMI/VNC) for `imshow`.
- `motion_track.py`: PiCamera 500×300 @30 fps, `PiRGBArray`, background accumulation, `absdiff`, threshold 5, dilate, contours > 5000 px area, prints `x= y= w= h=`, press `q` to quit.
  - **[AMBIG/bug]** It ends with `camera.release()`; PiCamera uses `close()`.
  - The 2-value `findContours` unpack works with OpenCV 2.4/4.x, not 3.x.
- "The latest version of OpenCV is 4.1" (dates the course to about 2019).

---

## 3. Chapter 2 — Multi-Robot Communication

### 3.1 Internal architecture facts (p.~45)
- "Yanshee implements data transmission between **STM32 and Raspberry Pi through IIC** hardware interface protocol, while **STM32 and sensor** also realize data reading through IIC connection." Sensor block diagrams (p.176, p.184) show `Touch Sensor —IIC BUS→ MCU STM32` and `Pressure Sensor —IIC Bus→ MCU STM32`.
- "Yanshee integrates Bluetooth and WIFI modules."
- "The mobile phone connects with Yanshee through Bluetooth and WIFI, and then transfers data through TCP/UDP protocol... One Yanshee controls another Yanshee through UDP protocol in LAN."
- I2C *addresses* are **not given [NOT IN COURSE]**.

### 3.2 Raw UDP control (p.~46–47) — undocumented elsewhere, very useful

**(a) Single servo over UDP to localhost:20001** ("Controlling one of Yanshee's steering gears by UDP communication").

Payload (JSON string):
```
{"cmd":"servo","type":"write","time":35,"angle":"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF<HH>"}
```
- `HOST = '127.0.0.1'`, `PORT = 20001`, `socket(AF_INET, SOCK_DGRAM)`.
- The `angle` string is 17 two-hex-digit bytes. **`FF` = leave that servo unchanged**; the 17th byte (last) is the head servo. The example `headangle(60)` sends `...FF3c`.
- `time` = 35 (units unstated; probably ×20 ms or ms ticks **[AMBIG]**).
- Code is Python 2 and mis-indented in the extraction.

**(b) Synchronized multi-robot actions: UDP broadcast to 255.255.255.255:20005** (`sc.py`).
```python
send_string = "{\"cmd\":\"action\",\"type\":\"start\", \"para\":{\"name\":\"" + Name + "\",\"repeat\":" + str(Repeat) + "}}"
sendmsg("255.255.255.255", send_string)   # PORT = 20005, SO_BROADCAST
```
- Usage (verbatim): `python sc.py stop`, `python sc.py Waka_waka`, `python sc.py wakawakax4`. An optional 2nd argument gives the repeat count. `default_port = 20006` is defined but unused.
- Motion names in this example: `stop`, `Waka_waka`, `wakawakax4`. Names map to `.hts` files ("action name: Name.hts").
- **[AMBIG]**
  - The text says "Jump wakawaka four times" with `wakawakax4`; that is a separate file name, not repeat=4.
  - Case differs (`Waka_waka` vs `wakawakax4`).
  - The usage message says `action_test.py`.
- Setup steps (verbatim):
  1. "Connect all the robots to the same LAN (wifi hotspot)."
  2. "Log on to a robot with VNC or other SSH tools or connect the robot with an HDMI cable. (You can use the mobile app to view the IP address of the robot first)."
  
  This is the **only** IP-discovery method the course gives: the **mobile app**.
- Exhibition relevance: one broadcast packet makes every robot on the subnet perform the same named motion, which gives a synchronized dance. Broadcast requires the AP to allow client-to-client broadcast (no "AP isolation"); this is my inference, not in the course.

### 3.3 Yanshee ROS graph (p.52–55)
ROS Kinetic. The **screenshot** of `rosnode list` on `pi@Yan_0A7E` shows:
```
/hal  /motion_centre  /rosout  /servo_centre  /sysconfig  /ubt_blockly_agent  /ubt_crp  /upgrade_node  /vision  /voice  /voice_tuling
```
(The PC-side screenshot on p.64 additionally shows `/feedback_node`.)

The text says "Includes 10 nodes" and describes them as follows:

| Node | Description (verbatim-ish) |
|---|---|
| `/hal` | All hardware-related abstraction layer services |
| `/Motion_centre` (actually `/motion_centre`) | Motion-related services |
| `/servo_centre` | Servo related services |
| `/sysconfig` | System configuration incl. language, volume |
| `/ubt_crp` | "All interfaces that interact with the outside world... including UDP requests with mobile apps" |
| `/upgrade_node` | System upgrade |
| `/vision` | All visual services |
| `/voice` | Speech recognition and TTS |
| `/voice_tuling` | Semantic comprehension (Tuling = Turing Robot, a Chinese NLP cloud **[ZH]**) |

**Services listed** (spelling as printed; capitalisation errors in the source such as `/Hal_ext_read`, `/Hal_servo_set`, `/Move`):

- `/hal`:
  - `/hal_action`
  - `/hal_battery_read` (power value)
  - `/hal_board_read` (onboard temperature sensor)
  - `/hal_env_read` (temperature, humidity, pressure)
  - `/hal_ext_enable_list_get`
  - `/hal_ext_read` ("Values of five kinds of external sensors")
  - `/hal_fallmanager_config`
  - `/hal_gyro_read`
  - `/hal_led_set`
  - `/hal_sensor_config`
  - `/hal_sensor_report_enable`
  - `/hal_servo_active`
  - `/hal_servo_offset_` ["Reading servo offset values" — name truncated, presumably `/hal_servo_offset_read` **[AMBIG]**]
  - `/hal_servo_offset_write`
  - `/hal_servo_read`
  - `/hal_servo_set`
  - `/hal_status_change`
  - `/hal_upgrade`
  - `/hal_version_read`
  - `/hal_volume_set`
- "/Move" node (presumably `/motion_centre`):
  - `/move_action`
  - `/move_servo_read`
  - `/move_servo_write`
  - `/move_status_change`
- `/sysconfig`:
  - `/sysconfig_fallmanager_get`
  - `/sysconfig_fallmanager_set`
  - `/sysconfig_language_get`, `/sysconfig_language_set` ("supports Chinese and English")
  - `/sysconfig_status_get`
  - `/sysconfig_status_set`
  - `/sysconfig_volume_get`
  - `/sysconfig_volume_set`
- `/upgrade_node`: `/upgrade_set`
- `/vision`:
  - `/vision_ai_event` ("face analysis, gesture recognition and face tracking")
  - `/vision_detect` ("number detection of human faces and hands")
  - `/vision_take_photo`
- `/voice`:
  - `/voice_asr_start`
  - `/voice_asr_stop`
  - `/voice_iat_detectedmsg`
  - `/voice_iat_detectingmsg` ("Supports speech recognition"; IAT = iFLYTEK "intelligent audio transcription" naming **[ZH vendor]**)
  - `/voice_tts_start`
  - `/voice_tts_stop`
  - `/voice_tuling`

Note **`fallmanager`**: there is a built-in fall manager that can be configured (`/hal_fallmanager_config`, `/sysconfig_fallmanager_get/_set`). Its parameters are not documented **[NOT IN COURSE]**. For an exhibition, check whether automatic fall recovery is enabled in the firmware before writing your own (see §6.6).

**Message package:** `ubt_msgs` (`from ubt_msgs.srv import *`). It ships on the robot. For PC use, copy the `ubt_msgs` folder into `~/catkin_ws/src` and run `catkin_make` (p.64). The robot's ROS environment is sourced with **`source /mnt/yanshee/setup.bash`** (p.155).

**Documented `.srv` / topic layouts:**

```
# ubt_msgs/srv/servo_write.srv   (service name 'hal_servo_set')
string buf        # 32×'F' + 2 hex digits (same 17-byte hex mask as UDP); example appends '"}' [AMBIG/bug]
int8 e_servo_type # example uses 1
int8 time         # example uses 10
---
int32 rc
```
```
# led.srv   (service 'hal_led_set')
int8 type   # 1 = chest (button) light, 2 = head light
int8 mode   # 1 = always on, 2 = breathing, 6 = flashing ("blink fast")
int8 color  # 1 = red, 2 = green, 3 = blue, "etc."  (other values not listed)
---
int32 rc
```
- Example: head red solid `(2,1,1)`; head blue `(2,1,3)`; chest blink red `(1,6,1)`; chest "normal state" = **blue breathing** `(1,2,3)`. So the robot's normal chest LED is blue breathing.

```
# voice_tts_start.srv   (service 'voice_tts_start')
int32 handle      # example 1
string msg        # e.g. "Hello I am Yanshee. Welcome! "
bool interrupt    # 1 = can be interrupted, 0 = cannot
bool asyn         # example passes False (variable named `async` — a reserved word in Python ≥3.7)
---
int32 rc
```
```
# vision_ai_event.srv   (service 'vision_ai_event')
string vision_type  # "face_analyse" | "gesture_analyse" | "face_tracking" | "stop_face_tracking"
---
int32 rc
```
- Topic **`hal_button_info`** (`std_msgs/Int32`) is the chest button press count: `1` = single press, `2` = double press. Example node name `button`; with several subscribers use unique names (`button1`, `button2`).
- Topic **`hal_angles_set`** (`ubt_msgs/angles_set`): fields `angles[17]` and `time`. This is the raw all-servo command; details in §5.4.

### 3.4 ROS across machines (p.63–70)
- PC: Ubuntu 16.04 + `sudo apt-get install ros-kinetic-desktop-full`; verify with turtlesim (`roscore`, `rosrun turtlesim turtlesim_node`, `rosrun turtlesim turtle_teleop_key`).
- PC↔robot example: VM in **bridged** mode on the same hotspot; add entries to `/etc/hosts` on both sides. Example values: robot hostname `Yan_0A7E` at 10.10.61.152; VM `ubuntu` at 10.10.60.199. Add the ROS_MASTER_URI lines to `~/.bashrc` (exact lines only in a screenshot) and run `source ~/.bashrc`. `rosnode list` on the PC then shows the robot's nodes. The ROS master runs **on the robot**, port **11311**.
- Robot↔robot: rename a robot with `sudo nano /etc/hostname` (to `pipi`), then in `/etc/hosts` change `127.0.1.1 raspberrypi` to `127.0.1.1 pipi`, then reboot. Then:
  ```
  # on pipi (master, 10.10.61.77):        # on raspberrypi (10.10.60.12):
  /etc/hosts: 10.10.60.12  raspberrypi     /etc/hosts: 10.10.61.77  pipi
  export ROS_HOSTNAME=pipi                 export ROS_HOSTNAME=raspberrypi
  export ROS_MASTER_URI=http://10.10.61.77:11311   (same on both)
  ```
- Bug **[AMBIG]**: `ros_test.py` defines `s(angle)` but calls `set_head_angle()`.
- **Warning (verbatim, p.69):** "At present, the ROS of the robot itself only supports sharing of a single master, and in the entire connected network, the node name must be unique and cannot be repeated. As the default node names of our Yanshee robots are all the same... **we do not recommend using ROS to control multiple robots. It is recommended to use UDP broadcast for more convenient control.** The same is true for PC."

---

## 4. Chapter 3 — Kinematics, servos, motions

### 4.1 Servo basics (p.71–77)
- **17 servos** (Fig. 4, p.74, shows them as blue dots: neck 1, 3 per arm, 5 per leg).
- "Yanshee's servo is a **180-degree servo** with a maximum range of 180 degrees for each joint... 0-180 degrees."
- Hardware for these lessons: "Yanshee robot, a mobile phone with the **Yanshee App** installed."
- Example:
  ```python
  import YanAPI, time
  ip_addr = "127.0.0.1"  # please change to your yanshee robot IP
  YanAPI.yan_api_init(ip_addr)
  res = YanAPI.set_servos_angles({"NeckLR":60}); time.sleep(1)
  res = YanAPI.set_servos_angles({"NeckLR":90}); ...120 ... 90
  print(res)
  ```
  - Only one servo name appears in the course: **`NeckLR`**. Other servo names are **[NOT IN COURSE]**.
  - `"127.0.0.1"` works when the script runs on the robot; use the robot's IP when running from a PC.
- **Head range figure (p.76): 30° (one side) … 90° (centre) … 150° (other side).** So NeckLR is safe within 30–150, with 90 as centre.

### 4.2 Readback, servo states and app PRP (p.78–87)
- Digital bus servos: a serial bus with a per-servo ID, internal 300 Hz drive (analog servos use 50 Hz PWM), and position readback.
- **Power-off ("softened")**: no holding torque, so the joint can be moved by hand.
- **Power-on ("hardened")**: holds position.
- **Self-protection (verbatim, p.82):** "if a large load is placed on the servos or if the servo is tried to manually move with a huge force, after a certain period of time the robot servo moves into a state in which it no longer responds to various controls... If a risk of overheating of servos is detected at any point, it force-stops working... **When Yanshee's servo moves into the self-protection state, the LED on the servo will begin and continue to flash.** This should be used by the user as an indication to **restart the robot and let the servo cool down**."
- **Yanshee App home tiles** (screenshot p.84): Motion Control, Curriculum, Blockly, Community, **PRP**, Sensor.
- **PRP readback procedure:**
  1. Home → "PRP" → "Programming". Tap the top-right button to connect the robot.
  2. Tap the **"M"** (manual readback) button at the lower left. The screen shows "Tap to select the body to be programmed".
  3. Tapping a limb **powers down** its servos (soft). Tapping again powers them up and they "return to its original position".
  4. Pose the limb by hand, then tap **M** again to record a frame. Repeat for more frames. Tap **Preview** to play them back.
  5. The on-screen warning reads: "*Please hold your robot to prevent falling" **[VERBATIM]**.
  6. The edit screen (p.85) shows frames 1…36, a timer, and toolbar buttons **Insert, Tune, Running time, Interval time, Copy, Paste, Mirror, Delete**, plus "Instructions" and **"Add sound to actions"**.
     - "Running time" = time to move into the frame.
     - "Interval time" = wait after the frame before the next one starts.
     - The default readback motion is slow; shorten the running time to speed it up.
  7. **Save** stores the action on the phone ("angular data, execution time/run length and interval duration of each action frame... are stored in a file").
  8. The **Share** button in the action list "send[s] the action file to the Yanshee robot and execute[s] it with the code."
- Python call of a motion:
  ```python
  YanAPI.sync_play_motion(name="raise", direction="left", speed="normal", repeat=2)  # "Raise left", twice
  YanAPI.sync_play_motion(name="reset")
  ```
  Save as `hitleft.py` under `/home/pi` and run `python hitleft.py`.
- **Motion file location (verbatim, p.86):** "Write your own push-up.hts... Name it push_up.hts. Then copy it to the corresponding robot directory and complete the Python code call under **mnt/1xrobot/res/hts**". So custom `.hts` files go in **`/mnt/1xrobot/res/hts/`** and are called by base name, e.g. `sync_play_motion(name="push_up")` (inferred).
- `.hts` internal format: **[NOT IN COURSE]**. `.layers` (UBTECH multi-track/music project format): **[NOT IN COURSE]**.

### 4.3 Gazebo simulation (p.88–103)
- Install:
  - `sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers`
  - `roscore; rosrun gazebo_ros gazebo`
  - If Gazebo crashes ("flashback"): `echo "export SVGA_VGPU10=0" >> ~/.bashrc`
- Model: `https://github.com/shaoyiwork/Yanshee_Fzstart`.
  - Fixes: set the maintainer email; rename the urdf folder to `Robots`; in `display.launch` change `gui` default from False to True.
  - `roslaunch yanshee display.launch` (rviz), `roslaunch yanshee gazebo.launch`.
- Add the `libgazebo_ros_control.so` plugin with namespace `/yanshee`, a fixed `world_joint` (origin z 0.285), and `<transmission>` with `PositionJointInterface`; controller type `position_controllers/JointPositionController`.
- Package `yanshee_gazebo`: `roslaunch yanshee_gazebo yanshee_gazebo.launch`; `rosrun yanshee_gazebo control`.
- Topics (`std_msgs/Float64`, radians):
  - `/yanshee/head_position_controller/command`
  - `/yanshee/left_arm{1,2,3}_position_controller/command`
  - `/yanshee/right_arm{1,2,3}_position_controller/command`
- Tip: on Gazebo segfault run `killall gzserver` and `killall gzclient`.
- Only head and arms are simulated. Legs are left as an exercise.

### 4.4 Forward kinematics (p.104–121)
- D-H method. The 3-DoF arm FK is:
  `x = cos t1 (a2 cos t2 + a3 cos(t2+t3))`, `y = sin t1 (...)`, `z = a2 sin t2 + a3 sin(t2+t3)`.
- Project: `https://github.com/shaoyiwork/Yanshee_Fzarm`.
- Joint limits used (rad):
  - head −0.78…0.78 (≈ ±45°, consistent with 30–150°)
  - arm1 −1.58…1.58
  - arm2 0…3.15
  - arm3 −1.58…1.58
- Axis note: "+y: left hand facing backwards, right hand facing forward".

### 4.5 Inverse kinematics on the real robot (p.122–139)
- Project `https://github.com/shaoyiwork/Yanshee_Arm`. Copy it to the robot Desktop. It contains `src/arm` and `src/ubt_msgs` ("all the built-in messages and services of the robot by default").
- Build and run: `cd Desktop/yanshee_arm/ && catkin_make && source devel/setup.bash && rosrun arm control`.
- Publishes to **`hal_angles_set`** at 50 Hz (see §5.4).
- Picks the IK solution closest to the previous angles. Reports "position error!" or "position limit error!" when there is no solution.

### 4.6 Gait (p.140–156)
- "**Because YanShee is not equipped with a force sensor**, so we will use a fixed walking mode."
- Static walking sequence: shift CoM left, step right, shift right, step left, …
- Project `https://github.com/shaoyiwork/Yanshee_Gait`:
  ```
  cd Yanshee_Gait
  source /mnt/yanshee/setup.bash
  catkin_make
  source devel/setup.bash
  rosrun ik_test ik_test_node
  ```
- Parameters: `LEG_UP_CONDITION 0.006`, `WAIST_LIMIT 0.007`, `GAIT_T 0.8`, `SPEED 0.01`; foot spacing `0.0563` m; waist height `0.15` m; `gait::Kinematics::IK_leg(l_foot, waist, r_foot, l_joint, r_joint)`; `joint_angle_.time = 20`.
- This is an experimental gait. Use built-in walk motions for demos.

---

## 5. Hardware facts gathered across the course

### 5.1 Compute and buses
- Raspberry Pi (model not named in the course) plus an **STM32** MCU. The Pi and STM32 talk over **I2C**, and the STM32 talks to the sensors over **I2C**.
- Bluetooth and Wi-Fi on board.
- Raspberry Pi **CSI camera** in the head (camera position pictures on pp.~350, 360).
- Microphone(s) for recording: pyaudio at 16 kHz mono 16-bit works (p.~309).
- Speaker: `aplay`, `sox play` work.
- Arch: **armv7l** (TensorFlow wheel `cp27-none-linux_armv7l`).

### 5.2 Servos
- 17 bus servos, 180°, readback-capable, each with a status LED that flashes in self-protection.
- Name → ID mapping: **[NOT IN COURSE]** except `NeckLR` (head, array index 16).

### 5.3 Sensors (see §7 for API)

| Sensor | Built-in or add-on | Notes |
|---|---|---|
| IMU **MPU9250** (MPU6050 6-axis + AK8963 3-axis compass), DMP Euler angles | **built-in**, in torso (p.199 figure) | `get_sensors_gyro` |
| Infrared distance | add-on module labelled "infrared sensor **ID:23**" (p.160–161) | `get_sensors_infrared`, value in **mm** (screenshot p.163) |
| Ultrasonic distance | add-on; mounted on lower chest/belly in photo (p.169) | value in **mm** (screenshot p.170) |
| Capacitive touch | add-on; mounted on the arm (p.176); I2C to STM32 | value 0/1/2/3 |
| Pressure (thin-film piezoresistive, 2–20 kg) | add-on; mounted on the torso (p.184); I2C | |
| Temperature/humidity/(air pressure) "environment" | add-on module (p.190) | temp ℃, humidity %RH, pressure hPa |
| Onboard board temperature | built-in (`/hal_board_read`) | |
| Battery level | built-in (`/hal_battery_read`) | no API example, no capacity data |

- The add-on sensors are small black modules with an ID label. They attach by magnetic POGO connectors on the real robot, but **the course never mentions "POGO"** **[NOT IN COURSE]**; it shows only "Hardware Connection" images, some of them blank.
- Pressure table (kg → kΩ): 2→606, 3→400, 4→354.3, 5→332.5, 6→315, 8→294.6, 10→280.3, 12→271.9, 14→262.5, 16→256.2, 18→251.4, 20→246.

### 5.4 Raw servo array (`hal_angles_set`) — encoding and default pose
- `ubt_msgs::angles_set { angles[17] (int), time }`.
- Encoding: `int(deg * 2048 / 180)`, i.e. "the angular value encoding format of the actuator **0~2048**" for 0–180°.
- `time`: 25 (init) or 20 (loop). Units are not stated **[AMBIG]**.
- Default/initial pose used in the IK lesson (index: degrees):
  ```
  [0]=90 [1]=90 [2]=90 [3]=90 [4]=40 [5]=15 [6]=90 [7]=60 [8]=76 [9]=110 [10]=90 [11]=90 [12]=120 [13]=104 [14]=70 [15]=90 [16]=90
  ```
- The gait lesson uses `[1]=130, [2]=179` (arms down) for the same array.
- Index meaning, per IK `transform()`:
  - `[16]` = head
  - `[0],[1],[2]` = **right** arm joints 1–3 (shoulder roll?, shoulder pitch, elbow)
  - `[3],[4],[5]` = **left** arm 1–3
  - `[6..10]` and `[11..15]` = the two legs (5 DoF each)
- **[AMBIG]**: the gait code writes `left_arm` into `[0]` and `right_arm` into `[3]`, labels `[6..10]` as left leg and `[11..15]` as right leg. That contradicts the IK lesson's arm sides. Verify on hardware with small moves before relying on it.
- Leg neutral offsets in the gait code: `[6]=90+, [7]=98+, [8]=128−, [9]=94−, [10]=90+, [11]=90+, [12]=82−, [13]=52+, [14]=86+, [15]=90+` (sign = direction of the IK angle added).

### 5.5 File paths and commands on the robot (everything the course mentions)
- `/home/pi` — default home. Scripts are run from here or from `~/Desktop`.
- `/mnt/1xrobot/res/hts/` — motion (`.hts`) files.
- `/mnt/yanshee/setup.bash` — ROS environment for the robot's workspace.
- `ubt_msgs` package (on the robot; copy it to the PC to build clients).
- `~/yanshee-raspi-sdk/output/python/example` — old RobotApi SDK examples (screenshots only).
- `/home/pi/data/haarcascades/haarcascadefrontalface.xml` — Haar cascade path used in the face-detection lesson. Probably the author's own copy; it may not exist on your image **[AMBIG]**.
- `/etc/hostname`, `/etc/hosts` (`127.0.1.1 <hostname>`), `~/.bashrc` — ROS networking.
- Process/service names: only the ROS node names in §3.3. systemd unit names are **[NOT IN COURSE]**.

---

## 6. Motion API details (exact strings)

### 6.1 `YanAPI.sync_play_motion(...)`
Calls that appear in the course:
- `sync_play_motion(name="raise", direction="left", speed="normal", repeat=2)`
- `sync_play_motion(name="reset")`
- `sync_play_motion(name="GetupFront")`
- `sync_play_motion(name="GetupRear")`

"sync" means the call blocks until the motion finishes (implied by the name; not stated).

### 6.2 Motion names seen anywhere in the course
- `raise` (with `direction="left"`)
- `reset`
- `GetupFront`
- `GetupRear`
- `stop`
- `Waka_waka`
- `wakawakax4` (UDP)
- `push_up` (user-made example)

Prose also mentions squat/squatting, waving, walking, "Raise left", and getting up. The text calls the get-up motions `getup_in_front` / `getup_in_back` "(built-in action)", but the code uses `GetupFront` / `GetupRear` **[AMBIG: trust the code; verify against the robot's motion list]**.

The full list of names, valid `direction` values (only `left` shown) and `speed` values (only `normal` shown) is **[NOT IN COURSE]**.

### 6.3 `YanAPI.set_servos_angles(dict)`
Only `{"NeckLR": deg}` is shown. Returns `res` (printed; format not shown).

### 6.4 Motion via UDP or ROS
See §3.2 (`{"cmd":"action","type":"start","para":{"name":..,"repeat":N}}` on UDP 20005). The `/move_action`, `/hal_action` services exist; their srv layouts are not given.

### 6.5 Custom motions
Two routes:
- App PRP readback → Share to robot (§4.2).
- Hand-author an `.hts` file and copy it to `/mnt/1xrobot/res/hts`.

Script-level choreography is a third route, via `set_servos_angles` sequences or `hal_angles_set`.

### 6.6 Fall detection / get-up (p.193–202) — exhibition-critical
```python
#!/usr/bin/env python
import YanAPI            # NOTE: script also needs `import time` (missing in the course)
YanAPI.yan_api_init("127.0.0.1")
while True:
    time.sleep(1)
    res = YanAPI.get_sensors_gyro()
    if len(res["data"]["gyro"]) > 0:
        x = res["data"]["gyro"][0]["euler-x"]   # also "euler-y", "euler-z"
        if x > 160 or x < -160:
            YanAPI.start_voice_tts("Detected fall backward, I am going to get up", False)
            YanAPI.sync_play_motion(name="GetupRear")
        elif -20 < x < 20:
            YanAPI.start_voice_tts("Detected fall forward, I am going to get up", False)
            YanAPI.sync_play_motion(name="GetupFront")
```
- Rule (verbatim): "When the angle of the x-axis is between -20° and 20°, the robot falls forwards. When the x-axis angle is <-160° or >160°, the robot falls backward."
- **[AMBIG — important]** The screenshots on p.200 contradict this:
  - labelled "Running forward results" (upright): `angle_x: -179.51`, `angle_y: -1.95`, `angle_z: -71.18`
  - "Tripping while running forward": `angle_x: 0.49`
  
  If upright really reads about ±180°, this script would fire `GetupRear` while the robot is **standing**. **Measure `euler-x` (and y) on your unit in each posture (standing, face-down, face-up) before using these thresholds.** Consider combining the x and y axes and requiring the posture to persist for N seconds.
- **Safety warning (verbatim, p.201):** "if the robot is in the process of being picked up and the current stage is just in a state of falling, the robot performs a climbing action. **This can easily pinch your hand.**" The suggested fix is to distinguish a fall from a pick-up using speed (gyro rate plus acceleration).
- The built-in fall manager services (`/hal_fallmanager_config`, `/sysconfig_fallmanager_get|set`) exist, so do not run two fall-recovery loops at once (my inference).

---

## 7. Sensor API (YanAPI) — exact access patterns and value formats

All examples use `YanAPI.yan_api_init(ip)` first. Each getter returns a dict; the data lives under `res["data"][<type>]` as a **list** (empty when no sensor is attached). Each element has `"value"` (and `"id"`).

| Function | Access | Meaning / units |
|---|---|---|
| `YanAPI.get_sensors_infrared()` | `res["data"]["infrared"][0]["value"]`, `[0]["id"]` | distance, **mm** per screenshot; eSpeak lesson prints "mm" but speaks "centimeter" **[AMBIG]** |
| `YanAPI.get_sensors_ultrasonic()` | `res["data"]["ultrasonic"][0]["value"]` | distance, mm (screenshot) |
| `YanAPI.get_sensors_touch()` | `res["data"]["touch"][0]["value"]` | `0` none, `1` left, `2` right, `3` both |
| `YanAPI.get_sensors_pressure()` | `res["data"]["pressure"][0]["value"]` | printed with "N"; sensor rated 2–20 kg **[AMBIG unit]** |
| `YanAPI.get_sensors_environment()` | `res["data"]["environment"][0]["temperature"/"humidity"/"pressure"]` | ℃, %RH, hPa |
| `YanAPI.get_sensors_gyro()` | `res["data"]["gyro"][0]["euler-x"/"euler-y"/"euler-z"]` | degrees (Euler from DMP); full element printed in example |

- The IR lesson's "Code Description" is **stale** (verbatim): "import... the RobotApi library, then enter the robot name and connect the robot through the **ubtRobotDiscovery()** function and the **ubtRobotConnect()** function". That is the old SDK, and it does not match the YanAPI code shown.
- Sensor demo ideas from the course, all usable at an exhibition:
  - IR "Magic Hands": <20 cm retreat then squat; >20 cm walk forward; >30 cm stop and wave goodbye.
  - Ultrasonic obstacle bypass (~25 cm obstacle), saying "Obstacles discovered, danger ahead!" and then "I have successfully bypassed obstacles. Please give other instructions."
  - Handshake: robot extends its arm and says "Hello, can we shake hands?"; on touch it shakes and says "Thank you so much! I'm your intelligent robot assistant Yanshee, I wish you a happy time!".
  - Pressure-driven song tempo.
  - Clothing advice from temperature (>20 °C short sleeves, <10 °C jacket).

---

## 8. Voice / speech

### 8.1 Built-in voice stack (from the ROS graph)
- `/voice` node: ASR start/stop, "IAT" detecting/detected messages, TTS start/stop.
- `/voice_tuling`: "semantic comprehension". Tuling (图灵机器人) is a Chinese cloud chatbot. These names strongly suggest the built-in ASR/NLP is **iFLYTEK + Tuling, cloud-based and Chinese-centric [ZH]**.
- `/sysconfig_language_get|set` "supports Chinese and English".
- **The course never states what language the built-in ASR understands, never documents a wake word, and never shows `YanAPI` ASR calls [NOT IN COURSE].** Blockly's "Text heard" block exists, but its language is not specified.
- `YanAPI.start_voice_tts(text, False)` is the only YanAPI TTS call shown, with English text. The 2nd argument is presumably the "interrupt" flag, mirroring the ROS srv **[AMBIG]**.
- ROS TTS: `voice_tts_start(handle=1, msg, interrupt=1, asyn=False)`.

### 8.2 Offline English alternatives taught by the course (useful if cloud ASR is Chinese-only)
- **PocketSphinx** (p.312–319), built from source on the robot:
  ```
  wget http://sourceforge.net/projects/cmusphinx/files/sphinxbase/5prealpha/sphinxbase-5prealpha.tar.gz
  ./configure --enable-fixed; make clean all; sudo make install
  wget http://sourceforge.net/projects/cmusphinx/files/pocketsphinx/5prealpha/pocketsphinx-5prealpha.tar.gz
  ./configure; make clean all; sudo make install
  export LD_LIBRARY_PATH=/usr/local/lib ; export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
  ```
  - Without the exports you get "error while loading shared libraries: libpocketsphinx.so.3". To make it permanent, edit `/etc/ld.so.conf` and run `ldconfig`.
  - Build the language model by uploading a corpus to `http://www.speech.cs.cmu.edu/tools/lmtool-new.html` and downloading the `.dic` and `.lm` files.
  - Run: `pocketsphinx_continuous -hmm /usr/local/share/pocketsphinx/model/en-us/en-us -lm 3199.lm -dict 3199.dic -samprate 16000/8000/48000 -inmic yes`. Pick one sample rate.
  - "Input overrun, read calls are too rare (non-fatal)" is normal.
  - The example dictionary has command words: GARAGE, OPEN, PI, RAMONA, SHUTDOWN, START, TIME, TRANSLATOR, WEATHER, WHAT, OKAY. This works as a crude offline keyword spotter.
  - Hardware listed: HDMI monitor + wireless keyboard/mouse; software "Xshell" (an SSH client), which implies SSH access works.
- **eSpeak TTS** (p.320–324):
  - Install with `sudo apt install espeak`.
  - Basic: `espeak -ven+f3 -k5 -s150 "Hello, I am Yanshee"`.
  - In code: `espeak -ven-us+m7 -a 200 -s180 -k20 --stdout '<text>' | aplay`.
  - Voice variants: `+m1..+m7`, `+f1..+f4`, `+croak`, `+whisper`; list them with `espeak --voices`. A commented hint for volume: `amixer set PCM 100`.
  - Code bug **[AMBIG]**: it uses `call` without `from subprocess import call`, and has an indentation error.
- **Recording/playback** (p.302–311):
  - pyaudio `paInt16`, 1 ch, 16000 Hz, `frames_per_buffer=2000`; run with `sudo python record.py` to get `output.wav` (5 s).
  - `sudo apt-get install sox`, then `play output.wav`. Other players: aplay, mpg123.
- Keras 2-word ASR (seven/stop) — `https://github.com/shaoyiwork/ASR-two-words`, about 1 GB RAM, 90 % accuracy. Extension idea: on recognition, speak the word and flash the red light for 1 s.
- Music: the built-in music list and play API are **[NOT IN COURSE]**. Blockly's "Show" category has "traffic sound" (probably sound effects; translation unclear **[AMBIG]**).

---

## 9. Vision

### 9.1 Built-in vision (ROS `/vision`)
- `vision_ai_event` with `face_analyse`, `gesture_analyse`, `face_tracking`, `stop_face_tracking`.
- `/vision_detect` counts faces and hands. `/vision_take_photo` takes photos.
- Blockly AI categories:
  - age: infants / children / adolescents / middle-aged / elderly
  - gender
  - 7 expressions: happiness, anger, surprise, calmness, sadness, disgust, fear
  - face count
  - face database enrolment with name, recognised within 20 s
- Gesture classes, object categories and colour list of the built-in service: **[NOT IN COURSE]**.

### 9.2 DIY vision taught
- Camera: Raspberry Pi CSI camera.
  - `raspistill -t 2000 -o image1.jpg`
  - `raspistill -t 2000 -o image2.jpg -w 640 -h 480`
  - picamera photo 1024×768 with a 2 s warm-up (the code comment is in Chinese "摄像头预热2秒" **[ZH]**)
  - video `camera.start_recording('my_video.h264')`, 60 s at 640×480; play it in VLC
  - OpenCV live view at 640×480, 32 fps
- **Possible conflict (my inference, not stated):** opening PiCamera directly will likely fail or steal the camera while the robot's `/vision` service holds it, and vice versa.
- Colour HSV ranges (OpenCV H 0–180):

  | Colour | Lower | Upper |
  |---|---|---|
  | red | [0,128,46] | [5,255,255] |
  | red (2nd range) | [156,128,46] | [180,255,255] |
  | green | [35,128,46] | [77,255,255] |
  | blue | [100,128,46] | [124,255,255] |
  | yellow | [15,128,46] | [34,255,255] |
  | black (defined, unused) | [0,0,0] | [180,255,10] |
  | white (defined, unused) | [0,0,70] | [180,30,255] |

  - Detection rule: total contour length > 400 → "Currrent color is <name>".
  - Traffic-light demo: on red, stop and squat; on green, stand, walk and say "Master, I've crossed the street."
- Face detection: Haar cascade, PiCamera 320×240 @40 fps, `detectMultiScale(gray, scaleFactor=2.0)`, FPS overlay. Training custom cascades is done on Windows with OpenCV 3.4.1 ("Because the Raspberry Pi Arm is underpowered, we need to train on the desktop").
- Face recognition: PCA (eigenface) and LDA (fisherface) on ORL att_faces (40×10, 112×92). Run with `python project3.py eigenface <dims> <train_per_person> <path>`.
- Cat/dog CNN: trained on a PC ("more than 8G graphics card, CPU i7"), testable on the robot. Code: `shaoyiwork/My-TensorFlow-tutorials` (folder 猫狗识别 = "cat-dog recognition" **[ZH]**).

---

## 10. Machine-learning deployment on the robot (p.287–301)
- "robots cannot train complex large models, or the training is too slow". The course trains on an Ubuntu 16.04 VM and deploys `.h5` files.
- Robot install (Python 2.7, armv7l):
  ```
  wget https://www.piwheels.hostedpi.com/simple/tensorflow/tensorflow-1.11.0-cp27-none-linux_armv7l.whl
  pip install tensorflow-1.11.0-cp27-none-linux_armv7l.whl
  sudo apt-get install libhdf5-serial-dev
  pip install h5py
  pip install pillow imutils
  pip install scipy --no-cache-dir
  pip install keras
  ```
  - Tip (verbatim): "the installation may fail due to network reasons. Don't worry, just execute it a few more times or restart the robot and install it several times."
  - Verify with `python -c "import tensorflow, keras"`.
- MNIST MLP: train on the PC, save `mnist_mode.h5`, then predict `mnist_test.png` on the Pi with `load_model`.
- Olivetti face LeNet: `https://github.com/shaoyiwork/Keras-simple-face`. On the robot, comment out `train_model` and `test_model` and just `load_weights('model_weights.h5')`.
- Other external repos (all author "shaoyiwork"):
  - Machine-Learning-Regression
  - Rnn_poetry
  - Caffe-fiveclasstest
  - Yanshee_Fzstart
  - Yanshee_Fzarm
  - Yanshee_Arm
  - Yanshee_Gait
  - ASR-two-words
  - Keras-simple-face
  - My-TensorFlow-tutorials
- Packages assumed present on the robot: OpenCV (cv2), picamera, numpy, sklearn, pandas, matplotlib. The course does not say which come preinstalled **[AMBIG]**.

---

## 11. Setup & connectivity — everything the course says (and does not)

| Topic | What the course says | Status |
|---|---|---|
| First boot | nothing | [NOT IN COURSE] |
| Wi-Fi config | robots and PC "same LAN (wifi hotspot)"; nothing on how to provision | [NOT IN COURSE] |
| Find robot IP | "You can use the mobile app to view the IP address of the robot first" (p.~47) | partial |
| Username | `pi` (prompts `pi@Yan_0A7E`, `pi@Yan_672C`), home `/home/pi` | from screenshots |
| Password | not stated | [NOT IN COURSE] |
| Hostname | `Yan_XXXX` in screenshots; text says default `raspberrypi` | [AMBIG] |
| Desktop access | "HDMI monitor or VNC"; HDMI cable + wireless keyboard/mouse listed as lesson hardware | mentioned, no setup |
| SSH | "VNC or other SSH tools"; "Xshell" listed as software | mentioned, no setup |
| JupyterLab :8888 | absent | [NOT IN COURSE] |
| REST :9090 / Swagger | only "RestfulAPI based on the http protocol" (Gazebo summary) | [NOT IN COURSE] |
| Blockly | on-robot desktop "pc-blockly"; also an app tile "Blockly" | covered (§2.3) |
| uCode | not mentioned | [NOT IN COURSE] |
| UDP ports | 20001 (servo cmd, localhost), 20005 (action broadcast), 20006 (unused variable) | covered |
| ROS master | on robot, `http://<robot-ip>:11311` | covered |
| OTA / upgrade | ROS `/upgrade_node`, `/upgrade_set`, `/hal_upgrade` exist; no procedure | names only |
| Factory reset | absent | [NOT IN COURSE] |
| Servo calibration/zeroing | services `/hal_servo_offset_*` (read/write offsets) exist; no procedure | names only |
| Battery / charging / runtime | `/hal_battery_read` exists; nothing else | [NOT IN COURSE] |
| Volume | `/hal_volume_set`, `/sysconfig_volume_get|set`; `amixer set PCM 100` hint | names only |
| Language | `/sysconfig_language_get|set` Chinese/English | names only |

---

## 12. All warnings, caveats, known issues and troubleshooting tips found

1. **Servo self-protection/overheat**: the servo stops responding and its LED flashes continuously. Restart the robot and let it cool (p.82). Caused by heavy load, forcing joints by hand, or sustained high current.
2. **Pinch hazard**: the get-up routine can fire while someone is picking the robot up (p.201).
3. **PRP readback**: "Please hold your robot to prevent falling" when servos are powered down (p.84 screenshot). Powering a limb back up makes it snap back to its original position.
4. **ROS multi-robot**: single master only, node names must be unique, defaults are identical on all robots. The course recommends **UDP broadcast** instead (p.69).
5. Gazebo crash: set `export SVGA_VGPU10=0`. On segfault: `killall gzserver; killall gzclient`.
6. PocketSphinx: the `LD_LIBRARY_PATH`/`PKG_CONFIG_PATH` exports are needed, otherwise the `libpocketsphinx.so.3` error appears. "Input overrun" is non-fatal.
7. TF install on the Pi may fail from network issues; retry or reboot.
8. Caffe: if loss is `-nan`, reduce `base_lr` tenfold.
9. Robot training is too slow; train on a PC.
10. Haar cascade training must be done on a PC, because the Pi is "underpowered".
11. Gait: no foot force sensors, so the gait is open-loop.
12. Code defects to watch when copying examples **[AMBIG]**:
    - `fall_manager.py` lacks `import time` and mixes py2/py3 prints.
    - The eSpeak example lacks `from subprocess import call`.
    - `ros_test.py` calls the undefined `set_head_angle`.
    - `motion_track.py` uses `camera.release()`.
    - The TTS srv argument is named `async` (a Python 3.7+ keyword).
    - The servo buf appends a stray `"}`.
    - The IR lesson description refers to the old RobotApi functions.
    - Gyro thresholds conflict with the screenshot values.
    - IR mm vs cm.
    - Pressure "N" vs kg.
    - get-up motion names differ between prose and code.
13. Pressure lesson hardware list says "One Touch Sensor" (copy error).

---

## 13. Exhibition-oriented takeaways (from course content plus clearly-marked inference)

- **Synchronized multi-robot shows**: use UDP broadcast to `255.255.255.255:20005` with `{"cmd":"action","type":"start","para":{"name":"<motion>","repeat":N}}`. `name:"stop"` stops all robots. The course recommends this over ROS.
- **Fall recovery**:
  - Built-in fall-manager services exist (`/hal_fallmanager_config`, `/sysconfig_fallmanager_*`).
  - The DIY loop is `get_sensors_gyro` plus `GetupFront`/`GetupRear`.
  - **Validate the euler-x thresholds on your robot** (§6.6) and add pick-up discrimination to avoid the pinch hazard.
- **Heat**: continuous motion or loading triggers servo self-protection, shown by a flashing servo LED. Build rest periods between routines (e.g. play a motion, then idle several minutes; my inference). Avoid letting visitors force the joints. Keep a restart procedure handy.
- **Status feedback**:
  - Chest LED via `hal_led_set`: type 1 = chest, 2 = head; mode 1 solid, 2 breathing, 6 fast flash; colour 1 red, 2 green, 3 blue.
  - The normal state is blue breathing.
  - Use red flashing as an alarm, like the Blockly obstacle demo.
- **Chest button as an operator control**: subscribe to ROS topic `hal_button_info` (1 = single press, 2 = double press). This can start/stop demos without a phone.
- **Interactive demos already designed in the course**:
  - Blockly face recognition + greeting
  - handshake via the touch sensor
  - IR "magic hands" follow/retreat/wave
  - traffic-light colour recognition
  - obstacle warning
  - temperature advice
  - sensor read-aloud
- **Voice**:
  - The built-in ASR/semantic stack appears to be Chinese cloud services (iFLYTEK-style "IAT", "Tuling") **[ZH; inferred from names]**.
  - For English, the course teaches offline **PocketSphinx** keyword spotting and **eSpeak** TTS.
  - YanAPI `start_voice_tts` was demonstrated with English sentences.
  - No wake word is documented.
- **Battery life, run-time limits, charging behaviour**: **[NOT IN COURSE]**. Obtain them from the user manual or measure them; `/hal_battery_read` can be polled for logging.
- **Network**: one shared hotspot/LAN for all robots and the control PC. Use the app to find the IP. For ROS from a PC, use a bridged VM, `/etc/hosts` entries and `ROS_MASTER_URI=http://<robot>:11311`.
