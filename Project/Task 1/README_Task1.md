# OpenMANIPULATOR-X Peg Sorting — Installation and Operation Guide

Vision-guided peg sorting with a ROBOTIS OpenMANIPULATOR-X and an Intel RealSense D435i.
The camera finds four coloured pegs and their sockets, converts the positions into the
robot's base frame, and the arm picks each peg and places it into its matching socket.

| Peg colour | Peg shape | Socket |
|---|---|---|
| Green | Cylinder, Ø 40 mm | Circle |
| Blue | Square prism, side 40 mm | Square |
| Orange | Hexagonal prism, side 20 mm | Hexagon |
| Yellow | Triangular prism, side 30 mm | Triangle |

All pegs are 100 mm tall.

---

## Contents

1. [What you need](#1-what-you-need)
2. [The three archives](#2-the-three-archives)
3. [Install ROS 2 Jazzy](#3-install-ros-2-jazzy)
4. [Unpack the workspace](#4-unpack-the-workspace)
5. [Install the dependencies](#5-install-the-dependencies)
6. [Configure the USB serial adapter (U2D2)](#6-configure-the-usb-serial-adapter-u2d2)
7. [Check the ROBOTIS configuration](#7-check-the-robotis-configuration)
8. [Build](#8-build)
9. [Hardware setup](#9-hardware-setup)
10. [First start and checks](#10-first-start-and-checks)
11. [Camera calibration](#11-camera-calibration)
12. [Running the peg sorting](#12-running-the-peg-sorting)
13. [Tuning](#13-tuning)
14. [Troubleshooting](#14-troubleshooting)
15. [Reference](#15-reference)

---

## 1. What you need

**Computer**
- Ubuntu **24.04** (64-bit), native install recommended (USB passthrough in virtual machines is unreliable)
- A **USB 3** port for the camera and any USB port for the U2D2
- Internet access during installation, about 10 GB of free disk space

**Hardware**
- ROBOTIS OpenMANIPULATOR-X with Dynamixel XM430-W350 motors, IDs **11–15**
  (11 base, 12 shoulder, 13 elbow, 14 wrist, 15 gripper), 1 Mbps, Protocol 2.0
- ROBOTIS U2D2 + U2D2 Power Hub with a **12 V** supply
- Intel RealSense **D435i** with a USB 3 cable
- Peg board with the four sockets, the four pegs, and a printed ArUco marker
  (**DICT_4X4_50, ID 4**, black square **37.4 mm**)

---

## 2. The three archives

| Archive | Contents |
|---|---|
| `omx_calib_ws.zip` | The project workspace: peg detection, calibration tools, pick-and-place, launch files |
| `open_manipulator.zip` | ROBOTIS OpenMANIPULATOR packages (arm description, ros2_control bringup), with the project's modifications |
| `open_manipulator_msgs.zip` | ROBOTIS message definitions used by the packages above |

Everything is built in **one** workspace, `~/omx_calib_ws`.

---

## 3. Install ROS 2 Jazzy

Skip this section if `ls /opt/ros/jazzy/setup.bash` already prints a path.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

If any of these commands fails, follow the official guide instead:
<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html>

Check:
```bash
source /opt/ros/jazzy/setup.bash && ros2 --help | head -3
```

---

## 4. Unpack the workspace

Put the three archives in `~/Downloads`, then:

```bash
cd ~
unzip -q ~/Downloads/omx_calib_ws.zip            # creates ~/omx_calib_ws
mkdir -p ~/omx_calib_ws/src/robotis
cd ~/omx_calib_ws/src/robotis
unzip -q ~/Downloads/open_manipulator.zip
unzip -q ~/Downloads/open_manipulator_msgs.zip
```

Remove build output that may have come from another computer:
```bash
cd ~/omx_calib_ws
rm -rf build install log
```

If the archive contains a source copy of the RealSense ROS wrapper (`src/third_party/realsense-ros`),
disable it. The packaged version from Section 5 is used instead:
```bash
[ -d src/third_party/realsense-ros ] && touch src/third_party/realsense-ros/COLCON_IGNORE
```

Only the OpenMANIPULATOR-**X** packages are needed. If these folders exist, disable them, because
they need extra dependencies such as MoveIt:
```bash
for d in open_manipulator_moveit_config open_manipulator_gui open_manipulator_playground \
         open_manipulator_collision ros2_controller; do
  f=$(find src/robotis -maxdepth 3 -type d -name "$d" | head -1)
  [ -n "$f" ] && touch "$f/COLCON_IGNORE" && echo "ignored $f"
done
```

Check the layout:
```bash
ls ~/omx_calib_ws/src
# expected (at least): omx_calibration  peg_detection  robotis
find ~/omx_calib_ws/src -name package.xml -path "*open_manipulator_bringup*"
# expected: one path ending in open_manipulator_bringup/package.xml
```

---

## 5. Install the dependencies

### 5.1 ROS packages
```bash
sudo apt install -y python3-pip python3-numpy python3-yaml python3-rosdep \
  python3-colcon-common-extensions \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-control-msgs \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rqt-image-view \
  ros-jazzy-tf2-ros ros-jazzy-message-filters ros-jazzy-std-srvs \
  ros-jazzy-dynamixel-sdk ros-jazzy-dynamixel-hardware-interface \
  ros-jazzy-realsense2-camera
```

Then let `rosdep` install anything else the packages declare:
```bash
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
cd ~/omx_calib_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

### 5.2 RealSense access rules
The ROS RealSense package does not install the USB permission rules. Install them once:
```bash
sudo curl -sSL -o /etc/udev/rules.d/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 5.3 Python packages
The detector needs OpenCV ≥ 4.7 (ArUco API). Install the tested version **without** dependencies,
so that pip does not replace the system NumPy that ROS is built against:
```bash
python3 -m pip install --user --break-system-packages --no-deps opencv-python-headless==4.10.0.84
python3 -c "import cv2, numpy; print('OpenCV', cv2.__version__, 'NumPy', numpy.__version__); assert hasattr(cv2.aruco, 'ArucoDetector')"
```
Expected: `OpenCV 4.10.0 NumPy 1.26.x`. If NumPy shows 2.x, remove the pip copy with
`python3 -m pip uninstall --break-system-packages numpy`.

---

## 6. Configure the USB serial adapter (U2D2)

```bash
# 1 ms USB latency and open permissions for the U2D2 (FTDI)
echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' | \
  sudo tee /etc/udev/rules.d/99-omx-u2d2.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# brltty (Braille display service) takes over FTDI adapters: remove it
sudo apt purge -y brltty

# serial port permission for your user
sudo usermod -aG dialout $USER
```

**Log out and log back in** (or reboot) so that the group change takes effect.

Check, with the U2D2 plugged in:
```bash
ls /dev/ttyUSB0                                    # must exist
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # must print 1
groups | grep -o dialout                           # must print dialout
```

---

## 7. Check the ROBOTIS configuration

The board is behind the robot, at a base angle of about 180°. A standard Dynamixel base cannot
pass ±180° without turning almost a full revolution the other way, so this project runs the
**base motor (ID 11) in multi-turn mode**, and the motion code always takes the short way round.
`open_manipulator.zip` should already contain these changes. Verify them:

```bash
cd ~/omx_calib_ws/src/robotis
X=$(find . -name open_manipulator_x_position.ros2_control.xacro | head -1)
U=$(find . -name open_manipulator_x_arm.urdf.xacro | head -1)
echo "$X"; echo "$U"
grep -A7 '"ID">11<' "$X" | grep "Operating Mode"      # expected: ...Operating Mode">4</param>
grep -n '6\*pi' "$U"                                  # expected: joint1 limit lower="${-6*pi}" upper="${6*pi}"
grep -c "signed,-0.036099321" "$X"                    # expected: 2 (original gripper offset)
```

If a check fails, apply the change:
```bash
python3 - "$X" "$U" <<'PYEOF'
import re, sys
x, u = sys.argv[1], sys.argv[2]
s = open(x).read()
s = re.sub(r'(<param name="ID">11</param>.*?<param name="Operating Mode">)3(</param>)', r'\g<1>4\2', s, count=1, flags=re.S)
s = s.replace('signed,0.036481;', 'signed,-0.036099321;')
open(x, 'w').write(s)
s = open(u).read()
s = s.replace('lower="${-pi}" upper="${pi}"', 'lower="${-6*pi}" upper="${6*pi}"', 1)
open(u, 'w').write(s)
print('ROBOTIS configuration updated')
PYEOF
```

> Do not use the ROBOTIS start-up "home" move with this configuration. The project's launch file
> disables it (`init_position:=false`), and homing is done with `go_home` (Section 10).

---

## 8. Build

```bash
cd ~/omx_calib_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The build must end with `Summary: N packages finished` and no `failed` packages.

Add both setup files to every new terminal:
```bash
grep -qxF "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -qxF "source ~/omx_calib_ws/install/setup.bash" ~/.bashrc || echo "source ~/omx_calib_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check that the commands are available:
```bash
ros2 pkg list | grep -E "omx_calibration|peg_detection|open_manipulator_bringup"
```

> **Rebuilding:** after editing any file in `src/`, run
> `cd ~/omx_calib_ws && colcon build --symlink-install && source install/setup.bash`.

---

## 9. Hardware setup

### 9.1 Layout
- The robot base is fixed next to the board. The board, the sockets and the marker are **behind**
  the robot, at a base angle of about 180°.
- The ArUco marker lies flat, with its **centre 38 mm from the base rotation axis**, on the line
  that runs from the robot toward the board.
- Board axes as used by the software: **+x** points from the board toward the robot,
  **+z** points up, **+y** completes a right-handed frame. All coordinates the software prints are
  in **cm or m from the base rotation axis**.
- The camera looks down at the board from a fixed mount. **The whole board, the sockets, the marker
  and every peg start position must be in view.**
- Pegs stand upright on the mat, within about 25 cm of the base. Nothing else of the four peg
  colours (pens, stickers, tape) may be in the camera view.

### 9.2 Connections
1. U2D2 → USB port. Power Hub → **12 V on**.
2. RealSense → **USB 3** port (blue port, or a USB 3 hub).

### 9.3 Dynamixel settings (one-time)
These are stored in the motors. Check them once with ROBOTIS Dynamixel Wizard 2.0 (torque off):

| Item | IDs 11–14 | ID 15 (gripper) |
|---|---|---|
| Baud rate | 1 Mbps | 1 Mbps |
| Current Limit | 1193 | 1193 (must be ≥ 200) |
| PWM Limit | 885 | 885 |
| Homing Offset | 0 | 0 |
| Max / Min Position Limit | 4095 / 0 | 4095 / 0 |

Close Dynamixel Wizard before starting ROS. Only one program can use the serial port.

---

## 10. First start and checks

### 10.1 Start-up routine (every session)
1. **Switch the 12 V supply off, wait 5 s, switch it on.** This resets the motors' position
   counters. Always do this before launching.
2. Terminal 1, the arm, the camera, the detector and the image viewer:
   ```bash
   ros2 launch omx_calibration bringup.launch.py
   ```
   The arm switches its torque on and holds its position. It does **not** move at start-up.
3. Terminal 2, move to the home pose (short way round, gripper opens first):
   ```bash
   ros2 run omx_calibration go_home
   ```

Launch options: `vision:=false` (arm only), `arm:=false` (camera only), `viewer:=false`,
`port_name:=/dev/ttyUSB1`.

### 10.2 What a correct start looks like
- Terminal 1 shows the five motors found and `Configured and activated arm_controller`,
  `gripper_controller` and `joint_state_broadcaster`.
- The image viewer (`/peg_detector/annotated_image`) shows the pegs and sockets outlined and labelled.
- After about 2 s the bottom line of the image says **`camera pose FROZEN`** and the marker shows
  red/green/blue axes.
- `go_home` prints `reached ... (max error < 1 deg)`.

### 10.3 Gripper direction check
```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.019, max_effort: 0.0}}"
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -0.010, max_effort: 0.0}}"
```
**+0.019 must open** the gripper and **−0.010 must close** it. The result line should show a
position close to the command.

### 10.4 Coordinates check
```bash
ros2 topic echo /peg_detector/scene --once
```
Each peg reports `x`, `y`, `z_top` (m, from the base rotation axis), and each socket reports `x`, `y`, `z`.
All four pegs should report a similar `z_top` (≈ 0.10) wherever they stand.

---

## 11. Camera calibration

The detector converts camera coordinates into the robot frame through the marker. Because the
marker is small, a final correction is measured against points whose position you know.
**Do this once per setup, and again whenever the camera, the marker or the robot base moves.**

1. Arm at home (`go_home`) and `camera pose FROZEN` visible.
2. Start the fitting tool:
   ```bash
   ros2 run omx_calibration fit_camera
   ```
3. Clear any previous correction:
   ```
   fit_camera> reset
   ```
4. Place a peg upright at a spot and **measure** its centre with a ruler: x and y from the base
   rotation axis (board axes, Section 9.1), and z = peg top height (10 cm). Enter colour and values in **cm**:
   ```
   fit_camera> green -14 -6.3 10
   ```
   The tool averages 2 s of detections and prints the current error for that point.
5. Repeat for **4–6 spots spread over the work area**: near and far, left and right, close to the sockets.
6. Compute and inspect:
   ```
   fit_camera> fit
   ```
   The **AFTER** table should show every point within about **0.5 cm**. If one point is much worse
   than the others, it was probably mismeasured: remove it with `undo` (last point) and measure again.
7. Apply and save:
   ```
   fit_camera> apply
   fit_camera> quit
   ```
   The correction is active immediately and saved in
   `src/peg_detection/config/detection_params.yaml`. The camera pose is saved as well and reused at
   the next start (`marker.load_saved: true`).

Other commands: `list` (show points), `undo` (remove last point), `reset` (remove the correction).

> After a camera move: `ros2 service call /peg_detector/recalibrate std_srvs/srv/Trigger`,
> then repeat this section.

---

## 12. Running the peg sorting

Preconditions: start-up routine done (Section 10.1), calibration done (Section 11), arm at home,
pegs upright in the camera view, nothing else in the arm's path.

### 12.1 Check without moving
```bash
ros2 run omx_calibration pick_place --colour blue --check
```
This prints the detected peg and socket, the measured angles, and the predicted misalignment
at the socket.

### 12.2 Dry run
The gripper opens, then the arm hovers 3 cm above the peg and over the socket. It never grips.
```bash
ros2 run omx_calibration pick_place --colour green --dry-run
```

### 12.3 Pick and place
One peg (the program waits for **Enter** before each move):
```bash
ros2 run omx_calibration pick_place --colour green     # cylinder -> circle
ros2 run omx_calibration pick_place --colour blue      # square   -> square
ros2 run omx_calibration pick_place --colour orange    # hexagon  -> hexagon
ros2 run omx_calibration pick_place --colour yellow    # triangle -> triangle
```
All four in sequence:
```bash
ros2 run omx_calibration pick_place --colour all
```

Sequence for each peg: open gripper → above the peg → down to grip → close → lift → over the
socket → down → release → up → home.

**Stopping:** type `q` at a prompt or press **Ctrl+C** at any time. The arm stops and holds its
position. Then clear the area and run `ros2 run omx_calibration go_home`.

### 12.4 Options
| Option | Meaning |
|---|---|
| `--colour all` | green, blue, orange, yellow in sequence |
| `--check` | print the analysis only, no motion |
| `--dry-run` | hover only, never grip |
| `--no-step` | run without waiting for Enter |
| `--force` | insert even if the predicted misalignment exceeds `--max-misalign` |
| `--grip-depth M` | grip point below the peg top, m (default −0.04) |
| `--tool-length M` | wrist axis to grip point, m (default 0.085) |
| `--max-misalign DEG` | refuse to insert above this predicted angle error (default 12) |
| `--gripper-gap M` | measured inside width of the open gripper; enables the width check |

### 12.5 Why square, hexagon and triangle are handled differently
The arm has no wrist rotation: the gripper's yaw is set by the base angle. When the gripper
closes on a square or hexagonal peg, it **squeezes the peg straight**, whatever its starting angle.
The angle at which the peg then arrives over its socket is fixed by where the socket lies, so the
software predicts the remaining misalignment before moving and reports it.
A triangle can settle in one of two ways. If the program reports that it will settle the wrong way,
turn the triangle by about 60° by hand and run again.

---

## 13. Tuning

The following values were tuned on the reference setup. Re-check them on a new setup with a dry run.

| Value | Where | Default |
|---|---|---|
| Grip depth | `--grip-depth` / `src/omx_calibration/omx_calibration/pick_place.py` | −0.04 m |
| Tool length | `--tool-length` / `pick_place.py` | 0.085 m |
| Place offsets per peg | `SOCKET_OFFSET` in `pick_place.py` | see below |
| Home pose | `PARK_DEFAULT` in `src/omx_calibration/omx_calibration/wizard.py` | `[3.157, -0.960, -0.109, 1.094]` rad |
| Peg sizes | `peg_size_m.*` in the detector parameters | 0.040 / 0.020 / 0.030 m |

**Grip depth:** a larger value (e.g. −0.04 → −0.03) grips lower on the peg, a smaller value grips
higher. Change the default permanently:
```bash
cd ~/omx_calib_ws
sed -i -E "s/a.add_argument\('--grip-depth', type=float, default=[-0-9.]+,/a.add_argument('--grip-depth', type=float, default=-0.03,/" src/omx_calibration/omx_calibration/pick_place.py
colcon build --symlink-install --packages-select omx_calibration && source install/setup.bash
```

**Place offsets** shift the release point of each peg relative to the detected socket centre
(metres, robot x and y):
```python
SOCKET_OFFSET = {'green': (0.0, -0.005), 'orange': (0.0, -0.005),
                 'yellow': (-0.003, 0.005), 'blue': (-0.00425, -0.007)}
```
If a peg lands consistently beside its hole, edit its entry and rebuild.

---

## 14. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `[ID:011] ping ... no status packet`, or the scan finds no motors | 12 V supply off; U2D2 not plugged in; Dynamixel Wizard still open; latency or permissions (Section 6) |
| `/dev/ttyUSB0` missing; `dmesg` shows `error -71` | Bad USB cable or port. Try another port and cable. Check that `brltty` is removed. |
| `symbol lookup error ... fastcdr` when launching | ROS packages at mixed versions: `sudo apt update && sudo apt full-upgrade -y`, then `rm -rf build install log` and rebuild |
| `Goal Current ... exceeds the limit value` (ID 15) | Gripper Current Limit too low. Set it to 1193 in Dynamixel Wizard (torque off). |
| Gripper moves the wrong way or reads far outside −0.010…+0.020 | The motor's position count is shifted. Power-cycle the 12 V, launch again, and repeat check 10.3. Do not run other programs that write to the motors while ROS is running. |
| `No RealSense devices were found` | Replug the camera into a USB 3 port; check the rules from Section 5.2; `rs-enumerate-devices` must list the camera |
| `MARKER: not visible` | Wrong ID or dictionary (must be DICT_4X4_50, ID 4), marker covered, or poorly lit. The arm should be at home during start-up. |
| Peg heights differ near and far | Table tilt not corrected. Check that `ros2 param get /peg_detector marker.refine_mode` returns `table`, then redo Section 11. |
| A peg is reported at an impossible position, or `is not a peg on the table` | Another object of that colour is in view. Remove it. |
| `NOT REACHABLE with any gripper pitch` | The target is outside the arm's reach with the current grip settings. Move the peg closer to the base, or reduce `--lift-clear`. |
| `joint error > 8 deg after settling - stopping` | The arm was blocked (collision with a peg or the board). Clear the path, run `go_home`, check the dry run. |
| Peg not lifted / dragged | Grip point too high on the peg. Increase `--grip-depth` slightly (e.g. −0.04 → −0.035). |
| Peg stops on the rim of the socket | Adjust that peg's `SOCKET_OFFSET` (Section 13), or check the predicted misalignment with `--check`. |
| `go_home` fails | Gripper stuck on a peg, or the arm is blocked. `go_home` opens the gripper and lifts first; clear any obstacle and run it again. |

---

## 15. Reference

### Commands
| Command | Purpose |
|---|---|
| `ros2 launch omx_calibration bringup.launch.py` | Arm, camera, detector, viewer |
| `ros2 run omx_calibration go_home` | Open gripper, lift, move to home |
| `ros2 run omx_calibration fit_camera` | Camera-to-robot correction from measured points |
| `ros2 run omx_calibration pick_place [...]` | Peg sorting (Section 12) |
| `ros2 run omx_calibration preflight` | System check (ROS, serial port, camera) |
| `ros2 run rqt_image_view rqt_image_view /peg_detector/annotated_image` | Detection image |
| `ros2 service call /peg_detector/recalibrate std_srvs/srv/Trigger` | Re-measure the camera pose |

### Topics
| Topic | Content |
|---|---|
| `/peg_detector/scene` | JSON: pegs (`x`, `y`, `z_top`, `yaw_deg`, `target_socket`) and sockets (`x`, `y`, `z`, `yaw_deg`), metres, robot base frame |
| `/peg_detector/annotated_image` | Camera image with detections |
| `/joint_states` | Joint angles (`joint1`–`joint4`, `gripper_left_joint`) |
| `/arm_controller/joint_trajectory` | Arm commands |
| `/gripper_controller/gripper_cmd` | Gripper action (`+0.019` open, `−0.010` closed) |

### Files
| Path | Content |
|---|---|
| `src/peg_detection/config/detection_params.yaml` | Detector, marker and frame parameters, fitted correction |
| `src/omx_calibration/omx_calibration/pick_place.py` | Pick-and-place logic and tuned values |
| `src/omx_calibration/omx_calibration/kinematics.py` | Forward/inverse kinematics (damped least squares) |
| `src/omx_calibration/omx_calibration/wizard.py` | Robot interface, home pose, gripper values |
| `~/omx_calibration/camera_pose.yaml` | Saved camera pose (created at the first freeze) |

### Coordinate frame used by the software
Origin on the **base rotation axis** at the height of the board surface, **+x** from the board toward
the robot, **+y** to the left when looking along +x, **+z** up. The base joint angle is about 180°
when the arm points at the board.
