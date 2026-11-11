# REFERENCE — technical details of omx_calib_ws (start with ../README.md instead)

This workspace calibrates **everything the peg-sorting task depends on**, in a fixed order,
with a pass/fail check at every step. When all steps pass, the pick-and-place code only has
to read one file and one topic.

```
 camera pixels ──(intrinsics, factory)──> camera 3D ──(marker + depth plane, step 3)──> drawn axes
 drawn axes ──(touch calibration, step 6)──> arm frame link1 ──(arm kinematics, step 2)──> joint angles
```
Each arrow can be wrong, and an error in any one of them shows up as "the coordinates are off".
The wizard checks them one at a time, so you know which arrow is wrong.

| Package | What it does |
|---|---|
> The simple guide is ../README.md. This file keeps the technical details.

| `omx_calibration` | `wizard` (guided calibration), `preflight` (hardware checks), `bringup.launch.py` (everything in one launch) |
| `peg_detection` | camera node: pegs (top-face centre) + sockets in `link1`, frozen camera pose |
| ROBOTIS `open_manipulator` | the arm driver (cloned by the install script unless you already have it) |
| `realsense-ros` | camera driver (built from source by the install script) |

**Output:** `~/omx_calibration/calibration.yaml` (all results), `peg_detector_overrides.yaml`
(loaded automatically by the camera node), `camera_pose.yaml`, `report.md` (numbers for your
course report).

---

## 1. Before you start: what you need
- Laptop with **Ubuntu 24.04 + ROS 2 Jazzy**, at least 2 free USB ports, one of them **USB 3** (blue).
- Arm: U2D2 + **12 V supply (≥ 5 A)**, power hub switch ON.
- Camera: RealSense D435i, original USB-C cable, on a rigid mount.
- Marker 0 sheet (`markers/aruco_id0_80mm_A4.pdf`, printed at 100 %), **taped flat at 4 corners**.
- Steel ruler or caliper, set-square or phone level app, the 4 pegs.

## 2. Hardware: known problems (all hit in our first tests)

### 2.1 Checklist (the wizard's preflight checks these)
| Check | Why |
|---|---|
| Dynamixel Wizard **closed** | two programs on one serial port corrupt each other's packets |
| U2D2 port works: `ls /dev/ttyUSB0`, `sudo dmesg \| tail` shows no `error -71` | `-71` = bad USB port/cable: the laptop cannot even see the U2D2 (happened on one laptop) |
| latency timer **1 ms** | default 16 ms: "There is no status packet", arm ignores commands |
| `brltty` removed | it grabs USB-serial adapters, `/dev/ttyUSB0` disappears |
| user in `dialout` | permission to open the port |
| 12 V on, motor LEDs blink at power-up | USB does not power the motors |
| camera on **USB 3** | USB 2 = low frame rate / no depth |

### 2.2 Joint zero is off (wizard step 2)
At all-zero joints the upper arm must be vertical and the forearm and gripper horizontal.
If one joint is off by more than ~2°, every position the arm computes is wrong, and wrong
differently in every pose, so no camera calibration can remove it.
- **Large error (≈ 15–30°)**: horn mounted on the wrong spline tooth → remount the horn.
- **Small error (2–5°)**: Dynamixel Wizard (ROS stopped) → motor ID → Torque Enable = 0 →
  **Homing Offset** (address 20) = −(error in degrees) × 11.38 ticks. Re-run step 2 to check the sign.

### 2.3 Base joint wrap ±180° (wizard step 2 and 6 warn about this)
At the dock, motor 11 reads about 357°, right at its 0/360° point. ROBOTIS maps that to
joint 1 ≈ ±180°. If the pegs and sockets lie **behind** the arm (negative x in `link1`) and on
both sides, some targets need +170° and others −170° on the base joint. The motor cannot
cross ±180°, so it swings **the long way round, almost 360°**, through everything on the table.
**Fix, pick one:**
1. **Homing Offset on motor 11 = 2048** (180°) in Dynamixel Wizard (ROS stopped, torque off).
   The arm's +x then points at the work area and everything sits around 0° on the base joint.
2. Rotate the arm (or the board) by 180° on the table.

After either fix, redo wizard steps **2, 6, 7, 8**. The camera calibration absorbs the new
direction automatically.

---

## 3. Install (once per laptop, ~30–40 min)
```bash
# ROS 2 Jazzy first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
cd ~ && unzip omx_calib_ws.zip && cd ~/omx_calib_ws
bash scripts/install_all.sh
```
It installs the RealSense SDK, U2D2 udev rule (latency 1 ms), removes brltty (asks first),
clones the ROBOTIS stack and realsense-ros into `src/`, runs rosdep, builds with
`--symlink-install` and adds the `source` lines to `~/.bashrc`.
**Log out and back in** if it added you to `dialout`.

Already have a working ROBOTIS workspace? Source it **before** running the script; it then
won't clone a second copy. Source both in every terminal: ROBOTIS first, then this one.

Check in a new terminal:
```bash
ros2 run omx_calibration preflight        # every line PASS (arm + camera plugged in)
```

---

## 4. Start a session (every time)
1. **Power:** 12 V on, U2D2 and camera plugged in, Dynamixel Wizard and realsense-viewer closed.
2. **Terminal 1:**
   ```bash
   ros2 launch omx_calibration bringup.launch.py
   ```
   This starts the arm (moves to its home pose), then the camera + peg detector, then the image
   viewer. Wait for **`Camera pose FROZEN`** (marker must be visible: arm away from it).
3. **Terminal 2:**
   ```bash
   ros2 run omx_calibration wizard
   ```
4. **Warm-up:** the first time in a session, let the camera stream **10–15 min** before step 3
   (D435 depth drifts while it warms up).

Options: `bringup.launch.py init_position:=false` (arm does not move at start),
`port_name:=/dev/ttyUSB1`, `viewer:=false`, `pointcloud:=true`.

---

## 5. The wizard, step by step
Menu: type the step number, or **`a`** to run 1→9 in order. Every step saves its result
(PASS/FAIL + numbers) to `calibration.yaml`, and the menu shows the status. `p` parks the arm.

**Jog mode** (used in steps 2, 5, 6, 7, 8): the gripper tip moves in a straight line and
**stays pointing straight down**.
| Key | Action |
|---|---|
| `w` / `s` | +x / −x (link1) |
| `a` / `d` | +y / −y |
| `r` / `f` | up / down |
| `1` `2` `3` | step 1 / 5 / 20 mm |
| `o` / `p` | gripper open / close |
| **Enter** | record |
| `x` | cancel |

The status line shows the tip position, step size, gripper and base angle. It never goes
below `z_floor_m` (default −5 mm, `--ros-args -p z_floor_m:=0.01` to raise it).

### 5.1 Step 1: Preflight (2 min)
Automatic: all checks from 2.1, plus `/joint_states`, TF `link1 → end_effector_link`, and the
camera node's `/peg_detector/status`.
**Pass:** every line PASS. **Fail:** the line tells you the fix.

### 5.2 Step 2: Arm (10 min)
1. **Model check (automatic):** our kinematics vs the live TF. **Pass: < 1 mm.** This is the
   model the pick-and-place IK will use.
2. **Zero pose:** the arm moves to all-zero joints (clear the space in front of it). Check
   with a set-square / level app: upper arm vertical; forearm and gripper horizontal and in
   line. Answer y/n; if n, describe the error → fix per 2.2.
3. **Base joint:** jog over the middle of the peg area, then over each end of the board.
   **Pass:** the base angles don't jump between about +170° and −170°. **Fail:** fix 2.3
   first; nothing later will work across that boundary.

### 5.3 Step 3: Camera (10 min)
Arm away from the marker. Measure the **black square of marker 0** edge to edge with the
ruler (mm) and type it in. The camera node re-measures its pose with the new size.
| Check | Pass |
|---|---|
| pose jitter while freezing | < 1 mm |
| tilt correction from the depth plane | < 3° (else marker paper not flat / glare) |
| marker size seen by depth vs ruler | within 3 % (else depth scale → 6.1) |
| live marker vs frozen pose | < 3 mm (else camera or sheet loose) |
| the 4 sockets at the same height | spread < 2 mm |

### 5.4 Step 4: Measurements (10 min)
Type ruler values in mm: board thickness, socket depth, each peg's height and width. Say
whether the pegs stand on the foam or the board. The wizard compares them with the camera.
**Pass:** socket height = board thickness (< 2 mm); each peg top = its height (< 3 mm).
These values are also what the pick-and-place uses for grasp and insertion heights.

### 5.5 Step 5: Gripper (5 min)
Only `o`/`p` work here. Record: fully open; for each peg, closed until it **just grips**
(hold the peg between the fingers by hand); fully closed. Saved as `gripper.open_m`,
`gripper.grip_m.<colour>`, `gripper.closed_m`.

### 5.6 Step 6: Robot frame, camera → arm (20 min, the important one)
Choose the peg colour (green) and the number of points (**6**). The gripper closes. Then for each point:
1. (optional) park the arm so it doesn't hide the peg;
2. put the peg at a **new** spot and press Enter. The camera records the peg top centre (1 s average);
3. jog the closed fingertips, pointing down, until they **just touch the centre of the peg top**
   (don't push it over), press Enter. The arm lifts 5 cm automatically.

**Spread the 6 spots:** 2 near the board, 2 near the base, the far +y and −y ends. A tight
cluster gives a bad rotation estimate.

The wizard fits rotation + offset and shows a residual per point. **Pass: RMS < 4 mm, max < 7 mm.**
It offers to drop an obvious outlier (moved peg / off-centre touch). The result is applied
to the running camera node immediately and saved for the next launch. No YAML editing.

> Always use the same gripper orientation (pointing down, fingers closed) here and for the real
> grasps. The fit then absorbs the offset between `end_effector_link` and the real fingertips.

### 5.7 Step 7: Verification on NEW points (10 min)
Same flow, **3 spots not used in step 6**. Error = camera vs touched tip.
**Pass: xy < 5 mm, z < 5 mm.** These are your **accuracy numbers for the report**.

### 5.8 Step 8: Poses (5 min)
- **Park pose:** jog to where the arm doesn't hide the pegs, sockets or marker from the camera → Enter.
- **Travel height:** jog the tip to a safe height over the pegs (tallest peg + ≥ 3 cm) → Enter.

### 5.9 Step 9: Summary
Status of all steps, and writes `~/omx_calibration/report.md`.
**All PASS → calibration done.**

---

## 6. When a check fails

### 6.1 Depth is off (step 3 marker size / step 4 heights off by the same %)
Close the bringup. `realsense-viewer` → Stereo Module → **On-Chip Calibration** (point at a
flat textured surface, ~0.5–1 m; accept if the health number improves), then **Tare
Calibration** with a known distance (follow the Viewer). Close it, relaunch, redo step 3.

### 6.2 Verification (step 7) fails but step 6 passed
| Pattern | Cause → fix |
|---|---|
| errors grow away from the marker | camera tilt: marker not flat → re-tape, redo 3, 6, 7 |
| errors depend on where the arm reaches | joint zero (2.2) → fix, redo 2, 6, 7 |
| same offset everywhere | touch technique: tip not on the centre / gripper not pointing down → redo 6 |
| random, large | peg moved when touched, camera bumped (`live vs frozen` > 3 mm) → redo 3, 6, 7 |

### 6.3 Other
| Symptom | Fix |
|---|---|
| `NO camera pose` | marker covered: park the arm, `ros2 service call /peg_detector/recalibrate std_srvs/srv/Trigger` |
| `top face not found` for a peg | glare on the peg top → change lighting |
| jog: "not reachable with the gripper pointing down" | too close to / too far from the base, or too low: move elsewhere |
| jog: "base joint would wrap" | see 2.3 |
| `/joint_states` missing | arm bringup not running / port error → 2.1 |

---

## 7. When to recalibrate
| What changed | Redo steps |
|---|---|
| nothing (new session) | 1, then a quick 7 with 1–2 points to confirm |
| camera bumped / moved | 3, 6, 7 |
| marker sheet moved | 3, 6, 7 |
| arm moved on the table | 2, 6, 7, 8 |
| Homing Offset / horn changed | 2, 6, 7, 8 |
| gripper fingers changed | 5, 6, 7 |
| different pegs | 4, 5 |

---

## 8. Using the calibration in the pick-and-place code
1. Start `ros2 launch omx_calibration bringup.launch.py` (arm + camera, calibration loaded).
2. Read `~/omx_calibration/calibration.yaml`:
```yaml
measurements: {board_thickness_mm, socket_depth_mm, pegs: {green: {height_mm, width_mm}, ...}}
gripper:      {open_m, closed_m, grip_m: {green: ..., ...}}
poses:        {park_joints: [4 rad], travel_z_m}
robot_frame:  {yaw_deg, offset_m, rms_mm}          # already applied by the camera node
verification: {xy_mean_mm, xy_max_mm}
```
3. Subscribe to `/peg_detector/scene` (`std_msgs/String`, JSON, metres, frame `link1`). Use it
   only when `"calibration": "frozen"`. Each peg gives `x, y, z_top`; each socket gives `x, y, z`.
4. IK: `from omx_calibration import kinematics as K` → `q, ok = K.ik((x, y, z), K.DOWN, q0=current_q)`
   (verified against the ROBOTIS URDF and checked against the live TF in step 2). Send `q` to
   `/arm_controller/joint_trajectory`; the gripper uses the `/gripper_controller/gripper_cmd`
   action (`control_msgs/GripperCommand`, position in metres).
5. Typical sequence per peg: park → above peg at `travel_z` → down to `z_top − ~15 mm` →
   close to `grip_m` → up → above socket → down to `z + peg height + ~5 mm` → open → up.

---

## 9. Files and topics
| Path | Content |
|---|---|
| `~/omx_calibration/calibration.yaml` | all results (wizard) |
| `~/omx_calibration/peg_detector_overrides.yaml` | marker size + robot frame for the camera node (auto-loaded) |
| `~/omx_calibration/camera_pose.yaml` | last frozen camera pose (`marker.load_saved: true` to start with it) |
| `~/omx_calibration/report.md` | summary for the report |

| Topic / service | |
|---|---|
| `/peg_detector/scene` | pegs + sockets, JSON, link1 |
| `/peg_detector/status` | calibration diagnostics, JSON (the wizard reads this) |
| `/peg_detector/annotated_image` | white dot = peg top centre, × = socket centre |
| `/peg_detector/recalibrate` | `std_srvs/Trigger`: re-measure the camera pose |
| `/arm_controller/joint_trajectory`, `/gripper_controller/gripper_cmd` | ROBOTIS arm / gripper |

Set a different calibration folder with `export OMX_CALIB_DIR=...` (wizard) and
`calibration_file:=...` (launch).
