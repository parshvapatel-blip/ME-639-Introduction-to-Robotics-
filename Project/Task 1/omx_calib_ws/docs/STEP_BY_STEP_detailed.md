# Step-by-step: exact commands, what to type, what you should see

Everything below happens on **one laptop** (Ubuntu 24.04, ROS 2 Jazzy), with the workspace
at `~/omx_calib_ws`.

**Where things are typed.** You use 3 terminal windows (open each with **Ctrl+Alt+T**):

| Name | What runs there | Do you type in it? |
|---|---|---|
| **T1** | the arm + camera + vision (`bringup.launch.py`), shows log messages | no, only Ctrl+C to stop |
| **T2** | the calibration **wizard**: every answer, every measurement, all jog keys | **yes, everything** |
| **T3** | spare: checks (`cat`, `ros2 topic echo ...`) | sometimes |

Plus one image window (rqt_image_view) that opens by itself. **Click on T2 before you type
answers or press jog keys.** Keys typed into the image window or T1 do nothing.

**How data gets recorded.** You never write files yourself. When the wizard asks for a ruler
value, you type the number in T2 and press Enter. When it asks you to position the arm, you
jog with keys in T2 and press **Enter** to record. Each finished step is saved immediately to
`~/omx_calibration/calibration.yaml`.

---

## PART A — Install (once, ~40 min)

### A1. Unzip
T3:
```bash
cd ~
unzip omx_calib_ws.zip
cd ~/omx_calib_ws
ls
```
Expected: `README.md  STEP_BY_STEP.md  markers  omx_robotis.repos  requirements.txt  scripts  src`

### A2. Run the installer
Same terminal (stay in `~/omx_calib_ws`):
```bash
bash scripts/install_all.sh
```
- It asks for your **sudo password** (type it, nothing shows while typing, press Enter).
- If asked `brltty is installed ... Remove it? [Y/n]`, press **Enter** (= yes).
- It takes 30–40 min. The last lines should be:
```
DONE. Open a NEW terminal (log out/in if you were added to dialout), then:
  ros2 run omx_calibration preflight
  ...
```
If it stops with `ERROR` or a red `Failed`, copy the last 30 lines and send them to me.

If it printed `Added to dialout: LOG OUT and back in.`, **log out of Ubuntu and log in again** now.

### A3. Check the install
**Open a new terminal** (the old one doesn't know about the new packages):
```bash
ros2 pkg list | grep -E "omx_calibration|peg_detection|open_manipulator_bringup|realsense2_camera"
```
Expected (4 lines):
```
omx_calibration
open_manipulator_bringup
peg_detection
realsense2_camera
```

---

## PART B — Start a session (every time, ~5 min + warm-up)

### B1. Hardware
1. 12 V power ON (motor LEDs blink once).
2. U2D2 USB into the laptop, camera into a **blue USB 3** port.
3. **Close Dynamixel Wizard and realsense-viewer** if they are open.
4. Marker sheet taped flat at 4 corners. Arm not covering the marker.

### B2. Preflight (T3)
```bash
ros2 run omx_calibration preflight
```
Expected:
```
Preflight checks
  [PASS] ROS 2 distro is Jazzy               ROS_DISTRO=jazzy
  [PASS] U2D2 serial port present             /dev/ttyUSB0
  [PASS] /dev/ttyUSB0 latency timer = 1 ms    latency_timer=1
  [PASS] brltty not installed                 not installed
  [PASS] user in dialout group                yes
  [PASS] Dynamixel Wizard closed              not running
  [PASS] RealSense D435i connected            found
  [PASS] RealSense on USB 3                   USB 3.2

All good.
```
A `[FAIL]` line is followed by `fix: ...`. Do that fix, then run the command again.

### B3. Start arm + camera + vision (T1)
```bash
ros2 launch omx_calibration bringup.launch.py
```
What happens, in order:
1. The **arm moves** to the ROBOTIS home pose (keep hands clear). If you don't want it to
   move at start: `ros2 launch omx_calibration bringup.launch.py init_position:=false`
2. After ~3 s the camera starts; T1 shows
   `[detection.launch] no calibration overrides ... (not calibrated yet)` (first time) or
   `using calibration overrides: /home/<you>/omx_calibration/peg_detector_overrides.yaml` (later).
3. `Intrinsics from camera_info (camera_color_optical_frame, 640x480): fx=... fy=...`
4. After a few seconds: **`Camera pose FROZEN from 30 frames (translation jitter 0.4 mm)`**
5. The image window opens. Bottom line in green: `camera pose FROZEN | frame link1 | ...`.
   White dots on the peg tops, white × on the sockets, magenta box + axes on the marker.

T1 also prints a `PEG DETECTOR` report every second; you can ignore it, because the wizard
reads the same numbers.

**Problems in T1:**
- **Bottom line in red, `NO camera pose: marker not visible`**: the arm or your hand covers marker 0. Move it away. It freezes about 3 s after the marker is visible.
- **`process has died` for `ros2_control_node`, or `no status packet`**: arm connection problem. Ctrl+C, check B1, run B2 again.
- **The image window is grey**: wait 10 s. Still grey: Ctrl+C in T1 and start again.

### B4. Warm-up
**First session of the day: leave everything running 10–15 min** before step 3 of the wizard.
Steps 1, 2 and 5 can be done during the warm-up.

### B5. Start the wizard (T2)
```bash
ros2 run omx_calibration wizard
```
Expected:
```
----------------------------------------------------------------------
 OMX CALIBRATION WIZARD     (file: /home/<you>/omx_calibration/calibration.yaml)
----------------------------------------------------------------------
  1. preflight     not done
  2. arm           not done
  3. camera        not done
  4. measurements  not done
  5. gripper       not done
  6. robot_frame   not done
  7. verification  not done
  8. poses         not done
  9. summary
  a. run steps 1-9 in order   p. park the arm   q. quit
choice:
```
Type a number and press **Enter** to run that step. After a step finishes, the menu
comes back with `PASS` or `FAIL` and the date next to it.

**How to answer prompts.** A prompt looks like `Move to zero now? (y/n) [y]:`.
- `[y]` is the default: pressing just **Enter** answers `y`.
- Type `n` + Enter for no.
- For numbers (`board thickness ... :`), type e.g. `6.8` + Enter. Units are always **mm** unless the prompt says otherwise.

**Jog mode** (steps 2, 5, 6, 7, 8): when a line starts with `>>`, the wizard is waiting for
keys. **Single keys, no Enter**, except Enter to record:

| Key | Moves the gripper tip |
|---|---|
| `w` / `s` | forward / back along x |
| `a` / `d` | left / right along y |
| `r` / `f` | up / down |
| `1` `2` `3` | step size 1 mm / 5 mm / 20 mm (start with `3` for big moves, `1` at the end) |
| `o` / `p` | open / close the gripper |
| **Enter** | **record this position** |
| `x` | cancel |

While you jog, one line keeps updating:
```
  tip x  -11.0  y   +6.0  z   +8.5 cm | step 5 mm | gripper -10.0 mm | base +154 deg
```
The gripper always stays pointing straight down while jogging. If the wizard asks
`Rotate the wrist to point down at the current position now? (y/n) [y]:`, press Enter. The
wrist turns, then jogging starts.

Directions feel wrong (`w` moves the arm "backwards")? That's normal: x/y are the arm's own
axes. Watch the arm and correct with the opposite key.

---

## PART C — The calibration, step by step (all in T2)

Total ~1.5 h the first time. Do the steps in order: type `1` + Enter, then `2` + Enter, and so on.
Or type `a` to run them one after another; the wizard asks `Continue with the next step?` in between.

### Step 1 — Preflight (T2: `1` + Enter)
Nothing to do: it runs the B2 checks plus three ROS checks.
Expected at the end:
```
  [PASS] /joint_states from the arm (ROBOTIS bringup running)
  [PASS] TF link1 -> end_effector_link (robot_state_publisher)
  [PASS] /peg_detector/status (vision running)

  All good.
```
**Saved:** `preflight: passed: true`

### Step 2 — Arm (T2: `2` + Enter, ~10 min)
1. Automatic: `[PASS] kinematics model matches TF at this pose (0.00 mm)`
2. `Move to zero now? (y/n) [y]:` → clear the space **above and in front** of the arm, press **Enter**.
   The arm stands up straight and points forward.
   `[PASS] arm reached the zero command (max joint error 0.4 deg)`
3. **Look at the arm** with a set-square or a phone level app on each link:
   - upper arm (motor 12 → 13) vertical
   - forearm (13 → 14) and gripper horizontal, in one straight line

   `Is that what you see (each within ~2 deg)? (y/n) [y]:` → Enter if yes. If not, type `n`, then
   describe it when asked (e.g. `joint 3 forearm tilted down about 4 deg`). The note is saved,
   and README 2.2 explains the fix.
4. `Move back to where it was? (y/n) [y]:` → Enter.
5. **Base joint check, 3 jogs:**
   `>> move over the middle of the peg area, Enter` → jog with `3` + `w/s/a/d` until the gripper
   is above the middle of the pegs (height doesn't matter, stay high with `r`), press **Enter**.
   It prints `base joint = +165 deg`. Repeat for `the +y end of the board` and `the -y end of the board`.

   Expected: `[PASS] workspace does not cross the base joint +-180 deg limit (range +140 .. +176 deg)`
   If it says **FAIL**: stop here and do README 2.3 (Homing Offset on motor 11), then run step 2 again.

**Saved:** `arm: fk_vs_tf_mm, zero_pose_visual_ok, base_angles_deg, base_wrap_ok`

### Step 3 — Camera (T2: `3` + Enter, ~10 min, after the warm-up)
1. Park the arm away from the marker (menu `p`, or jog it away).
2. **Measure** marker 0's **black square** edge to edge with a steel ruler, in mm (e.g. `37.0`).
3. ```
   length_m in use: 37.4 mm
   RULER: side of the black square of marker 0 in mm (Enter = keep):
   ```
   Type your value, e.g. `37.0` + Enter. The camera re-measures (a few seconds) and asks again.
   Now press just **Enter** (= keep).
4. Expected:
```
  [PASS] pose jitter while freezing: 0.30 mm (< 1)
  [PASS] tilt correction from the depth plane: 0.90 deg (< 3)
  [PASS] marker size seen by depth 37.2 mm vs ruler 37.0 mm (+0.6 %, within 3 %)
  [PASS] live marker vs frozen pose: 0.8 mm (< 3)
  [PASS] the 4 sockets are at the same height (spread 1.3 mm < 2)
```
Any FAIL → README 6.1 (depth calibration), then step 3 again.
**Saved:** `camera: ...` and `length_m` in `peg_detector_overrides.yaml`; used automatically from now on.

### Step 4 — Measurements (T2: `4` + Enter, ~10 min)
Have the ruler / caliper and the 4 pegs ready. Type each value in **mm** + Enter:
```
  board thickness (foam to board top): 6.8
  socket depth (board top to bottom of the hole): 15
  pegs stand on the foam (f) or on the board (b)? [f]: f
  green  peg height: 45
  green  peg width (across flats / diameter): 30
  ... (orange, blue, yellow)
```
Put the 4 pegs standing, visible, before you finish typing (the wizard compares right after).
Expected:
```
  [PASS] socket Z 6.8 mm vs board 6.8 mm (+0.0, < 2)
  [PASS] green  peg top 46.2 mm vs ruler 45.0 mm (+1.2, < 3)
  ...
```
**Saved:** `measurements:` (all your ruler values + the camera values)

### Step 5 — Gripper (T2: `5` + Enter, ~5 min)
Only `o` and `p` do anything here.
1. `>> open the gripper fully with o, Enter` → press `o` until it stops opening, then **Enter**.
2. For each colour: `>> green peg: close until it just grips, Enter (x = skip)` → hold that peg
   between the fingers with your hand, press `p` until it holds the peg firmly, then **Enter**.
   The gripper opens again; take the peg out.
3. `>> close fully (no peg) with p, Enter` → `p` until closed, then **Enter**.

**Saved:** `gripper: open_m, grip_m: {green: ..., ...}, closed_m` (in metres)

### Step 6 — Camera → arm frame (T2: `6` + Enter, ~20 min, the important step)
```
  Peg colour to use [green]:                      → Enter
  Number of points (6 recommended, min 3) [6]:    → Enter
```
The gripper closes. Then **6 times**:
```
  --- point 1/6 ---
  Move the arm to the park pose first (so it does not hide the peg)? (y/n) [y]:   → Enter
  Place the green peg at a NEW spot (spread them out), hands away, then Enter     → place peg, Enter
  vision: x -15.0  y +10.0  z_top +4.6 cm  (jitter 0.4 mm)
  >> jog the fingertips (closed, pointing down) to just TOUCH the centre of the peg top, Enter
```
Jog: `3` and `w/s/a/d` to get above the peg, `r/f` to come down, `2` when close, `1` for the last
millimetres. Look from the side **and** from the front: the fingertips should touch the
**middle** of the peg top, without pushing it. Press **Enter**.
```
  robot tip: x -14.8  y +10.3  z +4.5 cm   (vision - tip = 4 mm)
```
The arm lifts 5 cm by itself. Next point.

**Where to put the 6 points:** 2 near the board, 2 near the arm base, 1 at the far left end,
1 at the far right end. Spread out = good fit.

At the end:
```
  residuals after the fit:
   point 1:   0.9 mm
   ...
  RMS 2.2 mm   max 4.2 mm
  [PASS] fit quality: RMS 2.2 mm (< 4), max 4.2 mm (< 7)
  applied to the running detector and saved to /home/<you>/omx_calibration/peg_detector_overrides.yaml
```
If it asks `Point 5 looks bad ... Drop it and refit? (y/n)` → Enter (yes).
FAIL (RMS ≥ 4) → run step 6 again, touching more carefully.
**Saved:** `robot_frame: yaw_deg, offset_m, rms_mm, pairs`. **The image labels now show arm coordinates.**

### Step 7 — Verification (T2: `7` + Enter, ~10 min)
Same as step 6, but **3 new spots** you did not use in step 6, and nothing is fitted:
```
  error: dx +0.1  dy +2.0  dz -0.5 mm   xy 2.0 mm
  ...
  [PASS] xy error mean 1.5 / max 2.0 mm, z error max 1.2 mm (< 5)
```
**Write these numbers in your report.** FAIL → README 6.2.
**Saved:** `verification: points, xy_mean_mm, xy_max_mm, z_max_abs_mm`

### Step 8 — Poses (T2: `8` + Enter, ~5 min)
1. `>> jog to a good park pose ...` → jog the arm somewhere it hides **no** peg, socket or
   marker in the image window, then **Enter**.
2. `>> jog the (pointing-down) tip to the travel height over the pegs, Enter` → jog above the
   tallest peg, ~3–5 cm higher than its top, then **Enter**.

**Saved:** `poses: park_joints, travel_z_m`

### Step 9 — Summary (T2: `9` + Enter)
```
  1. preflight     PASS  (2026-11-10 10:12:03)
  ...
  8. poses         PASS  (2026-11-10 11:31:40)

  calibration file: /home/<you>/omx_calibration/calibration.yaml
  report:           /home/<you>/omx_calibration/report.md

  EVERYTHING PASSED - the pick-and-place code can use calibration.yaml + /peg_detector/scene.
```
Type `q` + Enter to leave the wizard.

---

## PART D — Check what was recorded (T3)
```bash
cat ~/omx_calibration/calibration.yaml              # every result
cat ~/omx_calibration/peg_detector_overrides.yaml   # what the camera node loads at start
cat ~/omx_calibration/report.md                     # summary for the course report
ros2 topic echo /peg_detector/scene --once          # live peg + socket positions (metres, arm frame)
```
Save a copy for the report:
```bash
cp -r ~/omx_calibration ~/omx_calibration_backup_$(date +%F)
```

## PART E — Next sessions
1. B1 → B2 → B3 in T1 (the calibration loads automatically) → B5 in T2.
2. Run **step 1**, then **step 7 with 2 points** (`Number of new points [3]: 2`).
   Both PASS → ready. FAIL → README section 7 tells you which steps to redo.

## PART F — Stopping
- T2: `q` + Enter
- T1: **Ctrl+C** (the arm keeps holding its pose; support it before switching the 12 V off)

## If something is unclear
Copy the last ~30 lines of **T1** and **T2** and send them to me.
