# Lab-day guide: calibrate the camera to the robot in one go

Follow this file from top to bottom. Every stage ends with a **CHECKPOINT**. If the checkpoint
passes, go on. If it fails, the box under it tells you exactly what to do. Never skip a
failed checkpoint: every later stage depends on the earlier ones.

**What we want at the end:** the camera reports every peg in **robot coordinates** (x, y, z in
metres, measured from the robot base) with an error below 5 mm, so your IK can use them directly.

**Time plan (~2.5 h)**
| Stage | What | Time |
|---|---|---|
| 0 | At home: install and test the laptop | 45 min (the day before) |
| 1 | Set up the table | 15 min |
| 2 | Start the software | 5 min |
| 3 | Arm check (during the camera warm-up) | 15 min |
| 4 | Camera rough calibration (marker) | 5 min |
| 5 | Exact calibration (touch 6 pegs) | 25 min |
| 6 | Proof (touch 3 new pegs) | 10 min |
| 7 | Save everything, take report pictures | 10 min |
| — | Spare time for one problem | 30 min |

**Two people:**
- **Person A** sits at the laptop and types everything.
- **Person B** watches the arm, keeps one hand near the **12 V power switch**, and does the
  ruler work and the "is it touching the middle?" checks.

If the arm does anything unexpected, **B switches the 12 V off** while supporting the arm with
the other hand (without power, the arm falls).

---

## STAGE 0 — The day before, at home (laptop only)

### 0.1 Install
Ubuntu 24.04 + ROS 2 Jazzy must already be installed. Then, in a terminal (**Ctrl+Alt+T**):
```bash
cd ~
unzip omx_calib_ws.zip
cd ~/omx_calib_ws
bash scripts/install_all.sh
```
- Type your password when asked (nothing shows while typing).
- If asked *"brltty is installed ... Remove it? [Y/n]"*, press **Enter**.
- Wait for **DONE** (30–40 min).
- If it printed *"Added to dialout: LOG OUT and back in"*: log out and log in again.

### 0.2 Test the install
Open a **new** terminal:
```bash
ros2 pkg list | grep -E "omx_calibration|peg_detection|open_manipulator_bringup|realsense2_camera"
```
**CHECKPOINT 0:** exactly these 4 lines:
```
omx_calibration
open_manipulator_bringup
peg_detection
realsense2_camera
```
> **Fails?** A line is missing → run `cd ~/omx_calib_ws && bash scripts/install_all.sh` again
> and send me the last 30 lines of its output.

### 0.3 Optional: test the camera at home
Plug in only the camera (blue USB port) and run:
```bash
ros2 launch peg_detection detection.launch.py
```
In a second terminal: `ros2 run rqt_image_view rqt_image_view`, then pick
`/peg_detector/annotated_image` from the list. You should see the camera image with
coloured outlines around anything peg-coloured. Stop both with **Ctrl+C**.

### 0.4 Pack this list
- [ ] laptop + charger
- [ ] **steel ruler** (mm marks) or caliper
- [ ] **set-square** or a phone with a level app
- [ ] **tape** (to fix the marker sheet flat)
- [ ] spare printed marker sheet (`markers/aruco_id0_80mm_A4.pdf`, printed at 100 % / "actual size")
- [ ] the 4 pegs
- [ ] this guide (printed or on a phone)

---

## STAGE 1 — Set up the table (15 min)

1. **Camera:** on its mount, tightened. From now on **nobody touches the camera**.
2. **Marker sheet:** flat on the table next to the robot base, **tape all 4 corners**. It
   must be fully visible to the camera, with no glare from lamps on it.
3. **Board and pegs:** board in place; pegs standing in front of it. Nothing on the table
   that the arm could hit.
4. **Cables:**
   - robot **12 V supply ON** (each motor LED blinks once);
   - **U2D2** USB into the laptop (not the port that gave `error -71` before);
   - **camera** into a **blue USB 3** port.
5. **Close Dynamixel Wizard and realsense-viewer** if they are open on the laptop.

Open **3 terminals** and arrange them side by side:
- **T1**: robot + camera (you don't type here after starting it)
- **T2**: the wizard (all typing happens here)
- **T3**: checks

In **T3**:
```bash
ros2 run omx_calibration preflight
```
**CHECKPOINT 1:** every line `[PASS]`, ends with `All good.`
> **Fails?** Under each `[FAIL]` line there is a `fix:` line. The usual ones:
> | FAIL line | Do this |
> |---|---|
> | `U2D2 serial port present` | replug the U2D2; try another USB port; run `sudo dmesg \| tail -5`: `error -71` = that port/cable is bad |
> | `latency timer = 1 ms` | unplug the U2D2, wait 3 s, plug it back in, run preflight again |
> | `Dynamixel Wizard closed` | close Dynamixel Wizard |
> | `RealSense D435i connected` / `on USB 3` | camera into another blue port; close realsense-viewer |
> | `user in dialout group` | log out and log in again |

---

## STAGE 2 — Start the software (5 min)

In **T1**:
```bash
ros2 launch omx_calibration bringup.launch.py
```
What happens:
1. **The arm moves** to its start pose (B: hands clear).
2. About 3 s later the camera starts.
3. About 6 s later a **video window** opens.
4. When the marker is visible, T1 prints **`Camera pose FROZEN from 30 frames`** and the
   bottom line of the video window turns **green**.

In **T2**:
```bash
ros2 run omx_calibration wizard
```
You see the wizard menu (steps 1–9, then `a`, `p`, `q`).

**CHECKPOINT 2:**
- [ ] the arm is standing still in its start pose
- [ ] the video window shows the table; peg outlines are drawn; **white dots on the peg tops**
- [ ] the bottom line of the video window is green: `camera pose FROZEN`
- [ ] T2 shows the menu
> **Fails?**
> | What you see | Do this |
> |---|---|
> | bottom line red `NO camera pose` | the arm covers the marker → in T2 type `p` + Enter (moves the arm away), then in T3: `ros2 service call /peg_detector/recalibrate std_srvs/srv/Trigger` |
> | a peg has no outline | light/colour problem; doesn't block calibration as long as the **green** peg is detected |
> | T1 shows `no status packet` / `process has died` | Ctrl+C in T1, check 12 V + USB, run preflight (T3), start T1 again |
> | video window grey | wait 10 s; still grey → Ctrl+C in T1 and start again |

**Start a 15-minute timer now.** The camera's depth needs to warm up before Stage 4. Do Stage 3 in the meantime.

### How to use the wizard (read once)
- **Menu:** type the step number + **Enter**.
- **Questions** look like `Move to zero now? (y/n) [y]:`. The value in `[ ]` is the default;
  just **Enter** accepts it.
- **Numbers** are always in **millimetres**: type e.g. `37.0` + Enter.
- **Moving the arm ("jog")**: when a line starts with `>>`, single keys move the gripper tip
  (no Enter needed). Click on **T2** first, or the keys go nowhere.

| Key | Gripper tip | | Key | |
|---|---|---|---|---|
| `w` / `s` | forward / back | | `3` | big steps 2 cm |
| `a` / `d` | left / right | | `2` | medium 5 mm |
| `r` / `f` | up / down | | `1` | tiny 1 mm |
| `o` / `p` | gripper open / close | | **Enter** | **save position** |
| `x` | cancel | | | |

The gripper always points straight down while jogging. "Forward" is the robot's forward:
if a key moves the arm the wrong way, use the opposite key. A live line shows
`tip x .. y .. z .. cm | step .. | base .. deg`.

---

## STAGE 3 — Arm check (wizard step 2, 15 min)

**Why:** the camera calibration trusts the robot's own idea of where its gripper is. If a
motor is a few degrees off, the robot misses even with a perfect camera. This also checks
the base-turning problem (3c).

In **T2**: `2` + Enter.

**3a. Model check (automatic).** Expected: `[PASS] kinematics model matches TF ... (0.00 mm)`

**3b. Zero pose.**
- `Move to zero now? (y/n) [y]:` → B clears the space **above and in front** of the arm → **Enter**.
- The arm stands up straight, pointing forward.
- B checks with the set-square / level app:
  - the upper arm (motor 12 → motor 13) is **vertical**;
  - the forearm (13 → 14) and the gripper are **horizontal** and in one straight line.
- `Is that what you see (each within ~2 deg)? (y/n) [y]:`
  - yes → **Enter**;
  - no → `n` + Enter, then type what's wrong (e.g. `motor 13 forearm 4 deg down`) + Enter.
- `Move back to where it was? (y/n) [y]:` → **Enter**.

**3c. Base-turning check.** Three jogs; for each, move the gripper **above** the named place
(stay high: press `r` a few times first), then **Enter**:
1. `>> move over the middle of the peg area` → `3`, then `w/a/s/d` until it's above the pegs → Enter
2. `>> move over the +y end of the board` → Enter when above one end of the board
3. `>> move over the -y end of the board` → Enter when above the other end

**CHECKPOINT 3:** the step ends with all lines `[PASS]`, including
`[PASS] workspace does not cross the base joint +-180 deg limit (range ... deg)`.

> **Fails: zero pose not straight (3b)?**
> A few degrees off is usually a motor's zero setting. Send me the description you typed
> and the photo; we decide together whether to fix it now. Continuing works, but the final
> accuracy will be worse.
>
> **Fails: base joint limit (3c)?** The base can't turn past ±180°, and your work area
> sits on both sides of that point. The fix (10 min):
> 1. **T2:** `q` + Enter. **T1:** Ctrl+C. (The arm keeps holding; B supports it anyway.)
> 2. Open **Dynamixel Wizard 2.0** → *Options*: Protocol 2.0, port `/dev/ttyUSB0`, baud 1000000 → **Scan**.
> 3. Click motor **ID 11** (the base) → in the control table, click **Torque Enable** → set **0** → **Save**.
>    (Only the base motor turns off; it doesn't carry weight, so nothing falls.)
> 4. Click **Homing Offset** (address 20) → type **2048** → **Save**.
> 5. **Close Dynamixel Wizard completely.**
> 6. **Warning:** at the next start the robot will **turn its base 180°** to face the board.
>    Clear the board area, B ready at the switch.
> 7. **T1:** `ros2 launch omx_calibration bringup.launch.py`, wait for `FROZEN`.
>    **T2:** `ros2 run omx_calibration wizard` → `2` + Enter again. Now 3c must PASS.

---

## STAGE 4 — Rough calibration from the marker (wizard step 3, 5 min)

**Why:** the marker gives the camera a first idea of where the robot is. Its exact size
matters: a 1 mm error in the size makes every distance ~3 % wrong.

Wait until the **15-minute warm-up** has passed.

1. In T2's menu: `p` + Enter (the arm moves away from the marker).
2. B measures the marker's **black square** from edge to edge with the steel ruler, in mm
   (for example 37.0). Only the black square, not the white border.
3. T2: `3` + Enter.
4. `RULER: side of the black square of marker 0 in mm (Enter = keep):` → type the number + Enter.
5. Wait a few seconds (the camera re-measures). The same question comes again → just **Enter**.

**CHECKPOINT 4:** five lines, all `[PASS]`:
```
  [PASS] pose jitter while freezing: ... mm (< 1)
  [PASS] tilt correction from the depth plane: ... deg (< 3)
  [PASS] marker size seen by depth ... mm vs ruler ... mm (... %, within 3 %)
  [PASS] live marker vs frozen pose: ... mm (< 3)
  [PASS] the 4 sockets are at the same height (spread ... mm < 2)
```
> **Fails?**
> | FAIL line | Cause → do this |
> |---|---|
> | jitter / live vs frozen | the camera or the sheet moves → tighten the mount, re-tape, run step 3 again |
> | tilt correction | marker sheet not flat or shiny → flatten, re-tape, block lamp glare, run step 3 again |
> | marker size seen by depth | re-measure with the ruler first. Still failing → the camera's depth needs its own calibration: **box below** |
> | sockets not at same height | board not flat on the table, or depth problem → box below |
>
> **Camera depth calibration (15 min):**
> 1. T2: `q` + Enter. T1: Ctrl+C.
> 2. In a terminal: `realsense-viewer`. Switch on **Stereo Module**.
> 3. Point the camera at a flat, textured surface (a wall with a poster, the table) 0.5–1 m away.
> 4. In the camera's panel click **More** → **On-Chip Calibration** → start; if the health
>    number gets better, **apply** it.
> 5. Again **More** → **Tare Calibration** and follow its instructions.
> 6. Close realsense-viewer, put the camera back on its mount **exactly as before** (or the
>    whole calibration starts over from Stage 4), start T1 and T2 again, redo Stage 4.

---

## STAGE 5 — Exact calibration: touch 6 pegs (wizard step 6, 25 min)

**Why:** this is the important step. For each point the program gets two numbers for the
same spot: where the **camera** sees the peg top, and where the **robot** says its gripper
tip is when it touches it. From 6 such pairs it computes the exact conversion.

**How to touch well:**
- fingers **closed**, gripper pointing **straight down** (the wizard does both);
- the middle of the fingertips **just touches the centre of the peg top**, without pushing
  the peg (if the peg moves, that point is spoiled);
- B checks from the **side** *and* from the **front** before Enter;
- last few mm with `1` (1 mm steps).

**Where to put the 6 points:** spread out over the work area:
```
   [board]   P1         P2          (near the board)
             P5   pegs area   P6    (far left, far right)
             P3         P4          (near the robot)
```

In **T2**: `6` + Enter → `Peg colour to use [green]:` **Enter** → `Number of points [6]:` **Enter**.
The gripper closes. Then, **6 times**:

| Wizard says | You do |
|---|---|
| `Move the arm to the park pose first ...? [y]` | **Enter** (arm moves out of the way) |
| `Place the green peg at a NEW spot ..., then Enter` | B puts the green peg at the next spot, hands away → **Enter** |
| `vision: x .. y .. z_top .. cm (jitter .. mm)` | (just information; jitter should be < 2 mm) |
| `>> jog the fingertips ... TOUCH the centre of the peg top, Enter` | `3` + `w/a/s/d` to get above the peg → `f` to come down → `2`, then `1` near the top → B checks side + front → **Enter** |
| `robot tip: x .. y .. z ..` | the arm lifts 5 cm by itself → next point |

At the end:
```
  residuals after the fit:
   point 1:   1.1 mm
   ...
  RMS 2.4 mm   max 3.9 mm
  [PASS] fit quality: RMS 2.4 mm (< 4), max 3.9 mm (< 7)
  applied to the running detector and saved to .../peg_detector_overrides.yaml
```
If it asks `Point N looks bad ... Drop it and refit? (y/n)` → **Enter** (yes).

**CHECKPOINT 5:** `[PASS] fit quality`. The labels in the video window now show robot
coordinates (they changed a little).
> **Fails (RMS ≥ 4 mm)?** Usually a peg moved while touching, or the touch was off-centre.
> Run step 6 again (`6` + Enter), more carefully, with the 1 mm steps.
> Fails twice → send me the residual lines.

---

## STAGE 6 — Proof: touch 3 new pegs (wizard step 7, 10 min)

**Why:** step 6 fits its own points well by construction. Only **new** points show the real
accuracy. This is the number for your report.

In **T2**: `7` + Enter → **Enter** (green) → **Enter** (3 points).
Same procedure as Stage 5, at **3 spots not used before** (e.g. between the earlier ones).
After each point:
```
  error: dx +0.8  dy -1.5  dz +0.4 mm   xy 1.7 mm
```
**CHECKPOINT 6:**
```
  [PASS] xy error mean ... / max ... mm, z error max ... mm (< 5)
```
**The calibration is done.**
> **Fails?**
> | Pattern | Cause → do this |
> |---|---|
> | all errors in the same direction | touches in Stage 5 were consistently off-centre → redo Stage 5, then 6 |
> | errors grow the farther from the marker | marker not flat → re-tape, redo Stages 4, 5, 6 |
> | errors differ with how far the arm reaches | a motor's zero (Stage 3b) → tell me |
> | one point very bad | that peg moved → run step 7 again |

---

## STAGE 7 — Save everything and collect report material (10 min)

In **T2**: `9` + Enter (summary + report file), then `q` + Enter.

In **T3**:
```bash
cat ~/omx_calibration/report.md                      # the summary
ros2 topic echo /peg_detector/scene --once           # peg positions in robot coordinates
cp -r ~/omx_calibration ~/omx_calibration_backup_$(date +%F)
```
For the report, collect:
- [ ] a screenshot of the video window (white dots on the pegs, labels with coordinates)
- [ ] the Stage 6 numbers (error per point, mean, max)
- [ ] `~/omx_calibration/report.md`
- [ ] a photo of the set-up (camera, marker, board, robot)

If time is left: wizard steps **5** (gripper values) and **8** (parking pose and travel
height). The pick-and-place code will use them.

**Final check** in T3 (shows what your IK will get):
```bash
ros2 topic echo /peg_detector/scene --once
```
```
"calibration": "frozen", "frame_id": "link1",
"pegs": {"green": {"x": -0.0941, "y": 0.0212, "z_top": 0.0902, ...}, ...},
"sockets": {"circle": {"x": -0.1682, "y": 0.1209, "z": 0.0100, ...}, ...}
```
x, y, z are in **metres from the robot base**. `z_top` is the top of the peg.

---

## Afterwards — using it with your IK
- Start `ros2 launch omx_calibration bringup.launch.py`; the calibration loads by itself.
- Read `/peg_detector/scene`, take a peg's `x, y, z_top`, give it to the IK, and send the joint
  angles to `/arm_controller/joint_trajectory`.
- Heights: the robot base point is the ROBOTIS standard (`link1`), with the shoulder 0.0595 m
  above it. Your old `ik.py` used 0.077 m, measured from the bottom of the plate, so change it to 0.0595.
- Or use the tested IK in this workspace:
  `from omx_calibration import kinematics as K; q, ok = K.ik((x, y, z), K.DOWN, q0=current_q)`

**Next sessions:** start T1 + wizard, run step 7 with 2 points. Under 5 mm → go.
**Redo Stages 4–6** whenever the **camera, the marker or the robot moved**.

## Safety, all the time
- B keeps a hand near the 12 V switch whenever the arm moves.
- Before any wizard step that moves the arm, look at the table: nothing in the way.
- Stopping T1 (Ctrl+C) leaves the arm holding its pose. Support the arm before switching 12 V off.
- Never touch the camera, marker or board between Stage 4 and Stage 6.

## Stuck?
Copy the last 30 lines of **T1** and **T2** (and a photo if it's about the arm) and send them to me.
