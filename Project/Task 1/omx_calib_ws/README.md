# Camera calibration for the peg-sorting robot — simple guide

> **Going to the lab? Follow `LAB_DAY_GUIDE.md`** — one path, checkpoints, and what to do if a check fails.

## What we are doing, in one picture

```
   CAMERA                                        ROBOT
   "I see the green peg at                       "Tell me where the peg is,
    pixel (412, 230)"                              measured from MY base"
            \                                         /
             \___________ CALIBRATION _______________/
                  finds the link between the two
```

The camera and the robot measure positions from different places, so the same peg has
different numbers for each of them. **Calibration finds the conversion between camera
numbers and robot numbers.** After that, every peg the camera sees is reported in robot
numbers (x, y, z from the robot base, in metres). You give those to your IK and the arm
goes there.

We do it in three parts:

| Part | What happens | Why |
|---|---|---|
| **A. Rough guess** | The camera looks at the printed marker next to the robot | gives a first conversion, usually off by 1–2 cm |
| **B. Exact fix** | You touch 6 peg tops with the gripper; the program compares "where the camera saw the peg" with "where the robot says its gripper is" | corrects the rough guess to a few mm |
| **C. Proof** | You touch 3 new pegs | shows the real error, a number for your report |

That is all the calibration. The program that guides you is called the **wizard**. A, B and C
are wizard steps **3, 6 and 7**. The wizard has other steps too; they are optional checks,
explained at the end.

---

## 1. Install (once per laptop, about 40 minutes)

You need Ubuntu 24.04 with ROS 2 Jazzy already installed.

Open a terminal (**Ctrl+Alt+T**) and type:
```bash
cd ~
unzip omx_calib_ws.zip
cd ~/omx_calib_ws
bash scripts/install_all.sh
```
- It asks for your password a few times (nothing shows while you type; press Enter).
- If it asks *"brltty is installed ... Remove it? [Y/n]"*, press **Enter**.
- When it finishes, it prints **DONE**.
- If it says *"Added to dialout: LOG OUT and back in"*, log out of Ubuntu and log in again.

Close the terminal. Every new terminal now knows the new programs.

---

## 2. Every time you work with the robot

### 2.1 Switch things on
1. Robot power supply (12 V) **ON**.
2. Robot USB adapter (U2D2) into the laptop. Camera into a **blue** USB port.
3. If **Dynamixel Wizard** or **realsense-viewer** is open, **close it**.
4. The printed marker must be flat (tape the corners) and not covered by the arm.

### 2.2 Terminal 1 — start the robot and the camera
Open a terminal and type:
```bash
ros2 launch omx_calibration bringup.launch.py
```
- The arm moves to its starting pose. Keep your hands away.
- After a few seconds a **video window** opens.
- Wait until this line appears in the terminal: **`Camera pose FROZEN`**.
  (The bottom line of the video window also turns green.)

**Leave Terminal 1 alone.** It shows messages; you never type in it. To stop everything later, click on it and press **Ctrl+C**.

### 2.3 Terminal 2 — start the wizard
Open a **second** terminal and type:
```bash
ros2 run omx_calibration wizard
```
You see a menu:
```
  1. preflight     not done
  2. arm           not done
  3. camera        not done
  ...
  9. summary
  a. run steps 1-9 in order   p. park the arm   q. quit
choice:
```
**Everything you do from now on is typed in Terminal 2.** Click on it before typing.

---

## 3. How to talk to the wizard

**Questions.** The wizard asks things like:
```
Move to zero now? (y/n) [y]:
```
The value in `[ ]` is the default; pressing just **Enter** answers it. Type `n` and Enter for no.
When it asks for a measurement, type the number in **millimetres** and press Enter.

**Moving the arm ("jog").** When a line starts with `>>`, you drive the gripper with single
keys (no Enter needed):

| Key | Gripper tip moves |
|---|---|
| `w` / `s` | forward / backward |
| `a` / `d` | left / right |
| `r` / `f` | up / down |
| `3` | big steps (2 cm), for travelling |
| `2` | medium steps (5 mm) |
| `1` | tiny steps (1 mm), for the last bit |
| `o` / `p` | open / close the gripper |
| **Enter** | **save this position** |
| `x` | cancel |

The gripper always points straight down while you jog. If the wizard first asks to *"rotate
the wrist to point down"*, press Enter.

"Forward" means the robot's own forward, so on the first try a key may move the arm the way
you didn't expect. Watch the arm and use the opposite key.

A line keeps updating so you can see where the gripper is:
```
  tip x -11.0  y +6.0  z +8.5 cm | step 5 mm | gripper -10.0 mm | base +154 deg
```

**Saving.** You never edit files. Every value you type and every **Enter** in jog mode is
saved automatically into `~/omx_calibration/calibration.yaml`.

---

## 4. The calibration (about 45 minutes)

Before you start: leave the camera running **10–15 minutes** (its depth measurement drifts
while it warms up). After that, **don't touch the camera, the marker or the board** until
you're done.

### Step 3 — Rough guess from the marker (5 minutes)
In Terminal 2 type `3` and Enter.

1. Move the arm away from the marker: type `p` in the menu first, or just make sure the arm
   isn't covering it.
2. With a steel ruler, measure the **black square** of the marker from edge to edge, in mm.
3. The wizard asks:
   ```
   RULER: side of the black square of marker 0 in mm (Enter = keep):
   ```
   Type your number (for example `37.0`) and press Enter. Wait a few seconds; it asks again.
   This time press just **Enter**.
4. You should see 5 lines, each starting with **[PASS]**.

If a line says **[FAIL]**: the marker is not flat, the camera moved, or the camera's depth is
off. Flatten and re-tape the marker and repeat step 3. If it still fails, see section 7.

### Step 6 — Exact fix: touch 6 pegs (25 minutes) — the important one
In Terminal 2 type `6` and Enter. Press **Enter** twice (green peg, 6 points).
The gripper closes. Now repeat the following **6 times**:

1. Wizard: *"Move the arm to the park pose first?"* → **Enter** (moves the arm out of the camera's view).
2. Wizard: *"Place the green peg at a NEW spot"* → put the peg somewhere new, take your hand away, press **Enter**.
   The camera records where it sees the peg top.
3. Wizard: `>> jog the fingertips ... to just TOUCH the centre of the peg top`
   - press `3` and use `w a s d` to get roughly above the peg;
   - press `f` to come down; switch to `2`, then `1` as you get close;
   - look from the side **and** from the front: the closed fingertips should just touch the
     **middle** of the peg top, without pushing it;
   - press **Enter**.
4. The arm lifts by itself. Next point.

**Where to put the peg for the 6 points:** spread them out, e.g. 2 near the board, 2 near
the robot, 1 far left, 1 far right. Six points in one corner give a poor result.

At the end the wizard shows how well the 6 points agree:
```
  RMS 2.2 mm   max 4.2 mm
  [PASS] fit quality ...
```
**RMS under 4 mm = good.** The result is saved and used immediately; the labels in the video
window now show robot coordinates.

If one point was clearly wrong (the peg moved, or you touched off-centre), the wizard asks
whether to drop it: press Enter (yes). If it says FAIL, run step 6 again and touch more carefully.

### Step 7 — Proof: touch 3 new pegs (10 minutes)
In Terminal 2 type `7` and Enter. Press Enter twice (green, 3 points). It is the same as
step 6, but use **3 new spots**. For each one, the wizard shows the error:
```
  error: dx +0.1  dy +2.0  dz -0.5 mm   xy 2.0 mm
```
At the end:
```
  [PASS] xy error mean 1.5 / max 2.0 mm ...
```
**Under 5 mm = done.** Write these numbers in your report: this is how accurate your
camera-to-robot conversion is.

**The calibration is finished.** Type `q` and Enter to leave the wizard.

---

## 5. Using the result in your IK program

While Terminal 1 is running, the camera publishes every peg and socket position **in robot
coordinates** on the topic `/peg_detector/scene`. To look at it, open a third terminal:
```bash
ros2 topic echo /peg_detector/scene --once
```
You get something like:
```
"pegs":    {"green":  {"x": -0.0941, "y": 0.0212, "z_top": 0.0902, ...}, ...}
"sockets": {"circle": {"x": -0.1682, "y": 0.1209, "z": 0.0100, ...}, ...}
```
- Units are **metres**, measured from the robot base.
- `z_top` is the **top** of the peg; grab a little lower (for example `z_top - 0.015`).
- Only use the numbers when the message says `"calibration": "frozen"`.

Your IK takes these x, y, z values, computes the joint angles, and sends them to the robot on
`/arm_controller/joint_trajectory` (the same topic your `ik.py` already uses).

> The robot base point used here is the ROBOTIS standard one (called `link1` in ROS). Your
> old `ik.py` measured height from the bottom of the base plate (0.077 m to the shoulder),
> but the ROBOTIS standard is 0.0595 m. Use 0.0595 so your IK and the camera agree on
> heights, or use the ready-made IK in this workspace (section 8).

---

## 6. The other wizard steps — do you need them?

The calibration itself is steps 3, 6 and 7. The others are **checks and helpers**:

| Step | What it does | Needed? |
|---|---|---|
| 1 preflight | checks USB, cables, power, programs | only when something doesn't work |
| 2 arm | checks the arm's own angles are correct, and checks problem B below | **once, 5 min, recommended** |
| 4 measurements | compares ruler heights with camera heights | optional extra check |
| 5 gripper | records how far the gripper opens/closes for each peg | handy for the pick-and-place code; can be done later |
| 8 poses | records a safe "parking" pose and travel height | handy for the pick-and-place code; can be done later |
| 9 summary | writes a short report file | optional |

**Why step 2 is recommended.** The calibration only works if the arm's own idea of where its
gripper is matches reality. If one motor is a few degrees off at "zero", the arm thinks the
gripper is somewhere it isn't, and no camera calibration fixes that. Step 2 checks it in 5
minutes: the arm stands straight up, and you look whether it really is straight.

---

## 7. Two problems to know about

**Problem A — the camera's depth is off.** Step 3 says *"marker size seen by depth ... FAIL"*.
Fix: stop Terminal 1 (Ctrl+C), open `realsense-viewer`, and in the *Stereo Module* menu run
**On-Chip Calibration**, then **Tare Calibration** (it guides you). Close it, start Terminal 1
again, and redo step 3.

**Problem B — the base can't turn past 180°.** The robot's base motor can turn from −180° to
+180°, but not past that point. Your pegs and sockets are behind the robot, right around that
point. Going from a peg on one side to a socket on the other, the base would have to swing
almost a full circle the other way, which can hit things.
Step 2 checks this (it asks you to move the gripper over the middle and both ends of the work
area). If it says FAIL, the fix is one of:
- in **Dynamixel Wizard** (with Terminal 1 stopped): motor **ID 11** → *Torque Enable* = 0 →
  *Homing Offset* = **2048** → save. This turns the robot's "forward" by 180° so it faces the board.
- or physically turn the robot (or the board) by 180°.

Then redo steps 2, 6 and 7.

---

## 8. If you want: ready-made IK in this workspace
In Python:
```python
from omx_calibration import kinematics as K
q, ok = K.ik((x, y, z), K.DOWN, q0=current_joint_angles)   # gripper pointing down
```
`q` is the 4 joint angles in radians, `ok` says whether the point is reachable. This IK
matches the ROBOTIS robot model exactly (tested).

---

## 9. Next time (calibration already done)
1. Section 2: start Terminal 1 and the wizard. The calibration loads automatically.
2. Run **step 7** with 2 points as a quick check (answer `2` to *"Number of new points"*).
3. Under 5 mm? Ready to work. If not: the camera or the marker probably moved; redo steps 3, 6, 7.

Redo the calibration (steps 3, 6, 7) whenever the **camera, the marker or the robot has moved**.

---

## 10. Where things are saved
| File | Content |
|---|---|
| `~/omx_calibration/calibration.yaml` | all results |
| `~/omx_calibration/peg_detector_overrides.yaml` | the conversion the camera uses (loaded automatically) |
| `~/omx_calibration/report.md` | short summary (made by step 9) |

To see them: `cat ~/omx_calibration/calibration.yaml`

## 11. Something doesn't work
| What you see | What to do |
|---|---|
| Video window bottom line red: *NO camera pose* | the arm or your hand covers the marker; move it away and wait 3 s |
| Terminal 1: *no status packet* or *process has died* | robot connection: check 12 V, USB, close Dynamixel Wizard, then run `ros2 run omx_calibration preflight` |
| Jog says *not reachable* | too close to / too far from the robot, or too low; move somewhere else |
| Anything else | copy the last 30 lines of Terminal 1 and Terminal 2 and send them to me |

More technical details (how it works inside, all settings): `docs/REFERENCE.md`.
