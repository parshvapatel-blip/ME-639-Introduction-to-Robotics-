# Lab 2 — Challenge 3: Forward Kinematics (DH) in MuJoCo

For two robot arms, forward kinematics is computed from a **Denavit–Hartenberg (DH) table**, and the result is compared live against MuJoCo's own end-effector pose.

🎥 Demo: [demo.mp4](./demo.mp4)

## Jump straight to the source code

| Robot | DOF | DH convention | Main script |
|---|---|---|---|
| **Franka Emika Panda** | 7 | Modified (Craig) | [`Franka-mujoco/script.py`](./Challange_3%20%28Forward%20Kinematics%29/Franka-mujoco/script.py) |
| **Addverb Heal** | 6 | Standard | [`Heal-mujoco/script.py`](./Challange_3%20%28Forward%20Kinematics%29/Heal-mujoco/script.py) |

Supporting files (in [`Challange_3 (Forward Kinematics)/`](./Challange_3%20%28Forward%20Kinematics%29/)):

- [`ITR_mujoco_fk_lab/`](./Challange_3%20%28Forward%20Kinematics%29/ITR_mujoco_fk_lab/) — lab package with all robot XMLs/meshes and `requirements.txt`
- [`6dofpythonscriptDH.py`](./Challange_3%20%28Forward%20Kinematics%29/6dofpythonscriptDH.py) / [`7dofpythonscriptDH.py`](./Challange_3%20%28Forward%20Kinematics%29/7dofpythonscriptDH.py) — minimal numpy-only DH tip-position calculators
- [`2 Dof non planar DH.pdf`](./Challange_3%20%28Forward%20Kinematics%29/2%20Dof%20non%20planar%20DH.pdf) — 2-DOF non-planar DH worked example

---

## What both `script.py` files do

1. Open a **Tk slider window** with one slider per joint (degrees), showing the DH table.
2. Compute forward kinematics from the DH table for the slider values.
3. Drive the MuJoCo robot to the same joint angles.
4. Draw the DH frames, the end-effector frame and a marker at the **DH-predicted** pose inside the viewer (optional trail).
5. Show the **position error between DH-FK and MuJoCo** live (in mm).

Options in the GUI: Home / Random pose, Smooth motion, Show DH frames, Trail.

### Franka Panda — [`Franka-mujoco/script.py`](./Challange_3%20%28Forward%20Kinematics%29/Franka-mujoco/script.py)
- Modified DH from the Franka docs, `T = Rx(α) Tx(a) Rz(θ) Tz(d)`.
- Flange offset 0.107 m, hand yaw −45°, TCP offset 0.1034 m (set `TCP_OFFSET = 0.0` for the flange).
- Model: `robot_descriptions/franka/mjx_scene.xml`

### Addverb Heal — [`Heal-mujoco/script.py`](./Challange_3%20%28Forward%20Kinematics%29/Heal-mujoco/script.py)
- Standard DH, `T = Rz(θ) Tz(d) Tx(a) Rx(α)`, with per-joint θ offsets in `DH_TABLE`.
- Model: `robot_descriptions/single_arm_heal_effort_actuation_rs_mj_2.xml`

---

## Run

```bash
cd "Challange_3 (Forward Kinematics)"
pip install mujoco numpy
sudo apt install python3-tk            # Linux, needed for the slider window

python Franka-mujoco/script.py         # GUI + MuJoCo viewer
python Heal-mujoco/script.py

# headless DH-vs-MuJoCo self-test (200 random poses, no window)
python Franka-mujoco/script.py --check
python Heal-mujoco/script.py --check
```

Useful flags: `--xml PATH` (use a specific XML), `--speed DEG_PER_S` (smooth-motion speed, default 90).
On macOS, start the viewer with `mjpython` instead of `python`.

> **Important — XML path lookup:** each script searches for `ITR_mujoco_fk_lab/` relative to its own location, then falls back to a hardcoded `/home/parshva/...` path. Because `script.py` sits in `Franka-mujoco/` / `Heal-mujoco/` (next to, not inside, `ITR_mujoco_fk_lab/`), auto-detect will fail on other machines — pass the XML explicitly:
> ```bash
> python Franka-mujoco/script.py --xml "ITR_mujoco_fk_lab/robot_descriptions/franka/mjx_scene.xml"
> python Heal-mujoco/script.py   --xml "ITR_mujoco_fk_lab/robot_descriptions/single_arm_heal_effort_actuation_rs_mj_2.xml"
> ```

## Folder layout

```
Lab2/
├── README.md
├── demo.mp4
└── Challange_3 (Forward Kinematics)/
    ├── Franka-mujoco/script.py     <-- Franka main code
    ├── Heal-mujoco/script.py       <-- Heal main code
    ├── ITR_mujoco_fk_lab/          (robot XMLs, meshes, requirements.txt)
    ├── 6dofpythonscriptDH.py
    ├── 7dofpythonscriptDH.py
    └── 2 Dof non planar DH.pdf
```

## Requirements
Python 3.9+, `mujoco`, `numpy`, `python3-tk` (Linux). Full lab deps in `ITR_mujoco_fk_lab/requirements.txt`.
