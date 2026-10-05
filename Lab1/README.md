# Lab 1 — Rotation Matrices & Teleoperation in MuJoCo

Two challenges, each with a keyboard-teleoperated robot in MuJoCo that shows the **live rotation matrix `R`** of the robot's base frame as an on-screen overlay.

🎥 Demo: [demo.mp4](./demo.mp4)

## Jump straight to the source code

| Challenge | Robot | Main script | Model file |
|---|---|---|---|
| **Challenge 1** | TurtleBot3 Waffle Pi (differential drive) | [`Challange_1/teleop.py`](./Challange_1/teleop.py) | [`turtlebot_mjcf.xml`](./Challange_1/turtlebot_mjcf.xml) |
| **Challenge 2** | Skydio X2 (quadrotor drone) | [`Challange_2/teleop.py`](./Challange_2/teleop.py) | [`skydio_x2/scene.xml`](./Challange_2/skydio_x2/scene.xml) |

---

## Challenge 1 — TurtleBot teleop ([`teleop.py`](./Challange_1/teleop.py))

Drives a differential-drive TurtleBot and prints the base rotation matrix `R` and commanded velocities above the robot.

- **Kinematics:** `v_left / v_right` are computed from the linear velocity `v` and angular velocity `w` using `WHEEL_RADIUS = 0.033 m` and `TRACK_WIDTH = 0.287 m`, then converted to wheel rad/s for `data.ctrl`.
- **Limits:** `CMD_LIN_VEL = 0.35 m/s`, `CMD_ANG_VEL = 1.5 rad/s`.
- **Viewer:** `mujoco.viewer.launch_passive` with a key callback.

| Key | Action |
|---|---|
| `W` / `I` / ↑ | Forward |
| `S` / `K` / ↓ | Backward |
| `A` / `J` / ← | Turn left |
| `D` / `L` / → | Turn right |
| `Space` / `X` | Full stop |
| `Esc` | Exit |

**Run**
```bash
cd Challange_1
pip install mujoco numpy
python teleop.py          # macOS: mjpython teleop.py
```
> Run from inside `Challange_1/` — the script loads `turtlebot_mjcf.xml` (and the `.stl` meshes beside it) via a relative path.

---

## Challenge 2 — Skydio X2 drone teleop ([`teleop.py`](./Challange_2/teleop.py))

Flies a quadrotor with a cascaded attitude controller and draws both the world frame and the drone's body frame (RGB = XYZ arrows) plus the live rotation matrix and telemetry.

- **Mixer:** builds a 4-motor mixer from the actuator site positions (thrust / pitch / roll / yaw).
- **Control:** PD attitude loop for pitch and roll, rate-tracking loop for yaw, hover thrust = `m·g / n_motors`.
- **Feel:** self-centering sticks (ramp in / ramp out) and throttle that holds its value like a real transmitter.
- **Rendering:** custom GLFW window with mouse orbit camera (left-drag rotate, right-drag / scroll zoom).

| Key | Action |
|---|---|
| ↑ / ↓ | Pitch forward / backward |
| ← / → | Yaw left / right |
| `W` / `S` | Throttle up / down |
| `X` | Reset commands |
| `Esc` | Exit |

**Run**
```bash
cd Challange_2
pip install mujoco numpy glfw
python teleop.py
```
> Run from inside `Challange_2/` — the script loads `skydio_x2/scene.xml` via a relative path.

---

## Folder layout

```
Lab1/
├── README.md
├── demo.mp4
├── Challange_1/
│   ├── teleop.py            <-- main code
│   ├── turtlebot_mjcf.xml
│   └── *.stl                (robot meshes)
└── Challange_2/
    ├── teleop.py            <-- main code
    └── skydio_x2/           (MuJoCo Menagerie Skydio X2 model)
```

## Requirements
Python 3.9+, `mujoco`, `numpy`, and `glfw` (Challenge 2 only).
