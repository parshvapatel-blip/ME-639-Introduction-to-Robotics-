#!/usr/bin/env python3
"""
franka_dh_fk.py  -  Forward kinematics (DH) of the Franka Emika Panda in MuJoCo
================================================================================

* A Tk window with one slider per joint (theta_1 ... theta_7, in degrees).
* The DH forward kinematics (Craig / modified DH, straight from the Franka docs)
  computes the end-effector pose for the slider values.
* The MuJoCo robot is driven to those joint values, so its end effector goes to
  the DH-computed point.  A yellow sphere + RGB axes are drawn at the pose
  predicted by DH, and the window shows DH-vs-MuJoCo position error live.

Run
---
    python franka_dh_fk.py                 # GUI + MuJoCo viewer
    python franka_dh_fk.py --check         # headless self-test (no window)
    python franka_dh_fk.py --xml "/path/to/robot_descriptions/franka/mjx_scene.xml"

Needs:  pip install mujoco numpy      and      sudo apt install python3-tk
(On macOS the viewer has to be started with `mjpython` instead of `python`.)

Which XML?  franka/mjx_scene.xml  (panda + floor, robot base at the world origin).
Note: franka/scene.xml in the zip does NOT load - it has `<option integration=...>`
(should be `integrator`) - so it is not used here.
"""

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

# --------------------------------------------------------------------------- #
#  1. Where is the XML?                                                        #
# --------------------------------------------------------------------------- #
XML_REL = Path("robot_descriptions/franka/mjx_scene.xml")

_HERE = Path(__file__).resolve().parent
ROOT_CANDIDATES = [
    _HERE.parent,                      # script lives in ITR_mujoco_fk_lab/scripts/
    _HERE,                             # script lives in ITR_mujoco_fk_lab/
    Path("/home/parshva/ITR/Challange_3 (Forward Kinematics)/ITR_mujoco_fk_lab"),
    Path.home() / "ITR/Challange_3 (Forward Kinematics)/ITR_mujoco_fk_lab",
]


def find_xml(cli_path=None) -> Path:
    if cli_path:
        p = Path(cli_path).expanduser()
        if p.is_file():
            return p
        sys.exit(f"[error] --xml file not found: {p}")
    for root in ROOT_CANDIDATES:
        p = root / XML_REL
        if p.is_file():
            return p
    tried = "\n  ".join(str(r / XML_REL) for r in ROOT_CANDIDATES)
    sys.exit("[error] Could not find the Franka XML. Tried:\n  " + tried +
             "\nPass it explicitly with  --xml PATH")


# --------------------------------------------------------------------------- #
#  2. DH parameters  (Craig / modified DH, Franka Control Interface docs)      #
#     T(i-1 -> i) = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)      #
# --------------------------------------------------------------------------- #
PI = np.pi
#          a_{i-1} [m]   alpha_{i-1} [rad]   d_i [m]   theta_offset_i [rad]
DH_TABLE = [
    (0.0,      0.0,       0.333,  0.0),   # joint 1
    (0.0,     -PI / 2,    0.0,    0.0),   # joint 2
    (0.0,      PI / 2,    0.316,  0.0),   # joint 3
    (0.0825,   PI / 2,    0.0,    0.0),   # joint 4
    (-0.0825, -PI / 2,    0.384,  0.0),   # joint 5
    (0.0,      PI / 2,    0.0,    0.0),   # joint 6
    (0.088,    PI / 2,    0.0,    0.0),   # joint 7
]
FLANGE_D = 0.107            # link7 -> flange (= 'hand' body origin in the XML)
HAND_YAW = -PI / 4          # the hand body is rotated -45 deg about z in the XML
TCP_OFFSET = 0.1034         # flange -> point between the finger tips (set 0.0 for flange)

READY_POSE = np.deg2rad([0, -45, 0, -135, 0, 90, 45])   # Panda "ready" pose (inside limits)
NJ = len(DH_TABLE)


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def Tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def dh_modified(a, alpha, d, theta):
    """Closed form of Rx(alpha) Tx(a) Rz(theta) Tz(d)."""
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([
        [ct,      -st,      0.0,  a],
        [st * ca,  ct * ca, -sa,  -d * sa],
        [st * sa,  ct * sa,  ca,   d * ca],
        [0.0,      0.0,      0.0,  1.0]])


T_TOOL = Tz(FLANGE_D) @ Rz(HAND_YAW) @ Tz(TCP_OFFSET)   # frame 7 -> end effector (TCP)


def dh_frames(q, T_base):
    """World poses of DH frames 1..7 followed by the end-effector (TCP) frame."""
    frames, T = [], T_base.copy()
    for (a, alpha, d, off), qi in zip(DH_TABLE, q):
        T = T @ dh_modified(a, alpha, d, qi + off)
        frames.append(T.copy())
    frames.append(T @ T_TOOL)
    return frames


def dh_fk(q, T_base):
    return dh_frames(q, T_base)[-1]


def rot_to_rpy(R):
    """Roll-pitch-yaw (X-Y-Z fixed axes, i.e. R = Rz(yaw) Ry(pitch) Rx(roll)) in degrees."""
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-8:
        roll, yaw = np.arctan2(R[2, 1], R[2, 2]), np.arctan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = 0.0, np.arctan2(-R[0, 1], R[1, 1])
    return np.degrees([roll, pitch, yaw])


# --------------------------------------------------------------------------- #
#  3. MuJoCo helpers                                                           #
# --------------------------------------------------------------------------- #
class Robot:
    def __init__(self, xml_path: Path):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.jnt = [m.joint(f"joint{i + 1}") for i in range(NJ)]
        self.qadr = [j.qposadr[0] for j in self.jnt]
        self.limits = np.array([m.jnt_range[j.id] for j in self.jnt])      # rad
        self.fingers = [m.joint("finger_joint1").qposadr[0], m.joint("finger_joint2").qposadr[0]]
        self.hand = m.body("hand").id
        base = m.body("link0").id
        mujoco.mj_forward(m, self.data)
        self.T_base = np.eye(4)
        self.T_base[:3, :3] = self.data.xmat[base].reshape(3, 3)
        self.T_base[:3, 3] = self.data.xpos[base]

    def set_q(self, q):
        """Kinematic 'teleport': put joints at q and recompute all body poses."""
        d = self.data
        for adr, qi in zip(self.qadr, q):
            d.qpos[adr] = qi
        for adr in self.fingers:
            d.qpos[adr] = 0.04                      # gripper open
        d.qvel[:] = 0
        mujoco.mj_forward(self.model, d)

    def ee_pose(self):
        """End-effector (TCP) pose as MuJoCo sees it."""
        T = np.eye(4)
        T[:3, :3] = self.data.xmat[self.hand].reshape(3, 3)
        T[:3, 3] = self.data.xpos[self.hand]
        return T @ Tz(TCP_OFFSET)


# --------------------------------------------------------------------------- #
#  4. Drawing of the DH frames / target marker inside the MuJoCo viewer        #
# --------------------------------------------------------------------------- #
AXIS_RGBA = [np.array([1, 0.1, 0.1, 1], np.float32),
             np.array([0.1, 0.9, 0.1, 1], np.float32),
             np.array([0.2, 0.4, 1, 1], np.float32)]


def _add_arrow(scn, p0, p1, rgba, width):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                        np.zeros(9), rgba)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, p0, p1)
    scn.ngeom += 1


def _add_sphere(scn, pos, radius, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0, 0]),
                        np.asarray(pos, float), np.eye(3).flatten(), rgba)
    scn.ngeom += 1


def _add_axes(scn, T, length, width):
    for k in range(3):
        _add_arrow(scn, T[:3, 3], T[:3, 3] + length * T[:3, k], AXIS_RGBA[k], width)


def draw_overlay(scn, frames, show_frames, trail):
    scn.ngeom = 0
    if show_frames:                                  # every DH frame, small
        for T in frames[:-1]:
            _add_axes(scn, T, 0.07, 0.004)
    T_ee = frames[-1]
    _add_axes(scn, T_ee, 0.15, 0.006)                # end-effector frame, big
    _add_sphere(scn, T_ee[:3, 3], 0.018, np.array([1, 0.85, 0, 0.55], np.float32))
    for p in trail:
        _add_sphere(scn, p, 0.004, np.array([1, 0.3, 0.9, 1], np.float32))


# --------------------------------------------------------------------------- #
#  5. The slider window                                                        #
# --------------------------------------------------------------------------- #
class SliderWindow:
    def __init__(self, robot: Robot):
        import tkinter as tk
        self.tk = tk
        self.robot = robot
        self.root = tk.Tk()
        self.root.title("Franka Panda - DH forward kinematics")
        self.root.geometry("+40+40")
        self.alive = True
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        mono = ("Courier", 10)
        # --- DH table (read only) ---
        hdr = "  i |  a(i-1) [m] | alpha(i-1) [deg] |  d_i [m] | theta_i\n"
        rows = "".join(
            f" {i + 1:>2} | {a:>11.4f} | {np.degrees(al):>16.1f} | {d:>8.3f} | q{i + 1}"
            f"{'' if off == 0 else f' + {np.degrees(off):.1f}'}\n"
            for i, (a, al, d, off) in enumerate(DH_TABLE))
        rows += f" EE | flange d={FLANGE_D} m, hand yaw={np.degrees(HAND_YAW):.0f} deg, TCP +{TCP_OFFSET} m along z\n"
        tk.Label(self.root, text="Modified (Craig) DH table", font=("Helvetica", 11, "bold")).pack(anchor="w", padx=8, pady=(8, 0))
        tk.Label(self.root, text=hdr + rows, font=mono, justify="left").pack(anchor="w", padx=8)

        # --- sliders ---
        box = tk.LabelFrame(self.root, text="Joint variables theta_i  [deg]")
        box.pack(fill="x", padx=8, pady=6)
        self.vars = []
        for i in range(NJ):
            lo, hi = np.degrees(robot.limits[i])
            v = tk.DoubleVar(value=round(float(np.degrees(READY_POSE[i])), 1))
            self.vars.append(v)
            tk.Label(box, text=f"theta{i + 1}", width=7, font=mono).grid(row=i, column=0)
            tk.Scale(box, variable=v, from_=round(lo, 1), to=round(hi, 1), resolution=0.1,
                     orient="horizontal", length=380, showvalue=False).grid(row=i, column=1, padx=4)
            tk.Spinbox(box, textvariable=v, from_=round(lo, 1), to=round(hi, 1), increment=1.0,
                       width=8, format="%.1f").grid(row=i, column=2, padx=4)
            tk.Label(box, text=f"[{lo:.0f}, {hi:.0f}]", font=("Courier", 9), fg="gray40").grid(row=i, column=3)

        # --- buttons / options ---
        bar = tk.Frame(self.root)
        bar.pack(fill="x", padx=8)
        tk.Button(bar, text="Ready pose", command=self.set_ready).pack(side="left", padx=2)
        tk.Button(bar, text="All zeros (clipped)", command=self.set_zero).pack(side="left", padx=2)
        tk.Button(bar, text="Random", command=self.set_random).pack(side="left", padx=2)
        tk.Button(bar, text="Clear trail", command=lambda: self.trail.clear()).pack(side="left", padx=2)
        self.smooth = tk.BooleanVar(value=True)
        self.frames_on = tk.BooleanVar(value=True)
        self.trail_on = tk.BooleanVar(value=False)
        opts = tk.Frame(self.root)
        opts.pack(fill="x", padx=8, pady=2)
        tk.Checkbutton(opts, text="Smooth motion", variable=self.smooth).pack(side="left")
        tk.Checkbutton(opts, text="Show DH frames", variable=self.frames_on).pack(side="left")
        tk.Checkbutton(opts, text="Trail", variable=self.trail_on).pack(side="left")
        self.trail = deque(maxlen=300)

        # --- read-outs ---
        out = tk.LabelFrame(self.root, text="End effector (TCP), world frame")
        out.pack(fill="x", padx=8, pady=6)
        self.lbl_dh = tk.Label(out, font=mono, justify="left", anchor="w")
        self.lbl_mj = tk.Label(out, font=mono, justify="left", anchor="w")
        self.lbl_err = tk.Label(out, font=mono, justify="left", anchor="w")
        self.lbl_T = tk.Label(out, font=mono, justify="left", anchor="w")
        for w in (self.lbl_dh, self.lbl_mj, self.lbl_err, self.lbl_T):
            w.pack(anchor="w", padx=6)

    # ---- actions ----
    def _set(self, q_deg):
        for v, lim, x in zip(self.vars, self.robot.limits, q_deg):
            lo, hi = np.degrees(lim)
            v.set(round(float(np.clip(x, lo, hi)), 1))

    def set_ready(self):
        self._set(np.degrees(READY_POSE))

    def set_zero(self):
        self._set(np.zeros(NJ))

    def set_random(self):
        lo, hi = np.degrees(self.robot.limits).T
        self._set(np.random.uniform(lo, hi))

    def _close(self):
        self.alive = False

    def get_q(self, fallback):
        q = np.array(fallback, float)
        for i, v in enumerate(self.vars):
            try:
                q[i] = np.radians(np.clip(v.get(), *np.degrees(self.robot.limits[i])))
            except self.tk.TclError:          # half-typed text in a spinbox
                pass
        return q

    def update_readouts(self, T_dh, T_mj):
        p, r = T_dh[:3, 3], rot_to_rpy(T_dh[:3, :3])
        pm, rm = T_mj[:3, 3], rot_to_rpy(T_mj[:3, :3])
        self.lbl_dh.config(text=f"DH-FK   xyz = [{p[0]:+.4f} {p[1]:+.4f} {p[2]:+.4f}] m   rpy = [{r[0]:+7.2f} {r[1]:+7.2f} {r[2]:+7.2f}] deg")
        self.lbl_mj.config(text=f"MuJoCo  xyz = [{pm[0]:+.4f} {pm[1]:+.4f} {pm[2]:+.4f}] m   rpy = [{rm[0]:+7.2f} {rm[1]:+7.2f} {rm[2]:+7.2f}] deg")
        self.lbl_err.config(text=f"|pos error| DH vs MuJoCo = {np.linalg.norm(p - pm) * 1000:.4f} mm")
        self.lbl_T.config(text="T_world_ee =\n" + "\n".join(
            "  [" + " ".join(f"{x:+9.4f}" for x in row) + "]" for row in T_dh))

    def pump(self):
        try:
            self.root.update()
        except self.tk.TclError:
            self.alive = False


# --------------------------------------------------------------------------- #
#  6. Self-test and main                                                       #
# --------------------------------------------------------------------------- #
def self_check(robot: Robot, n=200):
    rng = np.random.default_rng(0)
    lo, hi = robot.limits.T
    worst_p = worst_r = 0.0
    for _ in range(n):
        q = rng.uniform(lo, hi)
        robot.set_q(q)
        T_dh, T_mj = dh_fk(q, robot.T_base), robot.ee_pose()
        worst_p = max(worst_p, np.linalg.norm(T_dh[:3, 3] - T_mj[:3, 3]))
        worst_r = max(worst_r, np.abs(T_dh[:3, :3] - T_mj[:3, :3]).max())
    # overlay drawing smoke test (no OpenGL needed)
    scn = mujoco.MjvScene(robot.model, maxgeom=2000)
    draw_overlay(scn, dh_frames(READY_POSE, robot.T_base), True, [np.zeros(3)] * 5)
    print(f"[check] {n} random poses: max position error = {worst_p * 1e3:.6f} mm, "
          f"max rotation-matrix error = {worst_r:.2e}, overlay geoms = {scn.ngeom}")
    return worst_p < 1e-3


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xml", help="path to franka/mjx_scene.xml (auto-detected by default)")
    ap.add_argument("--check", action="store_true", help="headless DH-vs-MuJoCo self test, then exit")
    ap.add_argument("--speed", type=float, default=120.0, help="max joint speed for smooth motion [deg/s]")
    args = ap.parse_args()

    xml = find_xml(args.xml)
    print(f"[info] loading {xml}")
    robot = Robot(xml)

    if args.check:
        sys.exit(0 if self_check(robot) else 1)

    try:
        win = SliderWindow(robot)
    except Exception as e:                                   # no tkinter / no display
        sys.exit(f"[error] cannot open the slider window ({e}).\n"
                 "        Linux: sudo apt install python3-tk")

    q_cur = READY_POSE.copy()
    robot.set_q(q_cur)
    max_step = np.radians(args.speed)
    last = time.perf_counter()

    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        viewer.cam.lookat[:] = [0.3, 0.0, 0.5]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.0, 135, -20
        while viewer.is_running() and win.alive:
            now = time.perf_counter()
            dt, last = min(now - last, 0.1), now

            q_tgt = win.get_q(q_cur)
            if win.smooth.get():
                q_cur = q_cur + np.clip(q_tgt - q_cur, -max_step * dt, max_step * dt)
            else:
                q_cur = q_tgt

            robot.set_q(q_cur)                      # MuJoCo robot -> joint values
            frames = dh_frames(q_cur, robot.T_base)  # DH forward kinematics
            T_dh, T_mj = frames[-1], robot.ee_pose()

            if win.trail_on.get():
                if not win.trail or np.linalg.norm(win.trail[-1] - T_dh[:3, 3]) > 0.003:
                    win.trail.append(T_dh[:3, 3].copy())
            with viewer.lock():
                draw_overlay(viewer.user_scn, frames, win.frames_on.get(),
                             win.trail if win.trail_on.get() else [])
            viewer.sync()

            win.update_readouts(T_dh, T_mj)
            win.pump()
            time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))
        viewer.close()
    try:
        win.root.destroy()
    except Exception:
        pass


if __name__ == "__main__":
    main()