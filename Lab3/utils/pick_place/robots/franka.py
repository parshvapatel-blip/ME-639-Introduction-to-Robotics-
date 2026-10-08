"""
Franka Emika Panda 7-DoF + Franka Hand (MuJoCo Menagerie model, position servos).
Model: robot_descriptions/franka_emika_panda/panda.xml
(https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda)
"""
import os

import mujoco
import numpy as np

NAME = "franka"
DESC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "robot_descriptions"))
ARM_XML = os.path.join(DESC_DIR, "franka_emika_panda", "panda.xml")

# hand origin -> centre of the fingertip pads (finger body at 0.0584 + pad centre ~0.045)
TCP_OFFSET = 0.1034
ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
ARM_ACTUATORS = [f"actuator{i}" for i in range(1, 8)]
GRIPPER_ACTUATOR = "actuator8"
GRIPPER_BODY_PREFIXES = ("hand", "left_finger", "right_finger")
ARM_BODY_PREFIXES = ("link",)
HOME_SEED = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])  # menagerie 'home' key
GRIPPER_OPEN_QPOS = {"finger_joint1": 0.04, "finger_joint2": 0.04}

CONFIG = dict(
    # Panda base at the origin on the floor; table top at z = height.
    # Values chosen from a top-down reachability sweep (README, Franka section) -
    # replace with your own workspace analysis.
    TABLE=dict(center_xy=(0.50, 0.0), length=0.50, width=0.90, height=0.10,
               thickness=0.04, rgba=(0.55, 0.40, 0.28, 1.0)),
    TRAY=dict(center_xy=(0.45, -0.30), inner_size=(0.14, 0.14), wall_height=0.05,
              wall_thickness=0.008, floor_thickness=0.006, rgba=(0.15, 0.45, 0.85, 1.0)),
    CUBE=dict(half_size=0.02, mass=0.05, friction=(1.2, 0.05, 0.001), condim=6,
              rgba=(0.85, 0.15, 0.15, 1.0)),
    SPAWN=dict(x_range=(0.33, 0.68), y_range=(-0.15, 0.38), r_range=(0.33, 0.70),
               yaw_range=(-np.pi, np.pi), tray_keepout=0.06),
    PLAN=dict(pregrasp_height=0.12,
              grasp_depth=0.0,            # parallel jaw: no fingertip arc, grasp at cube centre
              lift_height=0.15, tray_clearance=0.04, place_release_height=0.015,
              seg_speed=0.20, min_seg_time=0.6, settle_time=0.35, grip_time=0.6, ik_rate=50.0),
    CTRL=dict(mode="position",            # menagerie actuators are joint position servos
              gripper_open=255.0, gripper_close=0.0,   # Franka hand: 255 = open, 0 = closed
              max_joint_vel=2.0, sim_timestep=0.002),
    CHECK=dict(lift_min_rise=0.03, slip_drop=0.03, tracking_tol=0.03,
               ik_pos_tol=2e-3, ik_rot_tol=2e-2, ik_max_iters=150),
    HOME_TCP=dict(pos=(0.35, 0.0, 0.45), yaw=0.0),
    CAMERA_POS=(1.85, 1.05, 1.35),    # overview camera (video / failure snapshots)
)


def build_arm_spec(C):
    arm = mujoco.MjSpec.from_file(ARM_XML)
    hand = arm.body("hand")
    tcp = hand.add_site()
    tcp.name, tcp.pos, tcp.size, tcp.rgba, tcp.group = "tcp", [0, 0, TCP_OFFSET], [0.006, 0, 0], [0, 1, 0, 0.8], 2
    # Rotate +90 deg about z so site-x = hand-y = finger closing axis (same convention as HEAL).
    tcp.quat = [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]

    # Name the fingertip pad boxes "<finger>_padN" and use the same collision masks as HEAL:
    # pads (1) touch the cube; finger mesh hulls (8) only touch table + tray.
    for body in ("left_finger", "right_finger"):
        k = 0
        for g in arm.body(body).geoms:
            if g.classname.name.startswith("fingertip_pad"):
                k += 1
                g.name = f"{body}_pad{k}"
                g.friction = [1.5, 0.05, 0.001]
                g.condim = 6
            elif g.contype or g.conaffinity:
                g.contype, g.conaffinity = 8, 8
    return arm
