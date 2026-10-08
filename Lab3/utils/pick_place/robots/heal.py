"""
HEAL 6-DoF arm + Robotiq 2F-85 (torque-controlled motors).
All HEAL-specific numbers and model construction live here.
"""
import os

import mujoco
import numpy as np

NAME = "heal"
DESC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "robot_descriptions"))
ARM_XML = os.path.join(DESC_DIR, "single_arm_heal_effort_actuation_rs_mj.xml")
GRIP_XML = os.path.join(DESC_DIR, "robotiq_2f85_v4", "2f85.xml")

TCP_OFFSET = 0.14   # m along flange z = centre of the four finger pads (measured)
ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
ARM_ACTUATORS = ["turret", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3"]
GRIPPER_ACTUATOR = "gripper/fingers_actuator"
GRIPPER_BODY_PREFIXES = ("gripper/",)
ARM_BODY_PREFIXES = ("link_", "end_effector")
HOME_SEED = np.zeros(6)
GRIPPER_OPEN_QPOS = {}

CONFIG = dict(
    # Table top is the plane z = height. See README section 5 for the rationale.
    TABLE=dict(center_xy=(0.38, 0.0), length=0.36, width=0.80, height=0.20,
               thickness=0.04, rgba=(0.55, 0.40, 0.28, 1.0)),
    TRAY=dict(center_xy=(0.32, -0.28), inner_size=(0.14, 0.14), wall_height=0.05,
              wall_thickness=0.008, floor_thickness=0.006, rgba=(0.15, 0.45, 0.85, 1.0)),
    # condim=6 adds torsional+rolling friction: without it the cube pivots about the
    # pad axis and squirts out of the pinch (README, failure analysis).
    CUBE=dict(half_size=0.02, mass=0.05, friction=(1.2, 0.05, 0.001), condim=6,
              rgba=(0.85, 0.15, 0.15, 1.0)),
    SPAWN=dict(x_range=(0.25, 0.50), y_range=(-0.12, 0.34), r_range=(0.25, 0.50),
               yaw_range=(-np.pi, np.pi), tray_keepout=0.06),
    PLAN=dict(pregrasp_height=0.12,
              grasp_depth=0.014,          # TCP above cube centre (2F-85 tips swing down when closing)
              lift_height=0.15, tray_clearance=0.04, place_release_height=0.015,
              seg_speed=0.20, min_seg_time=0.6, settle_time=0.35, grip_time=0.6, ik_rate=50.0),
    CTRL=dict(mode="torque",
              kp=np.array([900, 900, 700, 300, 300, 150.0]),
              kd=np.array([60, 60, 45, 18, 18, 8.0]),
              gripper_open=0.0, gripper_close=255.0,
              gripper_force=25.0,         # menagerie cap of 5 N is too weak to hold the cube
              max_joint_vel=2.0, sim_timestep=0.002),
    CHECK=dict(lift_min_rise=0.03, slip_drop=0.03, tracking_tol=0.03,
               ik_pos_tol=2e-3, ik_rot_tol=2e-2, ik_max_iters=150),
    HOME_TCP=dict(pos=(0.28, 0.0, 0.45), yaw=0.0),
    CAMERA_POS=(1.45, 0.75, 0.95),
)


def build_arm_spec(C):
    """HEAL + 2F-85 attached at 'right_center' (as in the starter code) + 'tcp' site."""
    arm = mujoco.MjSpec.from_file(ARM_XML)
    grip = mujoco.MjSpec.from_file(GRIP_XML)
    arm.attach(grip, prefix="gripper/", site=arm.site("right_center"))
    tcp = arm.body("gripper/base").add_site()
    tcp.name, tcp.pos, tcp.size, tcp.rgba, tcp.group = "tcp", [0, 0, TCP_OFFSET], [0.006, 0, 0], [0, 1, 0, 0.8], 2

    act = arm.actuator(GRIPPER_ACTUATOR)
    act.forcerange = [-C.CTRL["gripper_force"], C.CTRL["gripper_force"]]

    # Collision bitmasks: bit 0 (1) = world/robot/cube/pads, bit 3 (8) = finger mesh hulls.
    # Convex hulls of the 2F-85 linkages are much fatter than the real fingers and wedge the
    # cube out, so the cube only touches the pads; hulls still hit table + tray (1|8).
    for g in arm.geoms:
        if g.name.startswith("gripper/") or (g.parent and g.parent.name.startswith("gripper/")):
            if "pad" not in g.name and (g.contype or g.conaffinity):
                g.contype, g.conaffinity = 8, 8
    return arm
