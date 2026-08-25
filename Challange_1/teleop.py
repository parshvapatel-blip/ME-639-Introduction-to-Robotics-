import pathlib
import time
import numpy as np
import mujoco
import mujoco.viewer

MODEL_PATH = pathlib.Path("turtlebot_mjcf.xml")

# Kinematic Parameters
WHEEL_RADIUS = 0.033  # meters
TRACK_WIDTH = 0.287   # meters

# Velocity Command Limits
CMD_LIN_VEL = 0.35    # m/s
CMD_ANG_VEL = 1.50    # rad/s

target_lin_vel = 0.0
target_ang_vel = 0.0

# Keycodes (GLFW / IJKL + Arrow Keys)
KEY_UP = 265
KEY_DOWN = 264
KEY_LEFT = 263
KEY_RIGHT = 262

def key_callback(keycode: int):
    global target_lin_vel, target_ang_vel

    char = chr(keycode).lower() if 0 <= keycode < 128 else ""

    # FORWARD
    if keycode == KEY_UP or char in ("w", "i"):
        target_lin_vel = CMD_LIN_VEL
        target_ang_vel = 0.0

    # BACKWARD
    elif keycode == KEY_DOWN or char in ("s", "k"):
        target_lin_vel = -CMD_LIN_VEL
        target_ang_vel = 0.0

    # TURN LEFT
    elif keycode == KEY_LEFT or char in ("a", "j"):
        target_ang_vel = CMD_ANG_VEL

    # TURN RIGHT
    elif keycode == KEY_RIGHT or char in ("d", "l"):
        target_ang_vel = -CMD_ANG_VEL

    # FULL STOP
    elif keycode == 32 or char in ("x", "space"):
        target_lin_vel = 0.0
        target_ang_vel = 0.0

def main():
    global target_lin_vel, target_ang_vel

    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model file not found at: {MODEL_PATH.resolve()}")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    base_body_id = model.body("base_link").id

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        start_time = time.time()

        print("\n--- IN-VIEWER KINEMATICS OVERLAY CONTROLLER ---")
        print("Use W/A/S/D, I/J/K/L, or Arrow Keys to drive.")
        print("Press SPACE to stop | ESC to exit.")
        print("------------------------------------------------\n")

        while viewer.is_running():
            # Differential Drive Kinematics
            v_left = -target_lin_vel - (target_ang_vel * TRACK_WIDTH / 2.0)
            v_right = -target_lin_vel + (target_ang_vel * TRACK_WIDTH / 2.0)

            data.ctrl[0] = v_left / WHEEL_RADIUS
            data.ctrl[1] = v_right / WHEEL_RADIUS

            # Physics stepping
            sim_target = time.time() - start_time
            while data.time < sim_target:
                mujoco.mj_step(model, data)

            # Rotation matrix and position extraction
            R = data.xmat[base_body_id].reshape(3, 3)
            pos = data.xpos[base_body_id]

            # Clear existing 3D text geoms
            viewer.user_scn.ngeom = 0

            R_clean = np.where(np.abs(R) < 1e-4, 0.0, R)

            lines = [
                "Rotation Matrix R:",
                f"| {R_clean[0,0]: 6.2f}  {R_clean[0,1]: 6.2f}  {R_clean[0,2]: 6.2f} |",
                f"| {R_clean[1,0]: 6.2f}  {R_clean[1,1]: 6.2f}  {R_clean[1,2]: 6.2f} |",
                f"| {R_clean[2,0]: 6.2f}  {R_clean[2,1]: 6.2f}  {R_clean[2,2]: 6.2f} |",
                f"v: {target_lin_vel: 5.2f} m/s | w: {target_ang_vel: 5.2f} rad/s",
            ]

            base_z = pos[2] + 0.55
            for idx, line_text in enumerate(lines):
                geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                mujoco.mjv_initGeom(
                    geom,
                    type=mujoco.mjtGeom.mjGEOM_LABEL,
                    size=np.zeros(3),
                    pos=[pos[0], pos[1], base_z - (idx * 0.06)],
                    mat=np.eye(3).flatten(),
                    rgba=[0.2, 1.0, 0.4, 1.0],  # Matrix green text
                )
                geom.label = line_text
                viewer.user_scn.ngeom += 1

            viewer.sync()
            time.sleep(0.016)

if __name__ == "__main__":
    main()