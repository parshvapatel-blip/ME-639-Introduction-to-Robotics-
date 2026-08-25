import pathlib
import time
import numpy as np
import mujoco
import glfw

MODEL_PATH = pathlib.Path("skydio_x2/scene.xml")

# ---------------- Flight tuning ----------------
# Real drone (mode-2 style) mapping:
#   Up / Down arrows    -> pitch forward / backward (move front/back)
#   Left / Right arrows -> yaw rotate left / right   (rotate about vertical axis)
#   W / S                -> throttle up / down        (climb/descend)
PITCH_MAX = 0.10          # max pitch angle (rad) ~5.5 deg -> gentle forward/back speed
YAW_RATE_MAX = 0.8        # max yaw rate (rad/s)
THRUST_RATE = 2.5         # N/s change while holding W/S (slower climb/descend)
THRUST_MAX_OFFSET = 25.0  # clamp on total added thrust offset

ATTITUDE_RAMP_TIME = 0.25    # seconds to reach full stick deflection
ATTITUDE_RETURN_TIME = 0.35  # seconds to self-level back to 0 on release
ROLL_RETURN_RATE = 0.20 / ATTITUDE_RETURN_TIME  # roll always self-levels, no key drives it

# ---------------- Key state ----------------
# True while physically held down, updated by GLFW key callback (press/release, not just press)
keys_down = {
    "up": False, "down": False, "left": False, "right": False,
    "w": False, "s": False,
}

KEY_MAP = {
    glfw.KEY_UP: "up",
    glfw.KEY_DOWN: "down",
    glfw.KEY_LEFT: "left",
    glfw.KEY_RIGHT: "right",
    glfw.KEY_W: "w",
    glfw.KEY_S: "s",
}

reset_requested = False


def keyboard_callback(window, key, scancode, action, mods):
    global reset_requested
    if key == glfw.KEY_X and action == glfw.PRESS:
        reset_requested = True
        return
    if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
        glfw.set_window_should_close(window, True)
        return
    name = KEY_MAP.get(key)
    if name is None:
        return
    if action == glfw.PRESS:
        keys_down[name] = True
    elif action == glfw.RELEASE:
        keys_down[name] = False


# ---------------- Simple orbit camera (mouse) ----------------
cam_state = {"button_left": False, "button_right": False, "last_x": 0, "last_y": 0}


def mouse_button_callback(window, button, action, mods):
    if button == glfw.MOUSE_BUTTON_LEFT:
        cam_state["button_left"] = (action == glfw.PRESS)
    elif button == glfw.MOUSE_BUTTON_RIGHT:
        cam_state["button_right"] = (action == glfw.PRESS)
    cam_state["last_x"], cam_state["last_y"] = glfw.get_cursor_pos(window)


def cursor_pos_callback(window, xpos, ypos):
    dx = xpos - cam_state["last_x"]
    dy = ypos - cam_state["last_y"]
    cam_state["last_x"], cam_state["last_y"] = xpos, ypos
    if cam_state["button_left"]:
        cam.azimuth += dx * 0.3
        cam.elevation = np.clip(cam.elevation - dy * 0.3, -89.9, 89.9)
    elif cam_state["button_right"]:
        cam.distance = max(0.1, cam.distance * (1.0 + dy * 0.01))


def scroll_callback(window, xoffset, yoffset):
    cam.distance = max(0.1, cam.distance * (1.0 - yoffset * 0.08))


def add_axis_frame(scn, origin, R, scale=0.75, width=0.025):
    colors = [
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
    ]
    for i in range(3):
        if scn.ngeom >= scn.maxgeom:
            break
        axis_vec = R[:, i]
        p2 = origin + axis_vec * scale
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            size=[width, width, width],
            pos=np.zeros(3),
            mat=np.eye(3).flatten(),
            rgba=colors[i],
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, origin, p2)
        scn.ngeom += 1


def approach(current, target, rate, dt):
    """Move current toward target at most `rate` units/sec."""
    diff = target - current
    step = rate * dt
    if abs(diff) <= step:
        return target
    return current + step * np.sign(diff)


def main():
    global cam

    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model file not found at: {MODEL_PATH.resolve()}")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    base_body_id = 1 if model.nbody > 1 else 0
    total_mass = model.body(base_body_id).subtreemass[0]
    num_actuators = model.nu
    hover_thrust = (total_mass * 9.81) / num_actuators

    mixer = np.zeros((num_actuators, 4))
    for i in range(num_actuators):
        site_id = model.actuator_trnid[i, 0]
        site_pos = model.site_pos[site_id]
        yaw_gear = model.actuator_gear[i, 5]
        mixer[i, 0] = 1.0
        mixer[i, 1] = -np.sign(site_pos[0]) * 0.5
        mixer[i, 2] = np.sign(site_pos[1]) * 0.5
        mixer[i, 3] = np.sign(yaw_gear) * 1.0

    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.5
    for _ in range(100):
        data.ctrl[:num_actuators] = hover_thrust
        mujoco.mj_step(model, data)

    kp_att, kd_att = 2.5, 0.35
    kp_yaw = 0.8   # proper rate-tracking gain (was feedforward-only before, essentially dead)

    # ---------------- GLFW / render setup ----------------
    if not glfw.init():
        raise RuntimeError("Could not initialize GLFW")

    window = glfw.create_window(1200, 800, "Skydio X2 - Drone Control", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Could not create GLFW window")

    glfw.make_context_current(window)
    glfw.swap_interval(1)

    glfw.set_key_callback(window, keyboard_callback)
    glfw.set_mouse_button_callback(window, mouse_button_callback)
    glfw.set_cursor_pos_callback(window, cursor_pos_callback)
    glfw.set_scroll_callback(window, scroll_callback)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance = 4.0
    cam.azimuth = 135
    cam.elevation = -20
    cam.lookat = np.array([0.0, 0.0, 0.5])

    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    opt.frame = mujoco.mjtFrame.mjFRAME_NONE

    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    # smoothed / self-centering command state
    target_pitch = 0.0
    target_roll = 0.0
    target_yaw_rate = 0.0
    thrust_offset = 0.0

    start_time = time.time()
    last_frame_time = start_time

    global reset_requested

    while not glfw.window_should_close(window):
        now = time.time()
        dt = max(1e-4, now - last_frame_time)
        last_frame_time = now

        if reset_requested:
            target_pitch = 0.0
            target_roll = 0.0
            target_yaw_rate = 0.0
            thrust_offset = 0.0
            reset_requested = False

        # ---- self-centering stick logic (real transmitter feel) ----
        # Up/Down arrows: pitch forward/back. Left/Right arrows: yaw rotate.
        pitch_goal = -PITCH_MAX if keys_down["up"] else (PITCH_MAX if keys_down["down"] else 0.0)
        yaw_goal = YAW_RATE_MAX if keys_down["left"] else (-YAW_RATE_MAX if keys_down["right"] else 0.0)
        # roll is never driven by a key -> controller just holds it level (0)
        roll_goal = 0.0

        pitch_rate = PITCH_MAX / ATTITUDE_RAMP_TIME if pitch_goal != 0 else PITCH_MAX / ATTITUDE_RETURN_TIME
        yaw_rate_rate = YAW_RATE_MAX / ATTITUDE_RAMP_TIME if yaw_goal != 0 else YAW_RATE_MAX / ATTITUDE_RETURN_TIME
        roll_rate = ROLL_RETURN_RATE

        target_pitch = approach(target_pitch, pitch_goal, pitch_rate, dt)
        target_yaw_rate = approach(target_yaw_rate, yaw_goal, yaw_rate_rate, dt)
        target_roll = approach(target_roll, roll_goal, roll_rate, dt)

        # throttle: direct hold-to-climb/descend, holds value on release (like real throttle)
        if keys_down["w"]:
            thrust_offset += THRUST_RATE * dt
        elif keys_down["s"]:
            thrust_offset -= THRUST_RATE * dt
        thrust_offset = float(np.clip(thrust_offset, -THRUST_MAX_OFFSET, THRUST_MAX_OFFSET))

        # ---- physics step(s) to catch up to wall clock ----
        R = data.xmat[base_body_id].reshape(3, 3).copy()
        pos = data.xpos[base_body_id].copy()
        omega = data.qvel[3:6]

        roll_curr = np.arctan2(R[2, 1], R[2, 2])
        pitch_curr = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))

        u_pitch = np.clip(-kp_att * (pitch_curr - target_pitch) - kd_att * omega[1], -0.5, 0.5)
        u_roll = np.clip(-kp_att * (roll_curr - target_roll) - kd_att * omega[0], -0.5, 0.5)
        u_yaw = np.clip(kp_yaw * (target_yaw_rate - omega[2]), -1.0, 1.0)

        total_hover = hover_thrust + (thrust_offset / num_actuators)
        cmd_vector = np.array([total_hover, u_pitch, u_roll, u_yaw])
        data.ctrl[:num_actuators] = np.clip(mixer @ cmd_vector, 0.0, 12.0)

        sim_target = now - start_time
        while data.time < sim_target:
            mujoco.mj_step(model, data)

        # ---- overlay geoms ----
        R_clean = np.where(np.abs(R) < 1e-4, 0.0, R)

        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)

        add_axis_frame(scene, np.array([0.0, 0.0, 0.01]), np.eye(3), scale=0.25, width=0.012)
        body_frame_origin = pos + R[:, 2] * 0.15
        add_axis_frame(scene, body_frame_origin, R, scale=0.75, width=0.025)

        lines = [
            "Skydio X2 Rotation Matrix R:",
            f"| {R_clean[0,0]: 6.2f}  {R_clean[0,1]: 6.2f}  {R_clean[0,2]: 6.2f} |",
            f"| {R_clean[1,0]: 6.2f}  {R_clean[1,1]: 6.2f}  {R_clean[1,2]: 6.2f} |",
            f"| {R_clean[2,0]: 6.2f}  {R_clean[2,1]: 6.2f}  {R_clean[2,2]: 6.2f} |",
            f"Alt: {pos[2]:.2f}m | Pitch: {pitch_curr:+.2f} | Yaw rate: {target_yaw_rate:+.2f} | Thr: {thrust_offset:+.1f}N",
            "Up/Down: forward/back   Left/Right: rotate   W/S: throttle   X: reset",
        ]
        base_z = pos[2] + 0.90
        for idx, line_text in enumerate(lines):
            if scene.ngeom >= scene.maxgeom:
                break
            geom_text = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom_text,
                type=mujoco.mjtGeom.mjGEOM_LABEL,
                size=np.zeros(3),
                pos=[pos[0], pos[1], base_z - (idx * 0.07)],
                mat=np.eye(3).flatten(),
                rgba=[0.1, 0.9, 1.0, 1.0],
            )
            geom_text.label = line_text
            scene.ngeom += 1

        width, height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjr_render(viewport, scene, context)

        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()


if __name__ == "__main__":
    main()