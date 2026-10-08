"""
Pick-and-place episode runner, shared by all three IK methods.

Per episode
-----------
 1. reset(seed)     : arm to HOME, gripper open, cube at random (x, y, yaw)
 2. plan()          : solve IK for the key waypoints (pre-grasp, grasp, lift,
                      above-tray, place) -> early 'ik_infeasible' / 'joint_limit'
 3. execute()       : each segment = straight-line TCP interpolation (pos lerp,
                      quat slerp); the IK solver is called at PLAN['ik_rate'] Hz
                      on the interpolated pose (warm-started), a PD + gravity
                      + damping feed-forward torque controller tracks q_cmd.
 4. monitor (every physics step): forbidden contacts, TCP tracking error,
                      joint-velocity limit, cube slip; min clearances via
                      mj_geomDistance.
 5. verify          : cube resting inside the tray -> success.

Cube <-> tray collision avoidance ("smart" part)
-----------------------------------------------
 * transit altitude is computed from the *measured* cube-bottom offset below the
   TCP after lifting, so the cube bottom clears the tray wall top by
   PLAN['tray_clearance'] (not just the TCP);
 * the cube is carried at that altitude to a point vertically above the tray
   centre, then lowered straight down -> the swept volume never intersects a wall;
 * place yaw is snapped to the tray axes (multiple of pi/2, the one closest to the
   current yaw) so the cube footprint is aligned with the walls; the footprint is
   checked against the inner tray area (with margin) before descending;
 * the minimum cube<->wall distance is measured throughout and logged.
"""
import time

import mujoco
import numpy as np

from . import config as C
from .ik_solvers import make_solver, mat_to_quat, quat_mul, orientation_error
from .scene import build_model, arm_indices


def yaw_quat(yaw):
    """Top-down TCP orientation (tool z pointing down, closing axis at `yaw`)."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])
    return mat_to_quat(R)


def quat_slerp(q0, q1, t):
    q0, q1 = np.asarray(q0), np.asarray(q1)
    d = np.dot(q0, q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(d)
    return (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class EpisodeFailure(Exception):
    def __init__(self, reason, phase, detail=""):
        super().__init__(f"{reason} @ {phase}: {detail}")
        self.reason, self.phase, self.detail = reason, phase, detail


class PickPlaceEnv:
    def __init__(self, method="mink", solver_kwargs=None, render=None, viewer=None,
                 profile="nominal", smart_tray=True, on_failure=None):
        self.method = method
        self.spawn = C.SPAWN_PROFILES[profile]
        self.smart_tray = smart_tray
        self.on_failure = on_failure   # optional callable(env, EpisodeFailure) -> snapshot
        self.model, self.spec = build_model()
        m = self.model
        self.data = mujoco.MjData(m)
        self.qidx, self.vidx, self.aidx = arm_indices(m)
        R = C.ROBOT_MOD
        self.robot = C.ROBOT
        self.grip_act = m.actuator(R.GRIPPER_ACTUATOR).id
        self.tcp_id = m.site("tcp").id
        self.cube_bid = m.body("cube").id
        self.cube_gid = m.geom("cube_geom").id
        self.cube_qadr = m.jnt_qposadr[m.joint("cube_free").id]
        self.cube_vadr = m.jnt_dofadr[m.joint("cube_free").id]
        self.dt = m.opt.timestep
        self.render = render      # optional callable(env) every frame (video)
        self.viewer = viewer      # optional passive viewer

        kw = dict(max_iters=C.CHECK["ik_max_iters"], pos_tol=C.CHECK["ik_pos_tol"],
                  rot_tol=C.CHECK["ik_rot_tol"], dt=1.0 / C.PLAN["ik_rate"],
                  max_joint_vel=C.CTRL["max_joint_vel"])
        kw.update(solver_kwargs or {})
        self.ik = make_solver(method, m, "tcp", self.qidx, self.vidx, **kw)

        # --- geom groups for collision checks -------------------------------- #
        name = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        bname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
        self.table_g = {m.geom("table_top").id}
        self.tray_floor_g = {m.geom("tray_floor").id}
        self.tray_wall_g = {m.geom(f"tray_wall_{s}").id for s in ("px", "nx", "py", "ny")}
        self.tray_g = self.tray_floor_g | self.tray_wall_g
        self.pad_g = {g for g in range(m.ngeom) if "pad" in name(g)}
        self.gripper_g = {g for g in range(m.ngeom) if bname(g).startswith(R.GRIPPER_BODY_PREFIXES)}
        self.arm_g = {g for g in range(m.ngeom)
                      if bname(g).startswith(R.ARM_BODY_PREFIXES) and m.geom_contype[g]}
        self.grip_open_q = {m.jnt_qposadr[m.joint(j).id]: v for j, v in R.GRIPPER_OPEN_QPOS.items()}
        self.robot_g = self.gripper_g | self.arm_g
        # convex collision geoms used for clearance (skip visual-only)
        self.grip_coll_g = [g for g in self.gripper_g if m.geom_contype[g] or m.geom_conaffinity[g]]

        self.home_q = self._solve_home()

    def _solve_home(self):
        from .ik_solvers import DLSIK
        ik = DLSIK(self.model, "tcp", self.qidx, self.vidx, max_iters=500,
                   pos_tol=1e-4, rot_tol=1e-3, null_gain=0.0)
        mujoco.mj_forward(self.model, self.data)
        ik.set_seed_qpos(self.data.qpos)
        r = ik.solve(C.ROBOT_MOD.HOME_SEED.copy(), np.array(C.HOME_TCP["pos"]),
                     yaw_quat(C.HOME_TCP["yaw"]))
        assert r.converged, "HOME_TCP is not reachable - edit config.HOME_TCP"
        return r.q

    # ------------------------------------------------------------------ reset #
    def sample_cube(self, rng):
        S = self.spawn
        T = C.TRAY
        half_out = np.array(T["inner_size"]) / 2 + T["wall_thickness"] + S["tray_keepout"] \
            + C.CUBE["half_size"] * np.sqrt(2)
        for _ in range(1000):
            x = rng.uniform(*S["x_range"])
            y = rng.uniform(*S["y_range"])
            yaw = rng.uniform(*S["yaw_range"])
            r = np.hypot(x, y)
            if not (S["r_range"][0] <= r <= S["r_range"][1]):
                continue
            if abs(x - T["center_xy"][0]) < half_out[0] and abs(y - T["center_xy"][1]) < half_out[1]:
                continue
            return float(x), float(y), float(yaw)
        raise RuntimeError("could not sample a cube pose - check SPAWN ranges")

    def reset(self, seed):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        rng = np.random.default_rng(seed)
        x, y, yaw = self.sample_cube(rng)
        self.cube_init = dict(x=x, y=y, yaw=yaw)
        d.qpos[self.qidx] = self.home_q
        for adr, v in self.grip_open_q.items():
            d.qpos[adr] = v
        zc = C.TABLE["height"] + C.CUBE["half_size"] + 5e-4
        d.qpos[self.cube_qadr:self.cube_qadr + 3] = [x, y, zc]
        d.qpos[self.cube_qadr + 3:self.cube_qadr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        mujoco.mj_forward(m, d)
        self.q_cmd = self.home_q.copy()
        self.qd_cmd = np.zeros(len(self.qidx))
        self.grip_cmd = C.CTRL["gripper_open"]
        self.phase = "reset"
        self.allow = set()       # phase-dependent allowed contact categories
        self.ik_log = []         # per IK call: (iters, time, converged)
        self.metrics = dict(
            min_clear_cube_tray=np.inf, min_clear_grip_table=np.inf, min_clear_grip_tray=np.inf,
            max_track_err=0.0, joint_vel_violations=0, joint_limit_violations=0,
            ik_joint_limit_violations=0, ik_vel_violations=0, ik_nonconverged=0,
        )
        self.contacts_seen = []
        self._t_clear = 0
        self.ik.set_seed_qpos(d.qpos)
        self.simulate(0.3)       # let the cube settle
        return self.cube_init

    # ------------------------------------------------------------- control ---- #
    def _apply_control(self):
        m, d = self.model, self.data
        d.ctrl[self.grip_act] = self.grip_cmd
        if C.CTRL["mode"] == "position":           # Franka: joint position servos
            d.ctrl[self.aidx] = self.q_cmd
            return
        q = d.qpos[self.qidx]
        qd = d.qvel[self.vidx]
        kp, kd = C.CTRL["kp"], C.CTRL["kd"]
        tau = (d.qfrc_bias[self.vidx]
               + kp * (self.q_cmd - q)
               + kd * (self.qd_cmd - qd)
               + m.dof_damping[self.vidx] * self.qd_cmd)   # cancel joint viscous damping
        d.ctrl[self.aidx] = tau
        d.ctrl[self.grip_act] = self.grip_cmd

    def tcp_pose(self, d=None):
        d = d or self.data
        return d.site_xpos[self.tcp_id].copy(), mat_to_quat(d.site_xmat[self.tcp_id])

    def cube_pos(self):
        return self.data.xpos[self.cube_bid].copy()

    def simulate(self, duration, monitor=True):
        n = max(1, int(round(duration / self.dt)))
        for _ in range(n):
            self._apply_control()
            mujoco.mj_step(self.model, self.data)
            if monitor and self.phase != "reset":
                self._monitor()
            if self.render is not None:
                self.render(self)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()

    # ------------------------------------------------------------- monitor ---- #
    def _monitor(self):
        m, d = self.model, self.data
        # joint limits / velocities (sim state)
        q = d.qpos[self.qidx]
        jid = [m.dof_jntid[v] for v in self.vidx]
        if np.any(q < m.jnt_range[jid, 0] - 1e-3) or np.any(q > m.jnt_range[jid, 1] + 1e-3):
            self.metrics["joint_limit_violations"] += 1
        if np.any(np.abs(d.qvel[self.vidx]) > C.CTRL["max_joint_vel"] * 1.05):
            self.metrics["joint_vel_violations"] += 1

        # forbidden contacts
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            pair = {g1, g2}
            if pair & self.robot_g and pair & self.table_g:
                self._fail("collision_table", f"{self._gname(g1)}-{self._gname(g2)}")
            if pair & self.robot_g and pair & self.tray_g:
                self._fail("collision_tray", f"{self._gname(g1)}-{self._gname(g2)}")
            if self.cube_gid in pair and pair & self.tray_wall_g and "cube_tray_wall" not in self.allow:
                self._fail("collision_cube_tray", f"{self._gname(g1)}-{self._gname(g2)}")
            if self.cube_gid in pair and pair & self.arm_g:
                self._fail("collision_arm_cube", f"{self._gname(g1)}-{self._gname(g2)}")

        # clearances (every 10 steps - mj_geomDistance is not free)
        self._t_clear += 1
        if self._t_clear % 10 == 0:
            ft = np.zeros(6)
            for w in self.tray_wall_g:
                dist = mujoco.mj_geomDistance(m, d, self.cube_gid, w, 0.3, ft)
                self.metrics["min_clear_cube_tray"] = min(self.metrics["min_clear_cube_tray"], dist)
            for g in self.pad_g:
                dist = mujoco.mj_geomDistance(m, d, g, next(iter(self.table_g)), 0.3, ft)
                self.metrics["min_clear_grip_table"] = min(self.metrics["min_clear_grip_table"], dist)
                for w in self.tray_g:
                    dist = mujoco.mj_geomDistance(m, d, g, w, 0.3, ft)
                    self.metrics["min_clear_grip_tray"] = min(self.metrics["min_clear_grip_tray"], dist)

        # slip during carry
        if "carrying" in self.allow:
            tcp, _ = self.tcp_pose()
            drop = (tcp[2] - self.cube_pos()[2]) - self.carry_offset
            if drop > C.CHECK["slip_drop"]:
                self._fail("grasp_slip", f"cube dropped {drop*100:.1f} cm below hold")

    def _gname(self, g):
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"

    def _fail(self, reason, detail=""):
        raise EpisodeFailure(reason, self.phase, detail)

    # --------------------------------------------------------------- motion ---- #
    def _ik(self, q_init, pos, quat):
        r = self.ik.solve(q_init, pos, quat)
        self.ik_log.append((r.iterations, r.solve_time, r.converged))
        self.metrics["ik_joint_limit_violations"] += int(r.joint_limit_violation)
        self.metrics["ik_vel_violations"] += int(r.velocity_violation)
        self.metrics["ik_nonconverged"] += int(not r.converged)
        return r

    def move_to(self, pos, quat, phase, speed=None):
        """Cartesian straight-line segment from the current *commanded* TCP pose."""
        self.phase = phase
        p0, q0 = self.ik._fk(self.q_cmd)
        dist = np.linalg.norm(pos - p0)
        ang = np.linalg.norm(orientation_error(quat, q0))
        T = max(C.PLAN["min_seg_time"], dist / (speed or C.PLAN["seg_speed"]), ang / 1.5)
        n_ik = max(1, int(np.ceil(T * C.PLAN["ik_rate"])))
        steps_per_ik = max(1, int(round(1.0 / (C.PLAN["ik_rate"] * self.dt))))
        for k in range(1, n_ik + 1):
            s = k / n_ik
            s = 3 * s ** 2 - 2 * s ** 3           # smoothstep timing
            tp = p0 + s * (pos - p0)
            tq = quat_slerp(q0, quat, s)
            r = self._ik(self.q_cmd, tp, tq)
            q_prev = self.q_cmd.copy()
            # interpolate commanded joints between IK ticks for smooth torque
            for j in range(steps_per_ik):
                a = (j + 1) / steps_per_ik
                self.q_cmd = q_prev + a * (r.q - q_prev)
                self.qd_cmd = (r.q - q_prev) * C.PLAN["ik_rate"]
                self.simulate(self.dt)
            self.qd_cmd[:] = 0
            if k == n_ik and not r.converged and (r.pos_err > 5 * C.CHECK["ik_pos_tol"]
                                                   or r.rot_err > 5 * C.CHECK["ik_rot_tol"]):
                self._fail("ik_infeasible", f"pos_err={r.pos_err:.4f} rot_err={r.rot_err:.3f}")
        self.qd_cmd[:] = 0
        self.simulate(C.PLAN["settle_time"])
        tcp, _ = self.tcp_pose()
        err = np.linalg.norm(tcp - pos)
        self.metrics["max_track_err"] = max(self.metrics["max_track_err"], err)
        if err > C.CHECK["tracking_tol"]:
            self._fail("tracking_error", f"TCP {err*100:.1f} cm from target")

    def gripper(self, close, phase):
        self.phase = phase
        self.grip_cmd = C.CTRL["gripper_close"] if close else C.CTRL["gripper_open"]
        self.simulate(C.PLAN["grip_time"])

    def pads_touch_cube(self):
        d = self.data
        touched = set()
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if self.cube_gid in pair:
                other = (pair - {self.cube_gid}).pop()
                if other in self.pad_g:
                    touched.add("left" if "left" in self._gname(other) else "right")
        return touched

    # --------------------------------------------------------------- planning --- #
    def _grasp_yaw(self, cube_yaw, ref_yaw):
        """Cube is 4-fold symmetric: choose cube_yaw + k*pi/2 closest to ref (alternate grasp yaw)."""
        cands = [wrap(cube_yaw + k * np.pi / 2) for k in range(4)]
        return min(cands, key=lambda a: abs(wrap(a - ref_yaw)))

    def _tcp_yaw(self, quat):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        return np.arctan2(R[1, 0], R[0, 0])

    def plan(self):
        """Try the 4 symmetric grasp yaws (closest to the approach direction first);
        the first one whose whole waypoint chain is IK-feasible wins
        ('alternate grasp yaw' mitigation). Re-raises the last failure otherwise."""
        qc = self.data.qpos[self.cube_qadr + 3:self.cube_qadr + 7]
        cyaw = 2 * np.arctan2(qc[3], qc[0])
        ref = np.arctan2(self.cube_init["y"], self.cube_init["x"])
        cands = sorted([wrap(cyaw + k * np.pi / 2) for k in range(4)], key=lambda a: abs(wrap(a - ref)))
        t0 = time.perf_counter()
        self.plan_iters = 0
        last = None
        for i, gyaw in enumerate(cands):
            try:
                wp = self._plan_with(gyaw)
                self.grasp_yaw_attempts = i + 1
                self.plan_time = time.perf_counter() - t0
                return wp
            except EpisodeFailure as e:
                last = e
        self.grasp_yaw_attempts = len(cands)
        self.plan_time = time.perf_counter() - t0
        raise last

    def _plan_with(self, gyaw):
        self.phase = "plan"
        P, T, H = C.PLAN, C.TRAY, C.TABLE
        cx, cy, cyaw = self.cube_init["x"], self.cube_init["y"], self.cube_init["yaw"]
        cube_now = self.cube_pos()
        # actual cube yaw after settling
        qc = self.data.qpos[self.cube_qadr + 3:self.cube_qadr + 7]
        cyaw = 2 * np.arctan2(qc[3], qc[0])
        _, q_home = self.ik._fk(self.home_q)
        home_yaw = self._tcp_yaw(q_home)
        gq = yaw_quat(gyaw)

        top = H["height"]
        wall_top = top + T["floor_thickness"] + T["wall_height"]
        s = C.CUBE["half_size"]
        grasp_z = cube_now[2] + P["grasp_depth"]
        wp = {}
        wp["pregrasp"] = (np.array([cube_now[0], cube_now[1], grasp_z + P["pregrasp_height"]]), gq)
        wp["grasp"] = (np.array([cube_now[0], cube_now[1], grasp_z]), gq)
        # transit altitude: cube bottom clears wall top by tray_clearance
        cube_below_tcp = (grasp_z - cube_now[2]) + s       # TCP -> cube bottom
        transit_z = max(wall_top + P["tray_clearance"] + cube_below_tcp, top + P["lift_height"])
        wp["lift"] = (np.array([cube_now[0], cube_now[1], transit_z]), gq)
        # place yaw snapped to tray axes (cube faces parallel to walls)
        tx, ty = T["center_xy"]
        if self.smart_tray:
            pyaw = min([k * np.pi / 2 for k in range(-2, 3)], key=lambda a: abs(wrap(a - gyaw)))
        else:
            pyaw = gyaw          # ablation: keep the grasp yaw (cube not aligned to walls)
        pq = yaw_quat(pyaw)
        wp["above_tray"] = (np.array([tx, ty, transit_z]), pq)
        place_z = top + T["floor_thickness"] + P["place_release_height"] + cube_below_tcp
        wp["place"] = (np.array([tx, ty, place_z]), pq)
        wp["retreat"] = (np.array([tx, ty, transit_z]), pq)

        # footprint check: cube footprint in tray axes (+5 mm margin) must fit inside
        rel = wrap(pyaw)   # cube faces are aligned with the gripper closing axis
        half_ext = s * (abs(np.cos(rel)) + abs(np.sin(rel)))
        if self.smart_tray and half_ext + 0.005 > min(T["inner_size"]) / 2:
            raise EpisodeFailure("place_infeasible", "plan", "cube footprint exceeds tray")

        self.plan_ik = {}
        q = self.home_q.copy()
        keys = ["pregrasp", "grasp", "lift", "above_tray", "place"]
        if not self.smart_tray:
            keys.remove("above_tray")
        for k in keys:
            r = self.ik.solve(q, *wp[k])
            self.plan_iters += r.iterations
            self.plan_ik[k] = r
            if not r.converged:
                raise EpisodeFailure("ik_infeasible", f"plan:{k}",
                                     f"pos_err={r.pos_err:.4f} rot_err={r.rot_err:.3f}")
            if r.joint_limit_violation:
                raise EpisodeFailure("joint_limit", f"plan:{k}", "solution outside joint range")
            q = r.q
        self.waypoints = wp
        self.grasp_yaw, self.place_yaw, self.transit_z = gyaw, pyaw, transit_z
        return wp

    # --------------------------------------------------------------- episode ---- #
    def run_episode(self, seed):
        t_wall = time.perf_counter()
        self.reset(seed)
        rec = dict(seed=seed, robot=self.robot, method=self.method, **{f"cube_{k}": v for k, v in self.cube_init.items()})
        self.plan_time, self.plan_iters, self.grasp_yaw_attempts = np.nan, 0, 0
        try:
            wp = self.plan()
            self.allow = set()
            self.move_to(*wp["pregrasp"], "approach")
            self.move_to(*wp["grasp"], "descend", speed=0.08)
            self.gripper(True, "grasp")
            touched = self.pads_touch_cube()
            if len(touched) < 2:
                self._fail("grasp_failed", f"pads in contact: {sorted(touched) or 'none'}")
            z0 = self.cube_pos()[2]
            tcp, _ = self.tcp_pose()
            self.carry_offset = tcp[2] - self.cube_pos()[2]
            self.move_to(*wp["lift"], "lift", speed=0.10)
            if self.cube_pos()[2] - z0 < C.CHECK["lift_min_rise"]:
                self._fail("grasp_failed", "cube did not rise with the gripper")
            self.allow = {"carrying"}
            if self.smart_tray:
                # carry at the computed altitude to directly above the tray, then
                # descend vertically: swept cube volume never meets a wall
                self.move_to(*wp["above_tray"], "transit")
                self.move_to(*wp["place"], "place_descend", speed=0.08)
            else:
                # ablation: straight diagonal from lift point into the tray
                self.move_to(*wp["place"], "transit")
            self.allow = {"cube_tray_wall"}          # cube may brush a wall while released
            self.gripper(False, "release")
            self.move_to(*wp["retreat"], "retreat", speed=0.10)
            self.phase = "verify"
            self.allow = {"cube_tray_wall"}
            self.simulate(0.5)
            ok, why = self.cube_in_tray()
            if not ok:
                self._fail("place_missed", why)
            rec.update(success=True, failure_reason="", failure_phase="", failure_detail="")
        except EpisodeFailure as e:
            if self.on_failure is not None:
                try:
                    self.on_failure(self, e)
                except Exception as ex:      # snapshots must never kill a batch run
                    print("  [snapshot failed]", ex)
            rec.update(success=False, failure_reason=e.reason, failure_phase=e.phase,
                       failure_detail=e.detail)
        rec.update(self._summarise())
        rec["plan_time_s"] = self.plan_time
        rec["plan_ik_iters"] = self.plan_iters
        rec["grasp_yaw_attempts"] = self.grasp_yaw_attempts
        rec["sim_time_s"] = float(self.data.time)
        rec["wall_time_s"] = time.perf_counter() - t_wall
        return rec

    def cube_in_tray(self):
        T, H = C.TRAY, C.TABLE
        p = self.cube_pos()
        dx, dy = p[0] - T["center_xy"][0], p[1] - T["center_xy"][1]
        ix, iy = np.array(T["inner_size"]) / 2
        if abs(dx) > ix or abs(dy) > iy:
            return False, f"cube at ({p[0]:.3f},{p[1]:.3f}) outside tray"
        if p[2] > H["height"] + T["floor_thickness"] + 3 * C.CUBE["half_size"]:
            return False, "cube not resting on tray floor"
        if np.linalg.norm(self.data.qvel[self.cube_vadr:self.cube_vadr + 3]) > 0.05:
            return False, "cube still moving"
        return True, ""

    def _summarise(self):
        it = np.array([a for a, _, _ in self.ik_log]) if self.ik_log else np.zeros(1)
        tt = np.array([b for _, b, _ in self.ik_log]) if self.ik_log else np.zeros(1)
        mtr = self.metrics
        fin = lambda v: float(v) if np.isfinite(v) else np.nan
        return dict(
            ik_calls=len(self.ik_log),
            ik_iters_total=int(it.sum()), ik_iters_mean=float(it.mean()), ik_iters_max=int(it.max()),
            ik_time_total_s=float(tt.sum()), ik_time_mean_ms=float(tt.mean() * 1e3),
            ik_nonconverged=mtr["ik_nonconverged"],
            ik_joint_limit_violations=mtr["ik_joint_limit_violations"],
            ik_vel_violations=mtr["ik_vel_violations"],
            sim_joint_limit_violations=mtr["joint_limit_violations"],
            sim_joint_vel_violations=mtr["joint_vel_violations"],
            min_clear_cube_tray_m=fin(mtr["min_clear_cube_tray"]),
            min_clear_gripper_table_m=fin(mtr["min_clear_grip_table"]),
            min_clear_gripper_tray_m=fin(mtr["min_clear_grip_tray"]),
            max_track_err_m=mtr["max_track_err"],
            grasp_yaw=float(getattr(self, "grasp_yaw", np.nan)),
            place_yaw=float(getattr(self, "place_yaw", np.nan)),
        )
