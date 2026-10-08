"""
Three interchangeable IK solvers with one interface:

    solver.solve(q_init, target_pos, target_quat) -> IKResult

All of them are *iterative differential* solvers working on a private
planning copy of the model (they never touch the simulation MjData):

  * MinkIK  - off-the-shelf baseline (https://github.com/kevinzakka/mink)
  * DLSIK   - closed-form damped least squares, closed loop on the pose error,
              optional null-space joint-centering (supplementary notes, DLS section)
  * QPIK    - the same DLS cost written as a QP
                  min 1/2 dq^T H dq + c^T dq   s.t.  G dq <= h
              with joint-position limits and per-step velocity limits as
              hard inequality constraints (supplementary notes, QP section)

Quaternions follow MuJoCo convention: [w, x, y, z].
"""
import time
from dataclasses import dataclass, field

import mujoco
import numpy as np


# --------------------------------------------------------------------------- #
# Quaternion / pose-error helpers
# --------------------------------------------------------------------------- #
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def orientation_error(q_target, q_current):
    """Axis-angle vector eps = theta*u of q_e = q_t (x) q_c^-1 (world frame).
    Hemisphere continuity: force w >= 0 so we always take the short way."""
    qe = quat_mul(q_target, quat_conj(q_current))
    if qe[0] < 0:
        qe = -qe
    w = np.clip(qe[0], -1.0, 1.0)
    theta = 2.0 * np.arccos(w)
    if theta < 1e-6:
        return np.zeros(3)
    return theta * qe[1:] / np.sin(theta / 2.0)


def mat_to_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


# --------------------------------------------------------------------------- #
@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    iterations: int
    solve_time: float
    pos_err: float
    rot_err: float
    joint_limit_violation: bool = False
    velocity_violation: bool = False
    info: dict = field(default_factory=dict)


class BaseIK:
    """Holds a planning MjData and the arm DoF indices; subclasses implement _step()."""
    name = "base"

    def __init__(self, model, site_name, arm_qpos_idx, arm_dof_idx,
                 max_iters=150, pos_tol=1e-3, rot_tol=1e-2, dt=0.02,
                 max_joint_vel=2.0):
        self.model = model
        self.data = mujoco.MjData(model)
        self.site_id = model.site(site_name).id
        self.qidx = np.asarray(arm_qpos_idx)
        self.vidx = np.asarray(arm_dof_idx)
        self.n = len(self.qidx)
        self.max_iters = max_iters
        self.pos_tol = pos_tol
        self.rot_tol = rot_tol
        self.dt = dt                                 # integration step of the IK loop
        self.vmax = np.full(self.n, max_joint_vel)   # rad/s
        jid = [model.dof_jntid[v] for v in self.vidx]
        self.qmin = model.jnt_range[jid, 0].copy()
        self.qmax = model.jnt_range[jid, 1].copy()

    # ---- kinematics on the planning copy ---------------------------------- #
    def _fk(self, q):
        self.data.qpos[self.qidx] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        pos = self.data.site_xpos[self.site_id].copy()
        quat = mat_to_quat(self.data.site_xmat[self.site_id])
        return pos, quat

    def _jacobian(self):
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jp, jr, self.site_id)
        return np.vstack([jp[:, self.vidx], jr[:, self.vidx]])

    def _error(self, q, tpos, tquat):
        pos, quat = self._fk(q)
        return np.concatenate([tpos - pos, orientation_error(tquat, quat)])

    def set_seed_qpos(self, full_qpos):
        """Copy non-arm joints (gripper) so the site pose is consistent."""
        self.data.qpos[:] = full_qpos

    # ---- main loop -------------------------------------------------------- #
    def solve(self, q_init, tpos, tquat):
        t0 = time.perf_counter()
        q = np.array(q_init, dtype=float)
        jl_viol = vel_viol = False
        it = 0
        e = self._error(q, tpos, tquat)
        for it in range(1, self.max_iters + 1):
            if np.linalg.norm(e[:3]) < self.pos_tol and np.linalg.norm(e[3:]) < self.rot_tol:
                it -= 1
                break
            dq = self._step(q, e)
            if np.any(np.abs(dq) > self.vmax * self.dt + 1e-9):
                vel_viol = True
            q = q + dq
            if np.any(q < self.qmin - 1e-6) or np.any(q > self.qmax + 1e-6):
                jl_viol = True
            e = self._error(q, tpos, tquat)
        pe, re = np.linalg.norm(e[:3]), np.linalg.norm(e[3:])
        return IKResult(q=q, converged=bool(pe < self.pos_tol and re < self.rot_tol),
                        iterations=it, solve_time=time.perf_counter() - t0,
                        pos_err=float(pe), rot_err=float(re),
                        joint_limit_violation=jl_viol, velocity_violation=vel_viol)

    def _step(self, q, e):
        raise NotImplementedError


# --------------------------------------------------------------------------- #
# 1) Damped Least Squares (closed loop)
# --------------------------------------------------------------------------- #
class DLSIK(BaseIK):
    """dq = (J^T J + lam I)^-1 J^T xdot  +  (I - (J^T J + lam I)^-1 J^T J) dq_null

    xdot = K * e is the closed-loop task velocity (e = pose error).
    dq_null pulls the joints toward mid-range (joint-limit avoidance).
    No hard limits: dq is only *scaled* to respect the velocity bound,
    and q is NOT clipped, so violations show up honestly in the logs."""
    name = "dls"

    def __init__(self, *a, damping=1e-3, gain=1.0, null_gain=0.05,
                 clamp_velocity=True, **kw):
        super().__init__(*a, **kw)
        self.lam = damping
        self.gain = gain
        self.alpha = null_gain
        self.clamp_velocity = clamp_velocity
        self.q_center = 0.5 * (self.qmin + self.qmax)

    def _step(self, q, e):
        J = self._jacobian()
        xdot = self.gain * e / self.dt
        A = J.T @ J + self.lam * np.eye(self.n)
        Ainv_JT = np.linalg.solve(A, J.T)
        qdot = Ainv_JT @ xdot
        if self.alpha > 0:
            N = np.eye(self.n) - Ainv_JT @ J
            qdot += N @ (self.alpha * (self.q_center - q))
        if self.clamp_velocity:
            s = np.max(np.abs(qdot) / self.vmax)
            if s > 1.0:
                qdot /= s
        return qdot * self.dt


# --------------------------------------------------------------------------- #
# 2) QP-based IK with joint-limit + velocity constraints
# --------------------------------------------------------------------------- #
class QPIK(BaseIK):
    """min 1/2 dq^T H dq + c^T dq   s.t.  G dq <= h
       H = 2/dt^2 (J^T J + lam I),  c = -1/dt J^T xdot
       G = [I; -I; I; -I],  h = [qmax-q; -(qmin-q); vmax*dt; vmax*dt]"""
    name = "qp"

    def __init__(self, *a, damping=1e-3, gain=1.0, solver="daqp",
                 limit_margin=0.0, **kw):
        super().__init__(*a, **kw)
        from qpsolvers import solve_qp
        self._solve_qp = solve_qp
        self.lam = damping
        self.gain = gain
        self.solver = solver
        self.margin = limit_margin
        I = np.eye(self.n)
        self.G = np.vstack([I, -I, I, -I])
        self.qp_failures = 0

    def _step(self, q, e):
        J = self._jacobian()
        xdot = self.gain * e / self.dt
        H = (2.0 / self.dt ** 2) * (J.T @ J + self.lam * np.eye(self.n))
        c = -(2.0 / self.dt) * (J.T @ xdot)       # factor 2 matches H's 1/2 convention
        h = np.concatenate([
            (self.qmax - self.margin) - q,
            -((self.qmin + self.margin) - q),
            self.vmax * self.dt,
            self.vmax * self.dt,
        ])
        h = np.maximum(h, 0.0) if np.any(h < 0) else h   # infeasible start -> stay put on that side
        dq = self._solve_qp(H, c, self.G, h, solver=self.solver)
        if dq is None:
            self.qp_failures += 1
            return np.zeros(self.n)
        return dq


# --------------------------------------------------------------------------- #
# 3) Off-the-shelf: Mink
# --------------------------------------------------------------------------- #
class MinkIK(BaseIK):
    """Mink differential IK: FrameTask on the TCP site + ConfigurationLimit +
    VelocityLimit, solved as a QP by mink.solve_ik each iteration.
    The gripper joints are frozen with a DofFreezingTask-like posture task of
    very high cost so only the 6 arm joints move."""
    name = "mink"

    def __init__(self, model, site_name, arm_qpos_idx, arm_dof_idx,
                 solver="daqp", damping=1e-3, **kw):
        super().__init__(model, site_name, arm_qpos_idx, arm_dof_idx, **kw)
        import mink
        self.mink = mink
        self.cfg = mink.Configuration(model)
        self.task = mink.FrameTask(frame_name=site_name, frame_type="site",
                                   position_cost=1.0, orientation_cost=1.0,
                                   lm_damping=1.0)
        # Freeze everything that is not an arm dof via an equality constraint.
        frozen = [i for i in range(model.nv) if i not in set(self.vidx)]
        self.constraints = []
        if frozen:
            self.constraints.append(mink.DofFreezingTask(model, dof_indices=frozen))
        arm_joint_names = [model.joint(model.dof_jntid[v]).name for v in self.vidx]
        self.limits = [
            mink.ConfigurationLimit(model),
            mink.VelocityLimit(model, {n: float(v) for n, v in zip(arm_joint_names, self.vmax)}),
        ]
        self.solver = solver
        self.damping = damping

    def solve(self, q_init, tpos, tquat):
        mink = self.mink
        t0 = time.perf_counter()
        qfull = self.data.qpos.copy()
        qfull[self.qidx] = q_init
        self.cfg.update(qfull)
        self.task.set_target(mink.SE3.from_rotation_and_translation(
            mink.SO3(np.asarray(tquat)), np.asarray(tpos)))
        it = 0
        vel_viol = False
        for it in range(1, self.max_iters + 1):
            err = self.task.compute_error(self.cfg)       # [lin(3), ang(3)] in local frame
            if np.linalg.norm(err[:3]) < self.pos_tol and np.linalg.norm(err[3:]) < self.rot_tol:
                it -= 1
                break
            vel = mink.solve_ik(self.cfg, [self.task], self.dt, self.solver,
                                damping=self.damping, limits=self.limits,
                                constraints=self.constraints)
            if np.any(np.abs(vel[self.vidx]) > self.vmax + 1e-6):
                vel_viol = True
            self.cfg.integrate_inplace(vel, self.dt)
        q = self.cfg.q[self.qidx].copy()
        e = self._error(q, tpos, tquat)
        pe, re = np.linalg.norm(e[:3]), np.linalg.norm(e[3:])
        jl = bool(np.any(q < self.qmin - 1e-6) or np.any(q > self.qmax + 1e-6))
        return IKResult(q=q, converged=bool(pe < self.pos_tol and re < self.rot_tol),
                        iterations=it, solve_time=time.perf_counter() - t0,
                        pos_err=float(pe), rot_err=float(re),
                        joint_limit_violation=jl, velocity_violation=vel_viol)


SOLVERS = {"mink": MinkIK, "dls": DLSIK, "qp": QPIK}


def make_solver(name, model, site_name, arm_qpos_idx, arm_dof_idx, **kw):
    return SOLVERS[name](model, site_name, arm_qpos_idx, arm_dof_idx, **kw)
