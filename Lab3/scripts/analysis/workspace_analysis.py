#!/usr/bin/env python3
"""
Workspace analysis for HEAL and Franka, using the SAME MuJoCo models as the simulation
(robot + gripper + 'tcp' site from utils/pick_place/robots/<robot>.py).

1) Task (reachable) workspace
   Monte-Carlo forward kinematics: N random joint configurations, uniform inside the
   MuJoCo joint limits -> TCP positions. Plotted as density maps: top view (x-y) and
   side view (radius r - height z), with the table drawn in.

2) Dexterous workspace (yaw dexterity, top-down grasp)
   On a 2-D grid at a given TCP height, solve IK (DLS) for a top-down TCP pose at
   n_yaw yaws evenly spread over [-pi, pi). A cell's score = fraction of yaws that are
   reachable within joint limits. Score 1.0 = arbitrary yaw feasible (dexterous).
   Computed at two heights: the grasp height (TCP at cube centre + grasp_depth) and
   the transit height, i.e. the two heights the pick-and-place actually uses.
   Table, tray and nominal spawn region are overlaid -> justifies the table design.

Outputs (results/<robot>/workspace/):
   task_workspace.png, dexterous_workspace.png, dexterous_grid.csv

Usage (from Lab3/):
   python scripts/analysis/workspace_analysis.py --robot heal
   python scripts/analysis/workspace_analysis.py --robot franka
   options: --samples 50000 --grid 0.04 --yaws 8
"""
import argparse
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
from utils.pick_place import config as C  # noqa: E402


def task_workspace(model, qidx, n, rng):
    """Random joint configs inside limits -> TCP positions (N x 3)."""
    d = mujoco.MjData(model)
    jid = [model.jnt_qposadr.tolist().index(a) for a in qidx]
    lo, hi = model.jnt_range[jid, 0], model.jnt_range[jid, 1]
    sid = model.site("tcp").id
    pts = np.empty((n, 3))
    for i in range(n):
        d.qpos[qidx] = rng.uniform(lo, hi)
        mujoco.mj_kinematics(model, d)
        pts[i] = d.site_xpos[sid]
    return pts


def dexterous_grid(model, qidx, vidx, z, xs, ys, n_yaw, ik_cls, yaw_quat):
    """Fraction of n_yaw top-down yaws reachable (within joint limits) per (x, y) cell."""
    ik = ik_cls(model, "tcp", qidx, vidx, max_iters=200, pos_tol=2e-3, rot_tol=2e-2,
                null_gain=0.0)
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    ik.set_seed_qpos(d.qpos)
    home = C.ROBOT_MOD.HOME_SEED
    yaws = np.linspace(-np.pi, np.pi, n_yaw, endpoint=False)
    score = np.zeros((len(ys), len(xs)))
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            base = np.arctan2(y, x)
            seeds = [home, home + np.r_[base, np.zeros(len(qidx) - 1)]]
            ok = 0
            for yaw in yaws:
                for s in seeds:
                    r = ik.solve(s, np.array([x, y, z]), yaw_quat(yaw))
                    if r.converged and not r.joint_limit_violation:
                        ok += 1
                        break
            score[iy, ix] = ok / n_yaw
    return score


def draw_layout(ax):
    T, R, S = C.TABLE, C.TRAY, C.SPAWN
    tx, ty = T["center_xy"]
    ax.add_patch(patches.Rectangle((tx - T["length"] / 2, ty - T["width"] / 2), T["length"],
                                   T["width"], fill=False, ec="k", lw=2, label="table"))
    ix, iy = R["inner_size"]
    w = R["wall_thickness"]
    ax.add_patch(patches.Rectangle((R["center_xy"][0] - ix / 2 - w, R["center_xy"][1] - iy / 2 - w),
                                   ix + 2 * w, iy + 2 * w, fill=False, ec="b", lw=2, label="tray"))
    ax.add_patch(patches.Rectangle((S["x_range"][0], S["y_range"][0]), np.ptp(S["x_range"]),
                                   np.ptp(S["y_range"]), fill=False, ec="lime", ls="--", lw=1.5,
                                   label="spawn box"))
    for r in S["r_range"]:
        ax.add_patch(patches.Circle((0, 0), r, fill=False, ec="lime", ls=":", lw=1.2))
    ax.plot(0, 0, "ks", ms=7, label="robot base")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", choices=list(C.ROBOTS), required=True)
    p.add_argument("--samples", type=int, default=50000, help="FK samples (task workspace)")
    p.add_argument("--grid", type=float, default=0.04, help="grid step (m), dexterous workspace")
    p.add_argument("--yaws", type=int, default=8, help="yaws per cell, dexterous workspace")
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()

    C.use(a.robot)
    from utils.pick_place.scene import build_model, arm_indices
    from utils.pick_place.ik_solvers import DLSIK
    from utils.pick_place.pipeline import yaw_quat

    out = os.path.join(ROOT, "results", a.robot, "workspace")
    os.makedirs(out, exist_ok=True)
    model, _ = build_model()
    qidx, vidx, _ = arm_indices(model)
    T, P = C.TABLE, C.PLAN
    name = a.robot.upper()

    # ---------------- task workspace ---------------- #
    t0 = time.time()
    pts = task_workspace(model, qidx, a.samples, np.random.default_rng(a.seed))
    print(f"[{a.robot}] task workspace: {a.samples} FK samples in {time.time() - t0:.1f}s")
    r = np.hypot(pts[:, 0], pts[:, 1])
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    h = ax[0].hexbin(pts[:, 0], pts[:, 1], gridsize=70, bins="log", cmap="viridis", mincnt=1)
    draw_layout(ax[0])
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x (m)")
    ax[0].set_ylabel("y (m)")
    ax[0].set_title(f"{name} task workspace – top view (TCP positions)")
    ax[0].legend(loc="lower left", fontsize=8)
    fig.colorbar(h, ax=ax[0], label="log10(samples)")
    h2 = ax[1].hexbin(r, pts[:, 2], gridsize=70, bins="log", cmap="viridis", mincnt=1)
    x_far = np.hypot(T["center_xy"][0] + T["length"] / 2, T["width"] / 2)
    x_near = T["center_xy"][0] - T["length"] / 2
    ax[1].plot([x_near, x_far], [T["height"]] * 2, "k-", lw=3, label="table top (radial extent)")
    ax[1].axhline(0, color="gray", lw=0.8)
    ax[1].set_xlabel("radius r = sqrt(x²+y²) (m)")
    ax[1].set_ylabel("z (m)")
    ax[1].set_title(f"{name} task workspace – side view")
    ax[1].legend(loc="lower left", fontsize=8)
    fig.colorbar(h2, ax=ax[1], label="log10(samples)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "task_workspace.png"), dpi=140)
    plt.close(fig)
    print(f"  reach: r_max={r.max():.3f} m, z in [{pts[:, 2].min():.3f}, {pts[:, 2].max():.3f}] m")

    # ---------------- dexterous workspace ---------------- #
    cube_z = T["height"] + C.CUBE["half_size"]
    grasp_z = cube_z + P["grasp_depth"]
    wall_top = T["height"] + C.TRAY["floor_thickness"] + C.TRAY["wall_height"]
    transit_z = max(wall_top + P["tray_clearance"] + P["grasp_depth"] + C.CUBE["half_size"],
                    T["height"] + P["lift_height"])
    xmax = T["center_xy"][0] + T["length"] / 2 + 0.12
    ymax = T["width"] / 2 + 0.08
    xs = np.arange(0.0, xmax + 1e-9, a.grid)
    ys = np.arange(-ymax, ymax + 1e-9, a.grid)
    heights = [("grasp height", grasp_z), ("transit height", transit_z)]
    fig, axs = plt.subplots(1, 2, figsize=(14, 6.5))
    rows = []
    for ax, (lab, z) in zip(axs, heights):
        t0 = time.time()
        sc = dexterous_grid(model, qidx, vidx, z, xs, ys, a.yaws, DLSIK, yaw_quat)
        print(f"[{a.robot}] dexterous @ {lab} z={z:.3f}: {sc.size} cells x {a.yaws} yaws "
              f"in {time.time() - t0:.1f}s; fully dexterous cells: {(sc == 1).sum()}")
        im = ax.imshow(sc, origin="lower", cmap="RdYlGn", vmin=0, vmax=1, aspect="equal",
                       extent=[xs[0] - a.grid / 2, xs[-1] + a.grid / 2,
                               ys[0] - a.grid / 2, ys[-1] + a.grid / 2])
        draw_layout(ax)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_xlim(-0.05, xs[-1] + a.grid / 2)
        ax.set_ylim(ys[0] - a.grid / 2, ys[-1] + a.grid / 2)
        ax.set_title(f"{name} dexterous ws @ {lab}\nTCP z = {z:.3f} m, {a.yaws} top-down yaws")
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                rows.append((lab, z, x, y, sc[iy, ix]))
    axs[0].legend(loc="lower left", fontsize=8)
    fig.colorbar(im, ax=axs, label="yaw-feasible fraction (1 = dexterous)", shrink=0.8)
    fig.savefig(os.path.join(out, "dexterous_workspace.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)
    with open(os.path.join(out, "dexterous_grid.csv"), "w") as f:
        f.write("height_label,z,x,y,yaw_feasible_fraction\n")
        for rw in rows:
            f.write(f"{rw[0]},{rw[1]:.4f},{rw[2]:.3f},{rw[3]:.3f},{rw[4]:.3f}\n")
    print(f"saved -> {out}")


if __name__ == "__main__":
    main()
