#!/usr/bin/env python3
"""
Compare IK methods from the run folders in logs/<robot>/.

For every robot and (profile, naive/smart) group it takes the newest run per method and writes:
  results/<robot>/<group>/metrics_table.md / .csv  - summary table (Section 10 + 13 metrics)
  results/<group>/success_and_failures.png - success rate + failure breakdown
  results/<group>/ik_timing.png            - IK time per call, plan time, iterations
  results/<group>/clearance.png            - min cube-tray / gripper-table clearance
  results/<group>/spawn_map.png            - cube spawn poses, success vs failure
  results/<robot>/ablation_tray_avoidance.png - smart vs naive tray approach (if both exist)
  results/robot_comparison.md              - HEAL vs Franka, nominal profile, all methods

Usage (from Lab3/):  python scripts/analysis/compare_ik_methods.py [--robot heal|franka] [--logs logs] [--out results]
"""
import argparse
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.patches as patches  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
from utils.pick_place import config as C  # noqa: E402

METHODS = ["mink", "dls", "qp"]
LABEL = {"mink": "Mink (off-the-shelf)", "dls": "DLS-IK", "qp": "QP-IK"}
COLOR = {"mink": "#4C72B0", "dls": "#DD8452", "qp": "#55A868"}


def load_runs(logdir):
    runs = []
    for s in sorted(glob.glob(os.path.join(logdir, "*", "summary.json"))):
        d = os.path.dirname(s)
        summ = json.load(open(s))
        df = pd.read_csv(os.path.join(d, "episodes.csv"))
        key = (summ["profile"], "smart" if summ["smart_tray"] else "naive")
        runs.append(dict(dir=d, summary=summ, df=df, key=key, method=summ["method"]))
    groups = {}
    for r in runs:                               # newest run per method wins (sorted by name/time)
        groups.setdefault(r["key"], {})[r["method"]] = r
    return groups


def table(group):
    rows = []
    for m in METHODS:
        if m not in group:
            continue
        s, df = group[m]["summary"], group[m]["df"]
        rows.append({
            "Method": LABEL[m],
            "Episodes": s["episodes"],
            "Success rate (%)": round(100 * s["success_rate"], 1),
            "Mean plan time (ms)": round(s["mean_plan_time_ms"], 2),
            "IK time / call (ms)": round(s["mean_ik_time_per_call_ms"], 3),
            "IK time / episode (s)": round(s["mean_ik_time_per_episode_s"], 3),
            "Plan IK iters": round(s["mean_plan_ik_iters"], 1),
            "IK iters / call": round(s["mean_ik_iters_per_call"], 2),
            "Max IK iters": s["max_ik_iters"],
            "Min clr cube-tray (cm)": round(100 * s["min_clearance_cube_tray_m"], 2),
            "Min clr gripper-table (cm)": round(100 * s["min_clearance_gripper_table_m"], 2),
            "Mean max track err (mm)": round(s["mean_max_track_err_mm"], 2),
            "IK joint-limit viol.": s["ik_joint_limit_violations"],
            "IK velocity viol.": s["ik_velocity_violations"],
            "Sim vel-limit steps": s["sim_joint_vel_violation_steps"],
            "Failures": ", ".join(f"{k}:{v}" for k, v in s["failure_breakdown"].items()) or "-",
        })
    return pd.DataFrame(rows)


def plot_success(group, path):
    ms = [m for m in METHODS if m in group]
    reasons = sorted({k for m in ms for k in group[m]["summary"]["failure_breakdown"]})
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    sr = [100 * group[m]["summary"]["success_rate"] for m in ms]
    ax[0].bar([LABEL[m] for m in ms], sr, color=[COLOR[m] for m in ms])
    for i, v in enumerate(sr):
        ax[0].text(i, v + 1, f"{v:.0f}%", ha="center")
    ax[0].set_ylim(0, 110)
    ax[0].set_ylabel("success rate (%)")
    ax[0].set_title("Success rate")
    bottom = np.zeros(len(ms))
    cmap = plt.get_cmap("tab10")
    for i, r in enumerate(reasons):
        v = np.array([group[m]["summary"]["failure_breakdown"].get(r, 0) for m in ms])
        ax[1].bar([LABEL[m] for m in ms], v, bottom=bottom, label=r, color=cmap(i))
        bottom += v
    ax[1].set_ylabel("# failed episodes")
    ax[1].set_title("Failure breakdown")
    if reasons:
        ax[1].legend(fontsize=8)
    else:
        ax[1].text(0.5, 0.5, "no failures", ha="center", transform=ax[1].transAxes)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_timing(group, path):
    ms = [m for m in METHODS if m in group]
    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    for a, col, ttl, sc in [(ax[0], "ik_time_mean_ms", "IK time per call (ms)", 1),
                            (ax[1], "plan_time_s", "Planning time per episode (ms)", 1e3),
                            (ax[2], "plan_ik_iters", "Planning IK iterations / episode", 1)]:
        data = [group[m]["df"][col].dropna().values * sc for m in ms]
        bp = a.boxplot(data, tick_labels=[LABEL[m] for m in ms], patch_artist=True)
        for p, m in zip(bp["boxes"], ms):
            p.set_facecolor(COLOR[m])
        a.set_title(ttl)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_clearance(group, path):
    ms = [m for m in METHODS if m in group]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for a, col, ttl in [(ax[0], "min_clear_cube_tray_m", "Min cube-tray-wall clearance / episode"),
                        (ax[1], "min_clear_gripper_table_m", "Min finger-pad-table clearance / episode")]:
        data = [group[m]["df"][col].dropna().values * 100 for m in ms]
        bp = a.boxplot(data, tick_labels=[LABEL[m] for m in ms], patch_artist=True)
        for p, m in zip(bp["boxes"], ms):
            p.set_facecolor(COLOR[m])
        a.axhline(0, color="r", lw=1, ls="--")
        a.set_ylabel("cm")
        a.set_title(ttl)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def draw_layout(a):
    T, R = C.TABLE, C.TRAY
    tx, ty = T["center_xy"]
    a.add_patch(patches.Rectangle((tx - T["length"] / 2, ty - T["width"] / 2), T["length"],
                                  T["width"], fc="#f3e1c7", ec="k", label="table"))
    ix, iy = R["inner_size"]
    w = R["wall_thickness"]
    a.add_patch(patches.Rectangle((R["center_xy"][0] - ix / 2 - w, R["center_xy"][1] - iy / 2 - w),
                                  ix + 2 * w, iy + 2 * w, fc="#9cc3ea", ec="b", label="tray"))
    S = C.SPAWN
    a.add_patch(patches.Rectangle((S["x_range"][0], S["y_range"][0]), np.ptp(S["x_range"]),
                                  np.ptp(S["y_range"]), fill=False, ec="g", ls="--",
                                  label="nominal spawn box"))
    for r in S["r_range"]:
        a.add_patch(patches.Circle((0, 0), r, fill=False, ec="g", ls=":"))
    a.plot(0, 0, "ks", label="robot base")


def plot_spawn(group, path):
    ms = [m for m in METHODS if m in group]
    fig, ax = plt.subplots(1, len(ms), figsize=(5 * len(ms), 5.5), squeeze=False)
    for a, m in zip(ax[0], ms):
        draw_layout(a)
        df = group[m]["df"]
        ok, bad = df[df.success], df[~df.success]
        L = 0.025
        for _, r in ok.iterrows():
            a.plot([r.cube_x, r.cube_x + L * np.cos(r.cube_yaw)],
                   [r.cube_y, r.cube_y + L * np.sin(r.cube_yaw)], "g-", lw=1)
        a.scatter(ok.cube_x, ok.cube_y, c="g", s=22, label="success", zorder=3)
        a.scatter(bad.cube_x, bad.cube_y, c="r", marker="x", s=50, label="failure", zorder=3)
        for _, r in bad.iterrows():
            a.annotate(r.failure_reason, (r.cube_x, r.cube_y), fontsize=7, color="r",
                       xytext=(3, 3), textcoords="offset points")
        a.set_aspect("equal")
        T = C.TABLE
        a.set_xlim(-0.05, T["center_xy"][0] + T["length"] / 2 + 0.05)
        a.set_ylim(-T["width"] / 2 - 0.05, T["width"] / 2 + 0.05)
        a.set_xlabel("x (m)")
        a.set_ylabel("y (m)")
        a.set_title(LABEL[m])
    ax[0][0].legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_ablation(groups, path):
    smart, naive = groups.get(("nominal", "smart")), groups.get(("nominal", "naive"))
    if not smart or not naive:
        return False
    ms = [m for m in METHODS if m in smart and m in naive]
    x = np.arange(len(ms))
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for off, g, lab, c in [(-0.2, smart, "smart tray approach", "#55A868"),
                           (0.2, naive, "naive diagonal approach", "#C44E52")]:
        ax[0].bar(x + off, [100 * g[m]["summary"]["success_rate"] for m in ms], 0.4, label=lab, color=c)
        ax[1].bar(x + off, [100 * np.nanmedian(g[m]["df"]["min_clear_cube_tray_m"]) for m in ms],
                  0.4, label=lab, color=c)
    for a, t in [(ax[0], "Success rate (%)"), (ax[1], "Median min cube-tray clearance (cm)")]:
        a.set_xticks(x)
        a.set_xticklabels([LABEL[m] for m in ms])
        a.set_title(t)
        a.legend(fontsize=8)
    ax[1].axhline(0, color="k", lw=0.8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", choices=list(C.ROBOTS), default=None, help="default: all robots found")
    p.add_argument("--logs", default=os.path.join(ROOT, "logs"))
    p.add_argument("--out", default=os.path.join(ROOT, "results"))
    a = p.parse_args()
    robots = [a.robot] if a.robot else [r for r in C.ROBOTS if os.path.isdir(os.path.join(a.logs, r))]
    nominal = {}
    for robot in robots:
        C.use(robot)
        g = compare_robot(robot, os.path.join(a.logs, robot), os.path.join(a.out, robot))
        if g and ("nominal", "smart") in g:
            nominal[robot] = table(g[("nominal", "smart")])
    if len(nominal) > 1:
        rows = []
        for robot, t in nominal.items():
            t = t.copy()
            t.insert(0, "Robot", robot.upper())
            rows.append(t)
        cmp = pd.concat(rows)
        keep = ["Robot", "Method", "Episodes", "Success rate (%)", "Mean plan time (ms)",
                "IK time / call (ms)", "Plan IK iters", "Min clr cube-tray (cm)",
                "Mean max track err (mm)", "IK joint-limit viol.", "Failures"]
        with open(os.path.join(a.out, "robot_comparison.md"), "w") as f:
            f.write("# HEAL vs Franka - nominal profile\n\n" + cmp[keep].to_markdown(index=False) + "\n")
        print("\nwrote robot_comparison.md")


def compare_robot(robot, logdir, out):
    groups = load_runs(logdir)
    if not groups:
        print(f"[{robot}] no runs with summary.json in {logdir}")
        return None
    os.makedirs(out, exist_ok=True)
    a = argparse.Namespace(out=out)
    for (profile, mode), g in sorted(groups.items()):
        name = f"{profile}_{mode}"
        od = os.path.join(a.out, name)
        os.makedirs(od, exist_ok=True)
        t = table(g)
        t.to_csv(os.path.join(od, "metrics_table.csv"), index=False)
        with open(os.path.join(od, "metrics_table.md"), "w") as f:
            f.write(f"# {name}\n\n" + t.to_markdown(index=False) + "\n\nRuns:\n" +
                    "".join(f"- {m}: `{os.path.relpath(g[m]['dir'], ROOT)}`\n" for m in g))
        plot_success(g, os.path.join(od, "success_and_failures.png"))
        plot_timing(g, os.path.join(od, "ik_timing.png"))
        plot_clearance(g, os.path.join(od, "clearance.png"))
        plot_spawn(g, os.path.join(od, "spawn_map.png"))
        print(f"\n===== {robot} / {name} =====")
        print(t.drop(columns=["Failures"]).T.to_string(header=False))
        print("Failures:", dict(zip(t.Method, t.Failures)))
    if plot_ablation(groups, os.path.join(a.out, "ablation_tray_avoidance.png")):
        print("\nwrote ablation_tray_avoidance.png")
    print(f"\nresults -> {a.out}")
    return groups


if __name__ == "__main__":
    main()
