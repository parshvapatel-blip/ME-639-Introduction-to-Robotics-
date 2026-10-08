"""
Batch runner shared by scripts/ik_mink, scripts/ik_dls and scripts/ik_qp.

Outputs (one folder per run):
  logs/<robot>/<method>_<profile>[_naive]_<timestamp>/
      config.json        - every parameter used (reproducibility)
      episodes.csv       - one row per episode (all metrics, see README)
      episodes.jsonl     - same, JSON lines (appended live -> safe overnight)
      summary.json       - success rate, failure breakdown, means
      failures/ep_XXXX_<reason>.png   - snapshot at the failure instant
      failures/ep_XXXX_<reason>.npz   - full qpos/qvel/ctrl/time at failure
      video.mp4          - (--record) overview video of all episodes
"""
import argparse
import csv
import datetime as dt
import json
import os
import sys
import time

import numpy as np

from . import config as C
from .pipeline import PickPlaceEnv

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _jsonable(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, float) and not np.isfinite(o):
        return None
    return o


def config_snapshot():
    keep = ["TABLE", "TRAY", "CUBE", "SPAWN_PROFILES", "PLAN", "CTRL", "CHECK", "HOME_TCP"]
    return {k: json.loads(json.dumps(getattr(C, k), default=_jsonable)) for k in keep}


class Recorder:
    """Offscreen renderer for video (overview cam) and failure snapshots."""

    def __init__(self, model, video_path=None, fps=25, size=(360, 640)):
        import mujoco
        self.mujoco = mujoco
        self.r = mujoco.Renderer(model, *size)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = model.camera("overview").id
        self.writer = None
        self.fps = fps
        self.next_t = 0.0
        self.last_t = 0.0
        self.label = ""
        if video_path:
            import imageio
            self.writer = imageio.get_writer(video_path, fps=fps, codec="libx264",
                                             quality=7, macro_block_size=8)

    def frame(self, env):
        self.r.update_scene(env.data, self.cam)
        img = self.r.render().copy()
        return self._annotate(img)

    def _annotate(self, img):
        try:
            import cv2
            k = img.shape[1] / 1280.0                      # scale text with frame width
            cv2.putText(img, self.label, (int(20 * k), int(42 * k)), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0 * k, (255, 255, 255), max(1, int(round(2 * k))), cv2.LINE_AA)
        except Exception:
            pass
        return img

    def __call__(self, env):                     # per physics step hook
        if self.writer is None:
            return
        t = env.data.time
        # A new episode restarts sim time at 0. The hook runs after mj_step, so
        # t is never exactly 0 -> detect the jump backwards instead.
        if t < self.last_t:
            self.next_t = 0.0
        self.last_t = t
        if t >= self.next_t:
            self.label = f"{env.robot.upper()} {env.method.upper()}  ep {env._ep}  seed {env._seed}  phase: {env.phase}"
            self.writer.append_data(self.frame(env))
            self.next_t = t + 1.0 / self.fps

    def close(self):
        if self.writer is not None:
            self.writer.close()


def make_parser(default_method=None, default_robot=None):
    p = argparse.ArgumentParser(description="pick-and-place batch runner (HEAL / Franka)")
    if default_robot is None:
        p.add_argument("--robot", choices=list(C.ROBOTS), required=True)
    if default_method is None:
        p.add_argument("--method", choices=["mink", "dls", "qp"], required=True)
    p.add_argument("--episodes", "-n", type=int, default=25)
    p.add_argument("--seed", type=int, default=0, help="base seed; episode i uses seed+i")
    p.add_argument("--profile", choices=["nominal", "stress"], default="nominal")
    p.add_argument("--naive-tray", action="store_true",
                   help="ablation: disable cube-tray collision avoidance")
    p.add_argument("--viewer", action="store_true", help="live MuJoCo passive viewer")
    p.add_argument("--realtime", action="store_true", help="pace the viewer at real time")
    p.add_argument("--record", action="store_true", help="write video.mp4 (offscreen)")
    p.add_argument("--video-size", default="640x360", help="WxH for video/snapshots")
    p.add_argument("--no-snapshots", action="store_true", help="skip failure images")
    p.add_argument("--out", default=os.path.join(REPO_ROOT, "logs"))
    p.add_argument("--tag", default="", help="extra suffix for the run folder")
    return p


def run(args, method, robot):
    C.use(robot)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{method}_{args.profile}" + ("_naive" if args.naive_tray else "") \
        + (f"_{args.tag}" if args.tag else "") + f"_{stamp}"
    out = os.path.join(args.out, robot, name)
    os.makedirs(os.path.join(out, "failures"), exist_ok=True)

    env = PickPlaceEnv(method, profile=args.profile, smart_tray=not args.naive_tray)
    env._ep, env._seed = 0, 0

    # optional rendering -------------------------------------------------- #
    rec = None
    want_render = args.record or not args.no_snapshots
    if want_render:
        try:
            h, w = (int(v) for v in args.video_size.lower().split("x")[::-1])
            rec = Recorder(env.model, os.path.join(out, "video.mp4") if args.record else None,
                           size=(h, w))
        except Exception as ex:
            print(f"[warn] offscreen rendering unavailable ({ex}); "
                  "set MUJOCO_GL=egl or osmesa on headless machines. Continuing without images.")
            rec = None
    if rec is not None and args.record:
        env.render = rec

    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
        env.viewer = viewer
        if args.realtime:
            prev = env.render
            last = [time.perf_counter(), 0.0, 0.0]   # wall anchor, sim anchor, previous sim t

            def pace(e, prev=prev):
                if prev is not None:
                    prev(e)
                if e.data.time < last[2]:            # new episode: re-anchor the clock
                    last[0], last[1] = time.perf_counter(), e.data.time
                last[2] = e.data.time
                wall = time.perf_counter() - last[0]
                lag = (e.data.time - last[1]) - wall
                if lag > 0.005:
                    time.sleep(lag)
            env.render = pace

    def snapshot(e, err):
        base = os.path.join(out, "failures", f"ep_{e._ep:04d}_{err.reason}")
        np.savez(base + ".npz", qpos=e.data.qpos, qvel=e.data.qvel, ctrl=e.data.ctrl,
                 time=e.data.time, reason=err.reason, phase=err.phase, detail=err.detail,
                 seed=e._seed)
        if rec is not None:
            import imageio
            rec.label = f"{e.robot.upper()} {e.method.upper()} ep {e._ep} FAIL: {err.reason} @ {err.phase}"
            imageio.imwrite(base + ".png", rec.frame(e))

    if not args.no_snapshots:
        env.on_failure = snapshot

    with open(os.path.join(out, "config.json"), "w") as f:
        json.dump(dict(robot=robot, method=method, args=vars(args), config=config_snapshot()), f, indent=2,
                  default=_jsonable)

    print(f"[{robot}/{method}] {args.episodes} episodes, profile={args.profile}, "
          f"smart_tray={not args.naive_tray} -> {out}")
    rows = []
    jl = open(os.path.join(out, "episodes.jsonl"), "w")
    t0 = time.perf_counter()
    ep = 0
    while ep < args.episodes:                          # main episode loop
        seed = args.seed + ep
        env._ep, env._seed = ep, seed
        r = env.run_episode(seed)
        r["episode"] = ep
        rows.append(r)
        jl.write(json.dumps(r, default=_jsonable) + "\n")
        jl.flush()
        tag = "OK  " if r["success"] else f"FAIL {r['failure_reason']}@{r['failure_phase']}"
        print(f"  ep {ep:3d} seed {seed:4d} cube=({r['cube_x']:+.3f},{r['cube_y']:+.3f},"
              f"{np.degrees(r['cube_yaw']):+6.1f}deg)  {tag:38s} "
              f"ik {r['ik_time_mean_ms']:.2f}ms/{r['ik_iters_mean']:.2f}it  "
              f"clr {r['min_clear_cube_tray_m']*100 if np.isfinite(r['min_clear_cube_tray_m']) else float('nan'):5.1f}cm",
              flush=True)
        if viewer is not None and not viewer.is_running():
            break
        ep += 1
    jl.close()
    if rec is not None:
        rec.close()
    if viewer is not None:
        viewer.close()

    # ---- write CSV + summary ---------------------------------------------- #
    cols = ["episode", "seed", "robot", "method"] + [k for k in rows[0] if k not in ("episode", "seed", "robot", "method")]
    with open(os.path.join(out, "episodes.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: _jsonable(r.get(k)) for k in cols})

    summary = summarise(rows)
    summary.update(robot=robot, method=method, profile=args.profile, smart_tray=not args.naive_tray,
                   total_wall_time_s=time.perf_counter() - t0, run_dir=out)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2, default=_jsonable)
    print_summary(summary)
    return out, summary


def summarise(rows):
    n = len(rows)
    ok = [r for r in rows if r["success"]]
    fails = {}
    for r in rows:
        if not r["success"]:
            fails[r["failure_reason"]] = fails.get(r["failure_reason"], 0) + 1
    m = lambda k, rs=rows: float(np.nanmean([r[k] for r in rs])) if rs else float("nan")
    mn = lambda k, rs=rows: float(np.nanmin([r[k] for r in rs])) if rs else float("nan")
    return dict(
        episodes=n, successes=len(ok), success_rate=len(ok) / max(n, 1),
        failure_breakdown=fails,
        mean_plan_time_ms=m("plan_time_s") * 1e3,
        mean_ik_time_per_call_ms=m("ik_time_mean_ms"),
        mean_ik_time_per_episode_s=m("ik_time_total_s"),
        mean_ik_iters_per_call=m("ik_iters_mean"),
        mean_plan_ik_iters=m("plan_ik_iters"),
        max_ik_iters=int(np.max([r["ik_iters_max"] for r in rows])),
        min_clearance_cube_tray_m=mn("min_clear_cube_tray_m"),
        mean_min_clearance_cube_tray_m=m("min_clear_cube_tray_m"),
        min_clearance_gripper_table_m=mn("min_clear_gripper_table_m"),
        min_clearance_gripper_tray_m=mn("min_clear_gripper_tray_m"),
        mean_max_track_err_mm=m("max_track_err_m") * 1e3,
        ik_joint_limit_violations=int(sum(r["ik_joint_limit_violations"] for r in rows)),
        ik_velocity_violations=int(sum(r["ik_vel_violations"] for r in rows)),
        sim_joint_limit_violation_steps=int(sum(r["sim_joint_limit_violations"] for r in rows)),
        sim_joint_vel_violation_steps=int(sum(r["sim_joint_vel_violations"] for r in rows)),
        mean_episode_sim_time_s=m("sim_time_s"),
    )


def print_summary(s):
    print("\n==== summary ====")
    print(f" robot={s['robot']} method={s['method']} profile={s['profile']} smart_tray={s['smart_tray']}")
    print(f" success {s['successes']}/{s['episodes']} = {100*s['success_rate']:.1f}%")
    print(f" failures: {s['failure_breakdown'] or 'none'}")
    print(f" plan time {s['mean_plan_time_ms']:.2f} ms | IK/call {s['mean_ik_time_per_call_ms']:.3f} ms, "
          f"{s['mean_ik_iters_per_call']:.2f} it | plan IK iters {s['mean_plan_ik_iters']:.1f}")
    print(f" min clearance cube-tray {s['min_clearance_cube_tray_m']*100:.2f} cm | "
          f"gripper-table {s['min_clearance_gripper_table_m']*100:.2f} cm")
    print(f" IK joint-limit viol {s['ik_joint_limit_violations']} | IK vel viol {s['ik_velocity_violations']} | "
          f"sim vel-viol steps {s['sim_joint_vel_violation_steps']}")
    print(f" logs -> {s['run_dir']}")


def main(method=None, robot=None):
    args = make_parser(method, robot).parse_args()
    return run(args, method or args.method, robot or args.robot)


if __name__ == "__main__":
    main()
