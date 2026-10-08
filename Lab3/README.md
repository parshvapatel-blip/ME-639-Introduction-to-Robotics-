# ITR Lab 3 – Pick and Place in MuJoCo with IK (HEAL and Franka Panda)

Pick a randomly posed cube from a table and place it in a tray, for **two robots**
(HEAL 6-DoF + Robotiq 2F-85, Franka Emika Panda 7-DoF + Franka Hand) and **three IK methods**
behind one interface – 6 entry scripts sharing one pipeline:

| IK method | HEAL | Franka |
|---|---|---|
| Off-the-shelf **Mink** (https://github.com/kevinzakka/mink) – baseline | `scripts/heal/ik_mink/run_mink_pick_place.py` | `scripts/franka/ik_mink/run_mink_pick_place.py` |
| Closed-loop **DLS** + null-space joint centring | `scripts/heal/ik_dls/run_dls_pick_place.py` | `scripts/franka/ik_dls/run_dls_pick_place.py` |
| **QP-IK** with joint-position + velocity limits | `scripts/heal/ik_qp/run_qp_pick_place.py` | `scripts/franka/ik_qp/run_qp_pick_place.py` |

## 1. Install

```bash
pip install mujoco mink "qpsolvers[daqp]" numpy pandas matplotlib tabulate imageio imageio-ffmpeg opencv-python
```
Tested with MuJoCo 3.x, Python 3.12. On a headless machine set `MUJOCO_GL=egl` (or `osmesa`) for videos/snapshots.

## 2. Run (from the `Lab3/` folder)

Replace `heal` by `franka` in any path to run the Panda.
```bash
# 25-episode batch per method, identical seeds (episode i uses seed+i)
python scripts/heal/ik_mink/run_mink_pick_place.py --episodes 25 --seed 0
python scripts/heal/ik_dls/run_dls_pick_place.py   --episodes 25 --seed 0
python scripts/heal/ik_qp/run_qp_pick_place.py     --episodes 25 --seed 0

# demo video: records Mink, DLS, QP (5 episodes ≈ 1 min each) and joins them
#   -> videos/heal_all_methods.mp4   (needs: sudo apt install -y ffmpeg)
bash scripts/record_videos.sh heal
bash scripts/record_videos.sh franka

# watch live / record a single method
python scripts/franka/ik_dls/run_dls_pick_place.py -n 5 --viewer --realtime
python scripts/franka/ik_dls/run_dls_pick_place.py -n 5 --record --video-size 1280x720

# failure study: spawn over the whole table, incl. corners outside the reliable workspace
python scripts/franka/ik_qp/run_qp_pick_place.py -n 40 --seed 100 --profile stress

# ablation: switch OFF the cube-tray collision avoidance
python scripts/franka/ik_mink/run_mink_pick_place.py -n 25 --seed 0 --naive-tray

# tables + plots for every robot / run group in logs/  (or --robot heal|franka)
python scripts/analysis/compare_ik_methods.py

# inspect a scene in the MuJoCo viewer / dump it to XML
python -m utils.pick_place.scene --robot franka --view
python -m utils.pick_place.scene --robot franka --save
```
Overnight: raise `--episodes`; `episodes.jsonl` is appended after each episode, so nothing is lost if the run stops.
Run timing comparisons one after another (not in parallel) so CPU contention does not distort them.

## 3. Repository layout

```
Lab3/
├── README.md
├── robot_descriptions/
│   ├── single_arm_heal_effort_actuation_rs_mj.xml   # REBUILT (was missing) from arm_gripper.xml
│   ├── heal_meshes/                                 # (provided)
│   ├── robotiq_2f85_v4/  (+ assets/*.stl ADDED)     # HEAL gripper – MuJoCo Menagerie meshes
│   ├── franka_emika_panda/                          # ADDED – MuJoCo Menagerie Panda + hand + meshes
│   ├── heal_pick_place_scene.xml                    # compiled full scenes (inspection only)
│   ├── franka_pick_place_scene.xml
│   └── ... other provided files (ur5, dual_ur5, world, franka_scene, arm_gripper*)
├── utils/
│   ├── mj_velocity_control/                         # (provided)
│   └── pick_place/
│       ├── robots/heal.py, robots/franka.py         # ALL robot-specific numbers + model building
│       ├── config.py                                # selects the robot (config.use("franka"))
│       ├── scene.py                                 # robot + table + cube + tray via MjSpec
│       ├── ik_solvers.py                            # MinkIK, DLSIK, QPIK (solve(q0, pos, quat))
│       ├── pipeline.py                              # reset -> plan -> execute -> monitor -> verify
│       └── runner.py                                # CLI, batch loop, logs, snapshots, video, viewer
├── scripts/
│   ├── heal/   ik_mink/ ik_dls/ ik_qp/              # one entry script per method
│   ├── franka/ ik_mink/ ik_dls/ ik_qp/
│   ├── analysis/compare_ik_methods.py               # IK comparison tables + plots
│   ├── analysis/workspace_analysis.py               # task + dexterous workspace plots
│   ├── record_videos.sh                             # records + joins the demo video
│   └── starter_examples/                            # provided starter scripts (paths updated)
├── logs/heal/…, logs/franka/…                       # <method>_<profile>[_naive]_<time>/ per run
├── results/heal/…, results/franka/…, results/robot_comparison.md
│   └── <robot>/workspace/                           # task + dexterous workspace plots
├── videos/video.mp4                                 # final 3-min demo video (all IK methods)
├── submission/                                      # deliverables collected in one place (see below)
└── report/                                          # written report
```

### Fixes to the provided base code
* `single_arm_heal_effort_actuation_rs_mj.xml` (loaded by every starter script) was missing → rebuilt
  from the HEAL arm inside `arm_gripper.xml`, without its hand-written gripper (absolute `/Users/...` mesh paths).
  The starter scripts (`load_heal_robot.py`, `example_controlling_*.py`) now run unchanged.
* `robotiq_2f85_v4/assets/` was missing and `gripper.xml`/`gripper_cube.xml` are empty → official meshes added.
* The Franka files referenced by `franka_scene.xml` (`robot_description/mjx_panda.xml`, table arena, plate) are not in
  the base zip → the official Menagerie Panda was added under `robot_descriptions/franka_emika_panda/` and the scene is
  built in code (`franka_scene.xml` is left untouched; it still points to the missing files).
* Starter scripts moved to `scripts/starter_examples/` (their relative repo-root path was updated by one level).

### Where to find each deliverable

| Deliverable | Location |
|---|---|
| Code + instructions | this repository, this README |
| Workspace plots + table design | `submission/workspace_plots/` (task + dexterous, both robots), table rationale in §5 |
| Logs | `logs/<robot>/<run>/episodes.csv`, `summary.json` (18 runs) |
| Summary metrics tables | `submission/tables/` (`robot_comparison.md` = main table) |
| Failure case screenshots | `submission/failure_screenshots/` (`<robot>_<method>_<run>_ep_XXXX_<reason>.png`) |
| IK comparison plots | `submission/plots/` (success, timing, clearance, spawn maps, tray-avoidance ablation) |
| 3-min video | `videos/video.mp4` – Mink, DLS, QP on Franka then HEAL, 5 episodes each (~30 s per method per robot, 1 min per method in total) |
| Report | `report/` |

## 4. Method

**Scene.** Robot base at the origin; a `tcp` site between the finger pads with the same convention for both robots
(z = approach direction, x = finger closing axis), so the planner is robot-independent.

| | HEAL | Franka Panda |
|---|---|---|
| DoF | 6 (all joints ±π) | 7 (real limits, e.g. joint4 ∈ [−3.07, −0.07], joint6 ∈ [−0.02, 3.75]) |
| Gripper | Robotiq 2F-85 at `right_center` (as starter code), 4-bar linkage | Franka Hand, parallel jaw (255 = open, 0 = closed) |
| TCP | 0.14 m along flange z | 0.1034 m along hand z, rotated 90° so x = closing axis |
| Actuation | torque motors → `τ = qfrc_bias + Kp(q_cmd − q) + Kd(q̇_cmd − q̇) + b·q̇_cmd` (gravity comp., PD, feed-forward cancelling joint damping b = 55) | Menagerie position servos → `ctrl = q_cmd` |
| Grasp height | 14 mm above cube centre (2F-85 tips swing down) | cube centre |

**Motion plan per episode** (identical for both robots). home → pre-grasp (12 cm above) → descend → close → lift → transit → place → release → retreat.
1. *Plan:* IK on each key waypoint (warm-started in sequence). Unreachable → `ik_infeasible`, out of range → `joint_limit`.
2. *Execute:* each segment is a straight TCP line (smoothstep timing, quaternion slerp); the IK is called at 50 Hz on the
   interpolated pose and joint targets are interpolated between IK ticks.
3. *Alternate grasp yaw:* the cube is 4-fold symmetric, so the four yaws `cube_yaw + k·π/2` are tried, closest to the
   approach direction first; the first one whose whole waypoint chain is IK-feasible is used (`grasp_yaw_attempts` is
   logged). This matters on the Panda, whose joint-7 limit makes some yaws unreachable on the −y side.

**IK methods** (`ik_solvers.py`, quaternions w-first, pose error `e = [Δp, 2·Log(q_t ⊗ q_c⁻¹)]` with hemisphere flip):
* **DLS:** `q̇ = (JᵀJ + λI)⁻¹Jᵀ(K e/Δt) + (I − (JᵀJ+λI)⁻¹JᵀJ)·α(q_mid − q)`, λ = 1e-3; q̇ is scaled down (not clipped)
  to respect the velocity limit; joint limits are *not* enforced (violations are logged).
* **QP:** `min ½Δqᵀ H Δq + cᵀΔq`, `H = 2/Δt²(JᵀJ + λI)`, `c = −2/Δt·Jᵀẋ_d`, s.t. `G Δq ≤ h` with
  `G = [I; −I; I; −I]`, `h = [q_max−q; q−q_min; v_max Δt; v_max Δt]`; solver DAQP via `qpsolvers`.
* **Mink:** `FrameTask` on the `tcp` site + `ConfigurationLimit` + `VelocityLimit`; gripper DoFs frozen with `DofFreezingTask`.

All three use the same tolerances (2 mm, 0.02 rad), 150 max iterations and v_max = 2 rad/s.

## 5. Workspace & table design

Workspace plots: `python scripts/analysis/workspace_analysis.py --robot heal|franka` →
`results/<robot>/workspace/` (uses the same MuJoCo models as the simulation):
* **Task workspace** (`task_workspace.png`): 50 000 random joint configurations inside the joint limits → TCP
  positions, top view and side view (r–z), with the table drawn in. HEAL reaches r ≤ 0.85 m, Franka r ≤ 0.95 m.
* **Dexterous workspace** (`dexterous_workspace.png`, `dexterous_grid.csv`): 4 cm grid at the grasp height and the
  transit height; per cell, the fraction of 8 top-down yaws in [−π, π) that DLS IK reaches inside the joint limits
  (1.0 = any yaw feasible). Table, tray and spawn region are overlaid.
  * HEAL: the spawn box and tray lie in the fully dexterous region (score 1.0); the region ends at x ≈ 0.55–0.6 m,
    so the far table corners are outside it – exactly where the stress-test failures occur.
  * Franka: most of the table reaches 75–90 % of yaws (joint-7 limit), which motivates the alternate-grasp-yaw
    search; the area close to the base (r < 0.25 m) is not reachable top-down.

**HEAL**

| Parameter | Value | Rationale |
|---|---|---|
| Table length L (x) | 0.36 m, x ∈ [0.20, 0.56] | top-down grasps reachable for r ≲ 0.55 m at grasp height; starts clear of the base |
| Table width W (y)  | 0.80 m, y ∈ [−0.40, 0.40] | holds spawn area + tray; corners deliberately exceed reach (stress profile) |
| Table height H     | 0.20 m | at TCP z ≈ 0.12 m top-down poses exist only in a thin ring r ≈ 0.45–0.62 m; at z ≈ 0.2–0.36 m the full disc r ≲ 0.5 m is reachable, so grasp (0.23 m) and transit (0.36 m) heights both lie in the dexterous region |
| Spawn | x ∈ [0.25, 0.50], y ∈ [−0.12, 0.34], **r ∈ [0.25, 0.50]**, yaw ∈ [−π, π], ≥ 6 cm from tray | reach band at pre-grasp height |
| Tray | centre (0.32, −0.28), inner 14×14 cm, walls 5 cm | inside the reach band, out of the spawn area |

**Franka Panda**

| Parameter | Value | Rationale |
|---|---|---|
| Table length L (x) | 0.50 m, x ∈ [0.25, 0.75] | top-down poses reachable to r ≈ 0.75 m at grasp height |
| Table width W (y)  | 0.90 m, y ∈ [−0.45, 0.45] | spawn area + tray; outer corners exceed reach (stress profile) |
| Table height H     | 0.10 m | unlike HEAL, the Panda reaches top-down poses down to table level over the whole disc (r ≲ 0.75 m) at TCP z = 0.12–0.26 m, so a low table keeps the arm well inside its dexterous region |
| Spawn | x ∈ [0.33, 0.68], y ∈ [−0.15, 0.38], **r ∈ [0.33, 0.70]**, yaw ∈ [−π, π], ≥ 6 cm from tray | reach band with margin |
| Tray | centre (0.45, −0.30), inner 14×14 cm, walls 5 cm | inside the reach band, out of the spawn area |

## 6. Logs (per episode, `episodes.csv` / `.jsonl`)

`seed, cube_x, cube_y, cube_yaw, success, failure_reason, failure_phase, failure_detail, plan_time_s, plan_ik_iters,
ik_calls, ik_iters_total/mean/max, ik_time_total_s, ik_time_mean_ms, ik_nonconverged, ik_joint_limit_violations,
ik_vel_violations, sim_joint_limit_violations, sim_joint_vel_violations, min_clear_cube_tray_m,
min_clear_gripper_table_m, min_clear_gripper_tray_m, max_track_err_m, grasp_yaw, place_yaw, sim_time_s, wall_time_s`.
Clearances are signed distances from `mj_geomDistance` (finger pads vs table/tray, cube vs tray walls).
`summary.json` aggregates them; `failures/` holds a PNG + full state (`.npz`) at the failure instant.

Runs made after the alternate-yaw search was added (all Franka runs) also log `grasp_yaw_attempts`. **Failure labels:** `ik_infeasible`, `joint_limit`, `place_infeasible`, `collision_table`, `collision_tray`
(robot/fingers vs tray), `collision_cube_tray` (cube vs wall before release), `collision_arm_cube`,
`grasp_failed` (pads not both on cube, or cube not lifted), `grasp_slip` (cube drops >3 cm in hand),
`tracking_error` (TCP >3 cm from target), `place_missed` (cube not resting inside tray).

## 7. Cube–tray collision avoidance

1. **Transit altitude from the held cube, not the TCP:** cube bottom must clear the tray wall top by 4 cm.
2. **Above-then-down:** carry to directly above the tray centre, then descend vertically → the cube's swept volume never meets a wall.
3. **Place yaw snapped to the tray axes** (multiple of π/2 nearest the grasp yaw) → cube faces parallel to walls.
4. **Footprint check** before descending (rotated cube extent + 5 mm margin must fit the inner tray).
5. **Monitoring:** any cube–wall contact before release fails the episode; min clearance is logged.

## 8. Results (`logs/`, `results/`)

**Nominal profile, 25 episodes, seeds 0–24** (runs executed one after another;
full table in `results/robot_comparison.md`):

| Robot | Method | Success | Plan time | IK time/call | Plan IK iters | Min clr cube–tray | Mean max track err | Sim vel.-limit steps |
|---|---|---|---|---|---|---|---|---|
| HEAL   | Mink | 25/25 | 9.8 ms  | 0.16 ms | 97.3 | 3.2 cm | 1.8 mm | 0 |
| HEAL   | DLS  | 25/25 | 4.9 ms  | 0.09 ms | 93.0 | 3.2 cm | 1.8 mm | 0 |
| HEAL   | QP   | 25/25 | 6.7 ms  | 0.10 ms | 97.5 | 3.2 cm | 1.8 mm | 0 |
| Franka | Mink | 25/25 | 6.6 ms  | 0.15 ms | 66.0 | 4.4 cm | 8.6 mm | 319 |
| Franka | DLS  | 25/25 | 4.2 ms  | 0.08 ms | 77.2 | 4.3 cm | 8.6 mm | 0 |
| Franka | QP   | 25/25 | 4.6 ms  | 0.10 ms | 66.3 | 4.4 cm | 8.6 mm | 321 |

**Stress profile (whole table), 40 episodes, seeds 100–139** – identical outcome for all three methods:
* HEAL 37/40 (92.5 %): 1× `ik_infeasible` (r ≈ 0.62 m), 2× `collision_tray` (cube next to the tray).
* Franka 29/40 (72.5 %): 1× `ik_infeasible` (r ≈ 0.81 m), 5× `collision_table` (all at r > 0.72 m – the finger hull
  touches the table during the descent at full extension), 5× `collision_tray` (hand body hits a tray wall when
  the cube spawns next to the tray).

**Ablation (`--naive-tray`, nominal seeds):** 0/25 for every method and both robots
(HEAL: 21× `collision_cube_tray`, 4× `collision_tray`; Franka: 22× / 3×); min cube–wall clearance 0.03 cm vs
3.2 cm (HEAL) / 4.3 cm (Franka) with avoidance → `results/<robot>/ablation_tray_avoidance.png`.

*Note:* all runs were executed one after another on the same machine, so timings are comparable between runs.

### Discussion
* On identical seeds all three solvers produce identical task outcomes on both robots – failures come from
  reachability and geometry, not from the IK method. They differ in cost: closed-form DLS (one linear solve per
  iteration) is fastest; QP adds a solver call (+10–35 %); Mink is ≈ 1.6–2× DLS due to its general task/limit framework.
* While streaming at 50 Hz the targets move < 1 cm per tick, so every method converges in ≤ 1 iteration per call;
  iteration differences appear in planning (large home → waypoint jumps). On the redundant Panda, Mink and QP need
  fewer planning iterations (66) than DLS (77) – the DLS null-space term pulls towards mid-range and slows convergence.
* **Velocity limits on the Panda:** Mink and QP produced ≈ 320 simulation steps above 2 rad/s, DLS none. Both enforce
  the limit per IK *iteration*, but several iterations can run inside one 50 Hz tick, so the commanded per-tick motion can
  exceed the limit (mostly wrist rotation); DLS's null-space joint centring gives a smoother wrist path. Mitigation:
  bound the total Δq per tick, or run one QP step per tick.
* **Joint limits:** HEAL's joints are ±π, so limits never matter there. On the Panda the planned solutions stayed inside
  the limits for every method in the nominal region (0 violations) – thanks to the alternate-grasp-yaw search, which
  discards yaws whose chain would leave the joint range.
* **Tracking:** Franka's position servos have no gravity feed-forward, so mean max tracking error is 8.6 mm (HEAL with
  torque control + gravity compensation: 1.8 mm). The same sag explains the far-reach `collision_table` stress failures;
  enabling gravity compensation (`gravcomp` on the links) would reduce both.

## 9. Failure analysis & mitigations (found during development, mostly HEAL)

| Observed failure | Cause | Mitigation implemented |
|---|---|---|
| Cube flew off at spawn | arm XML has `inertiafromgeom="false"` → cube COM at a wrong offset | explicit cube inertia + `ipos = 0` |
| `collision_table` while closing | 2F-85 fingertips swing down in an arc | grasp 14 mm above cube centre (`grasp_depth`) |
| Cube wedged out of the pinch | mesh collisions use convex hulls, much fatter than real fingers | collision bitmasks: cube touches only the pads; finger hulls still collide with table/tray |
| `grasp_slip` at start of place descent (8/16) | cube pivots about the pad axis under soft contact | `condim=6` (torsional + rolling friction), noslip solver, 25 N grip cap → 16/16 |
| `ik_infeasible` in table corners | outside the top-down reach band | spawn restricted to r ∈ [0.25, 0.50]; could add a tilted approach |
| `collision_tray` next to the tray | fingers open over the tray wall | tray keep-out in spawn; could choose a grasp yaw that opens the fingers parallel to the wall |
| `collision_cube_tray` (naive) | diagonal approach drags the cube through the wall | §7 avoidance strategy |
| Panda: some grasp yaws unreachable (−y side) | joint-7 limit ±2.9 rad | alternate grasp-yaw search (all 4 symmetric yaws) |
| Panda: `collision_table` at r > 0.72 m (stress) | servo sag at full extension, finger hull reaches the table | keep spawn r ≤ 0.70 m; add gravity compensation or a few mm of grasp height |
