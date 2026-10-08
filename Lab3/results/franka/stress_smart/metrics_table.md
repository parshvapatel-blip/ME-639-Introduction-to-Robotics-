# stress_smart

| Method               |   Episodes |   Success rate (%) |   Mean plan time (ms) |   IK time / call (ms) |   IK time / episode (s) |   Plan IK iters |   IK iters / call |   Max IK iters |   Min clr cube-tray (cm) |   Min clr gripper-table (cm) |   Mean max track err (mm) |   IK joint-limit viol. |   IK velocity viol. |   Sim vel-limit steps | Failures                                             |
|:---------------------|-----------:|-------------------:|----------------------:|----------------------:|------------------------:|----------------:|------------------:|---------------:|-------------------------:|-----------------------------:|--------------------------:|-----------------------:|--------------------:|----------------------:|:-----------------------------------------------------|
| Mink (off-the-shelf) |         40 |               72.5 |                  8.65 |                 0.156 |                   0.062 |            84.9 |              0.75 |             20 |                     0.77 |                         0.09 |                      9.24 |                      0 |                   0 |                   232 | collision_table:5, ik_infeasible:1, collision_tray:5 |
| DLS-IK               |         40 |               72.5 |                  4.77 |                 0.082 |                   0.032 |            88.6 |              0.75 |              1 |                     0.77 |                         0.09 |                      9.27 |                      0 |                   0 |                     0 | collision_table:5, ik_infeasible:1, collision_tray:5 |
| QP-IK                |         40 |               72.5 |                  5.9  |                 0.097 |                   0.039 |            84.4 |              0.75 |             20 |                     0.77 |                         0.09 |                      9.25 |                      0 |                   0 |                   228 | collision_table:5, ik_infeasible:1, collision_tray:5 |

Runs:
- dls: `logs/franka/dls_stress_20261007_203439`
- mink: `logs/franka/mink_stress_20261007_203421`
- qp: `logs/franka/qp_stress_20261007_203455`
