# stress_smart

| Method               |   Episodes |   Success rate (%) |   Mean plan time (ms) |   IK time / call (ms) |   IK time / episode (s) |   Plan IK iters |   IK iters / call |   Max IK iters |   Min clr cube-tray (cm) |   Min clr gripper-table (cm) |   Mean max track err (mm) |   IK joint-limit viol. |   IK velocity viol. |   Sim vel-limit steps | Failures                          |
|:---------------------|-----------:|-------------------:|----------------------:|----------------------:|------------------------:|----------------:|------------------:|---------------:|-------------------------:|-----------------------------:|--------------------------:|-----------------------:|--------------------:|----------------------:|:----------------------------------|
| Mink (off-the-shelf) |         40 |               92.5 |                 10.9  |                 0.157 |                   0.062 |           107.9 |              0.75 |              1 |                     1.16 |                         0.25 |                      1.75 |                      0 |                   0 |                     0 | ik_infeasible:1, collision_tray:2 |
| DLS-IK               |         40 |               92.5 |                  5.64 |                 0.085 |                   0.034 |           108   |              0.75 |              1 |                     1.16 |                         0.25 |                      1.75 |                      0 |                   0 |                     0 | ik_infeasible:1, collision_tray:2 |
| QP-IK                |         40 |               92.5 |                  7.74 |                 0.103 |                   0.04  |           108.5 |              0.75 |              1 |                     1.16 |                         0.25 |                      1.75 |                      0 |                   0 |                     0 | ik_infeasible:1, collision_tray:2 |

Runs:
- dls: `logs/heal/dls_stress_20261007_203338`
- mink: `logs/heal/mink_stress_20261007_203316`
- qp: `logs/heal/qp_stress_20261007_203359`
