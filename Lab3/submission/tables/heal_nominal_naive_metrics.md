# nominal_naive

| Method               |   Episodes |   Success rate (%) |   Mean plan time (ms) |   IK time / call (ms) |   IK time / episode (s) |   Plan IK iters |   IK iters / call |   Max IK iters |   Min clr cube-tray (cm) |   Min clr gripper-table (cm) |   Mean max track err (mm) |   IK joint-limit viol. |   IK velocity viol. |   Sim vel-limit steps | Failures                                 |
|:---------------------|-----------:|-------------------:|----------------------:|----------------------:|------------------------:|----------------:|------------------:|---------------:|-------------------------:|-----------------------------:|--------------------------:|-----------------------:|--------------------:|----------------------:|:-----------------------------------------|
| Mink (off-the-shelf) |         25 |                  0 |                  8.87 |                 0.171 |                   0.043 |            81   |              0.79 |              1 |                     0.03 |                         0.25 |                      1.13 |                      0 |                   0 |                     0 | collision_cube_tray:21, collision_tray:4 |
| DLS-IK               |         25 |                  0 |                  4.51 |                 0.101 |                   0.026 |            77.4 |              0.79 |              1 |                     0.03 |                         0.25 |                      1.22 |                      0 |                   0 |                     0 | collision_cube_tray:21, collision_tray:4 |
| QP-IK                |         25 |                  0 |                  5.83 |                 0.108 |                   0.027 |            81.2 |              0.79 |              1 |                     0.03 |                         0.25 |                      1.09 |                      0 |                   0 |                     0 | collision_cube_tray:21, collision_tray:4 |

Runs:
- dls: `logs/heal/dls_nominal_naive_20261007_203535`
- mink: `logs/heal/mink_nominal_naive_20261007_203524`
- qp: `logs/heal/qp_nominal_naive_20261007_203545`
