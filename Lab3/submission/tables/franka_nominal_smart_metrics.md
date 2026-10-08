# nominal_smart

| Method               |   Episodes |   Success rate (%) |   Mean plan time (ms) |   IK time / call (ms) |   IK time / episode (s) |   Plan IK iters |   IK iters / call |   Max IK iters |   Min clr cube-tray (cm) |   Min clr gripper-table (cm) |   Mean max track err (mm) |   IK joint-limit viol. |   IK velocity viol. |   Sim vel-limit steps | Failures   |
|:---------------------|-----------:|-------------------:|----------------------:|----------------------:|------------------------:|----------------:|------------------:|---------------:|-------------------------:|-----------------------------:|--------------------------:|-----------------------:|--------------------:|----------------------:|:-----------|
| Mink (off-the-shelf) |         25 |                100 |                  6.58 |                 0.149 |                   0.069 |            66   |              0.77 |              9 |                     4.41 |                         0.21 |                      8.62 |                      0 |                   0 |                   319 | -          |
| DLS-IK               |         25 |                100 |                  4.22 |                 0.084 |                   0.039 |            77.2 |              0.76 |              1 |                     4.3  |                         0.21 |                      8.6  |                      0 |                   0 |                     0 | -          |
| QP-IK                |         25 |                100 |                  4.59 |                 0.098 |                   0.045 |            66.3 |              0.77 |              8 |                     4.41 |                         0.21 |                      8.62 |                      0 |                   0 |                   321 | -          |

Runs:
- dls: `logs/franka/dls_nominal_20261007_203234`
- mink: `logs/franka/mink_nominal_20261007_203221`
- qp: `logs/franka/qp_nominal_20261007_203246`
