# nominal_smart

| Method               |   Episodes |   Success rate (%) |   Mean plan time (ms) |   IK time / call (ms) |   IK time / episode (s) |   Plan IK iters |   IK iters / call |   Max IK iters |   Min clr cube-tray (cm) |   Min clr gripper-table (cm) |   Mean max track err (mm) |   IK joint-limit viol. |   IK velocity viol. |   Sim vel-limit steps | Failures   |
|:---------------------|-----------:|-------------------:|----------------------:|----------------------:|------------------------:|----------------:|------------------:|---------------:|-------------------------:|-----------------------------:|--------------------------:|-----------------------:|--------------------:|----------------------:|:-----------|
| Mink (off-the-shelf) |         25 |                100 |                  9.77 |                 0.158 |                   0.062 |            97.3 |              0.77 |              1 |                     3.24 |                         0.25 |                      1.84 |                      0 |                   0 |                     0 | -          |
| DLS-IK               |         25 |                100 |                  4.91 |                 0.086 |                   0.034 |            93   |              0.77 |              1 |                     3.23 |                         0.25 |                      1.84 |                      0 |                   0 |                     0 | -          |
| QP-IK                |         25 |                100 |                  6.75 |                 0.102 |                   0.04  |            97.5 |              0.77 |              1 |                     3.23 |                         0.25 |                      1.84 |                      0 |                   0 |                     0 | -          |

Runs:
- dls: `logs/heal/dls_nominal_20261007_203153`
- mink: `logs/heal/mink_nominal_20261007_203138`
- qp: `logs/heal/qp_nominal_20261007_203207`
