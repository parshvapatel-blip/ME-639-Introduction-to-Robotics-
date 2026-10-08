# HEAL vs Franka - nominal profile

| Robot   | Method               |   Episodes |   Success rate (%) |   Mean plan time (ms) |   IK time / call (ms) |   Plan IK iters |   Min clr cube-tray (cm) |   Mean max track err (mm) |   IK joint-limit viol. | Failures   |
|:--------|:---------------------|-----------:|-------------------:|----------------------:|----------------------:|----------------:|-------------------------:|--------------------------:|-----------------------:|:-----------|
| HEAL    | Mink (off-the-shelf) |         25 |                100 |                  9.77 |                 0.158 |            97.3 |                     3.24 |                      1.84 |                      0 | -          |
| HEAL    | DLS-IK               |         25 |                100 |                  4.91 |                 0.086 |            93   |                     3.23 |                      1.84 |                      0 | -          |
| HEAL    | QP-IK                |         25 |                100 |                  6.75 |                 0.102 |            97.5 |                     3.23 |                      1.84 |                      0 | -          |
| FRANKA  | Mink (off-the-shelf) |         25 |                100 |                  6.58 |                 0.149 |            66   |                     4.41 |                      8.62 |                      0 | -          |
| FRANKA  | DLS-IK               |         25 |                100 |                  4.22 |                 0.084 |            77.2 |                     4.3  |                      8.6  |                      0 | -          |
| FRANKA  | QP-IK                |         25 |                100 |                  4.59 |                 0.098 |            66.3 |                     4.41 |                      8.62 |                      0 | -          |
