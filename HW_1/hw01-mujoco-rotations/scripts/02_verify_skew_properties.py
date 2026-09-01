"""
02_verify_skew_properties.py -- HW1 Part 2, Task 2: verify the
skew-symmetric identities from Problem 5 in simulation.

STARTER CODE. Model loading and a simple "spin the body" simulation
loop are provided and working. Your job is to fill in the TODOs to:

  1. Log R(t), the body's rotation matrix, at several simulated
     time steps while it spins.
  2. At each logged time step, numerically check, for several
     random v, w, omega in R^3:
         R (v x w) == (R v) x (R w)                 [Problem 5a]
         R w^ R^T  == (R w)^                         [Problem 5b, No-AI on paper]
     using utils.hat() for the ^ operator.
  3. Print the residual (it should be ~1e-14, machine precision)
     and explain in your write-up why a small-but-nonzero residual
     doesn't fully validate the identity, while a residual near
     machine epsilon strongly supports it.

Note: you already proved these identities by hand in Problem 5.
This script is not a substitute for that proof -- it's a numerical
sanity check, and a chance to see *why* proofs and simulation are
complementary, not interchangeable.
"""

import numpy as np
import mujoco
import csv

from utils import hat, get_body_orientation, is_close_to_identity

MODEL_PATH = "../model/asymmetric_body.xml"

N_CHECKS_PER_STEP = 5     # how many random (v, w, omega) triples per logged step
N_LOGGED_STEPS = 5        # how many simulated time points to check
STEPS_BETWEEN_LOGS = 200  # sim steps to advance between each logged check


def random_unit_angular_velocity(rng):
    """A random constant angular velocity vector (rad/s), used to spin
    the body between logged checks."""
    w = rng.normal(size=3)
    return 2.0 * w / np.linalg.norm(w)


def check_identities(R, rng):
    max_resid_cross = 0.0
    max_resid_skew = 0.0

    for _ in range(N_CHECKS_PER_STEP):
        v = rng.normal(size=3)
        w = rng.normal(size=3)
        omega = rng.normal(size=3)

        # Identity 1: R(v x w) == (Rv) x (Rw)
        lhs_cross = R @ np.cross(v, w)
        rhs_cross = np.cross(R @ v, R @ w)
        resid_cross = np.linalg.norm(lhs_cross - rhs_cross)
        max_resid_cross = max(max_resid_cross, resid_cross)

        # Identity 2: R w^ R^T == (Rw)^
        lhs_skew = R @ hat(omega) @ R.T
        rhs_skew = hat(R @ omega)
        resid_skew = np.max(np.abs(lhs_skew - rhs_skew))
        max_resid_skew = max(max_resid_skew, resid_skew)

    return max_resid_cross, max_resid_skew


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed=0)

    # Spin the body with a fixed angular velocity
    data.qvel[3:6] = random_unit_angular_velocity(rng)
    mujoco.mj_forward(model, data)
    results = []

    print(f"{'step':>5} {'t (s)':>8} {'max resid: R(vxw)=(Rv)x(Rw)':>28} {'max resid: RwR^T=(Rw)^':>24}")
    for log_i in range(N_LOGGED_STEPS):
        for _ in range(STEPS_BETWEEN_LOGS):
            mujoco.mj_step(model, data)

        R = get_body_orientation(data)
        assert is_close_to_identity(R @ R.T, tol=1e-6), "R is not orthonormal!"

        # Calculate residuals before logging/appending
        resid_cross, resid_skew = check_identities(R, rng)
        results.append((log_i, data.time, resid_cross, resid_skew))

        print(f"{log_i:5d} {data.time:8.3f} {resid_cross:28.3e} {resid_skew:24.3e}")

    # Corrected indentation (4 spaces)
    with open("skew_check_residuals.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "t", "resid_cross", "resid_skew"])
        writer.writerows(results)


if __name__ == "__main__":
    main()
