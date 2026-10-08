#!/usr/bin/env python3
"""
FRANKA pick and place - Off-the-shelf Mink IK (https://github.com/kevinzakka/mink): FrameTask + ConfigurationLimit + VelocityLimit -> MinkIK
(solver code: utils/pick_place/ik_solvers.py, robot settings: utils/pick_place/robots/franka.py)

Run from the repo root (Lab3/):
    python scripts/franka/ik_mink/run_mink_pick_place.py --episodes 25 --seed 0          # batch -> logs/franka/
    python scripts/franka/ik_mink/run_mink_pick_place.py -n 5 --viewer --realtime        # watch live
    python scripts/franka/ik_mink/run_mink_pick_place.py -n 5 --record --video-size 1280x720
    python scripts/franka/ik_mink/run_mink_pick_place.py -n 40 --seed 100 --profile stress
    python scripts/franka/ik_mink/run_mink_pick_place.py -n 25 --naive-tray              # ablation
Same --seed for every method -> identical cube poses.
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from utils.pick_place.runner import main  # noqa: E402

if __name__ == "__main__":
    main(method="mink", robot="franka")
