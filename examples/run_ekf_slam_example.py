import os
import subprocess
import typing
import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns
import logging

from ekf_slam_utils import evaluate_ekf_slam_example

logging.basicConfig(level=logging.INFO)

cur_dir = os.path.dirname(os.path.abspath(__file__))

# Plot settings
sns.set_theme(style="whitegrid")
plt.rc("lines", linewidth=2)
plt.rc("axes", grid=True)
plt.rc("grid", linestyle="--")
# plt.rcParams.update({"font.size": 14})
plt.rc("text", usetex=True)
colors = sns.color_palette("deep")

def run_ekf_slam_example(executable_path: str, config_dict: typing.Dict):
    if not os.path.exists(executable_path):
        raise FileNotFoundError(f"Executable not found at {executable_path}")
    logging.info(f"Running EKF SLAM example from {executable_path}")

    try:
        cmd = [executable_path]

        args = [
            "--trajectory_path",
            config_dict["traj_path"],
            "--state_gt_path",
            config_dict["state_gt_path"],
            "--state_est_path",
            config_dict["state_est_path"],
            "--cov_est_path",
            config_dict["cov_est_path"],
            "--feature_map_path",
            config_dict["feature_map_path"],
        ]
        cmd.extend(args)
        result = subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        logging.error(f"Execution failed: {e.stderr}")
        raise


if __name__ == "__main__":
    executable_path = os.path.join(cur_dir, "../build/examples/ekf_slam_example")

    save_dir = os.path.join(cur_dir, "output")
    os.makedirs(save_dir, exist_ok=True)

    est_file = os.path.join(save_dir, "state_est.txt")
    cov_file = os.path.join(save_dir, "cov_est.txt")
    gt_file = os.path.join(save_dir, "state_gt.txt")
    feature_map_file = os.path.join(save_dir, "feature_map.txt")
    traj_path = os.path.join(cur_dir, "trajectories/euroc_mav/MH_01_easy.txt")

    config_dict = {
        "traj_path": traj_path,
        "state_gt_path": gt_file,
        "state_est_path": est_file,
        "cov_est_path": cov_file,
        "feature_map_path": feature_map_file,
    }

    run_ekf_slam_example(executable_path, config_dict)

    # Evaluate and plot results
    evaluate_ekf_slam_example(gt_file, est_file, cov_file, feature_map_file)
    plt.show()
