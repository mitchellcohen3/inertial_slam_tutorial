import os
import subprocess
import typing
import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns
import logging

from navlie.lib.imu import IMUState
from navlie.utils.plot import plot_poses

from pymlg import SO3, SE23

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


# class IMUState:
#     def __init__(
#         self,
#         stamp: float,
#         attitude: np.ndarray,
#         velocity: np.ndarray,
#         position: np.ndarray,
#         gyro_bias: np.ndarray,
#         accel_bias: np.ndarray,
#     ):
#         self.timestamp = stamp
#         self.attitude = attitude
#         self.velocity = velocity
#         self.position = position
#         self.gyro_bias = gyro_bias
#         self.accel_bias = accel_bias


def load_imu_states_from_asl(
    file_path: str,
) -> typing.List[IMUState]:
    """Loads IMU states from a text file in the ASL format."""
    data = np.loadtxt(file_path, delimiter=" ")
    imu_states = []
    for row in data:
        stamp = row[0]
        position = row[1:4]
        quat = row[4:8]
        velocity = row[8:11]
        bias_gyro = row[11:14]
        bias_accel = row[14:17]

        attitude = SO3.from_quat(quat, order="wxyz")
        nav_state = SE23.from_components(attitude, velocity, position)
        imu_state = IMUState(
            stamp=stamp,
            nav_state=nav_state,
            bias_gyro=bias_gyro,
            bias_accel=bias_accel,
        )
        imu_states.append(imu_state)
    return imu_states


def evaluate_ekf_slam_example(gt_file: str, est_file: str, cov_file: str):
    gt_states = load_imu_states_from_asl(gt_file)
    logging.info(f"Loaded {len(gt_states)} ground truth states from {gt_file}")

    # Plot the groundtruth states
    fig, ax = plot_poses(gt_states, label="groundtruth", step=None)
    ax.set_title("EKF SLAM Example - Groundtruth Trajectory")



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

    evaluate_ekf_slam_example(gt_file, est_file, cov_file)
    plt.show()
