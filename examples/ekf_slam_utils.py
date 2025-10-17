import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import logging
import typing

from navlie.lib.imu import IMUState
from navlie.utils.plot import plot_poses

from pymlg import SO3, SE23


def load_covariances_from_file(file: str, dof: int) -> typing.List[np.ndarray]:
    """Loads the full covariances from a file, where each row of the file is
    assumed to have a timestmap and the covariance entries in column major order."""
    cov_mats_file_np = np.loadtxt(file, delimiter=" ")
    cov_mats: typing.List[np.ndarray] = []
    stamps: typing.List[float] = []
    for row in cov_mats_file_np:
        cov_mats.append(row[1:].reshape((dof, dof), order="F"))
        stamps.append(row[0])

    return cov_mats, stamps


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


def minus_SE23(Y: np.ndarray, X: np.ndarray, lie_direction: str) -> np.ndarray:
    """Computes the minus operation for SE23, given a lie direction"""
    if lie_direction == "left":
        return SE23.Log(Y @ SE23.inverse(X))
    elif lie_direction == "right":
        return SE23.Log(SE23.inverse(X) @ Y)


def evaluate_ekf_slam_example(gt_file: str, est_file: str, cov_file: str):
    gt_states = load_imu_states_from_asl(gt_file)
    est_states = load_imu_states_from_asl(est_file)
    covs, stamps = load_covariances_from_file(cov_file, dof=15)

    logging.info(f"Loaded {len(gt_states)} ground truth states from {gt_file}")
    logging.info(f"Loaded {len(est_states)} estimated states from {est_file}")
    logging.info(f"Loaded {len(covs)} covariance matrices from {cov_file}")

    # ensure that the lengths match
    assert len(gt_states) == len(est_states) == len(covs) == len(stamps)

    # Plot the groundtruth states
    fig, ax = plot_poses(gt_states, label="groundtruth", step=None)
    plot_poses(est_states, label="estimated", step=None, ax=ax)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("EKF SLAM State Estimation")

    # Compute the errors and 3-sigma bounds
    full_nees: np.ndarray = np.zeros(len(gt_states))
    att_nees: np.ndarray = np.zeros(len(gt_states))
    pos_nees: np.ndarray = np.zeros(len(gt_states))

    # Compute the errors in the Lie algebra
    delta_xi = []
    three_sigma = []
    stamps = []

    idx = 0
    for x_gt, x_est, cov in zip(
        gt_states,
        est_states,
        covs,
    ):
        error = np.zeros(15)

        # Compute the error in the Lie algebra differently depending on
        # the state representation
        X_nav_gt = SE23.from_components(
            x_gt.attitude,
            x_gt.velocity,
            x_gt.position,
        )
        X_nav_est = SE23.from_components(
            x_est.attitude,
            x_est.velocity,
            x_est.position,
        )
        error_nav = minus_SE23(X_nav_gt, X_nav_est, "left")
        error[0:9] = error_nav.ravel()

        error[9:12] = x_gt.bias_gyro - x_est.bias_gyro
        error[12:15] = x_gt.bias_accel - x_est.bias_accel

        delta_xi.append(error)

        # Compute the NEES values
        full_nees[idx] = error @ np.linalg.solve(cov, error)
        att_nees[idx] = error[0:3] @ np.linalg.solve(cov[0:3, 0:3], error[0:3])
        pos_nees[idx] = error[6:9] @ np.linalg.solve(cov[6:9, 6:9], error[6:9])

        # Compute the 3-sigma bounds
        three_sigma.append(3.0 * np.sqrt(np.diag(cov)))
        stamps.append(x_gt.stamp)
        idx += 1

    delta_xi = np.array(delta_xi)
    three_sigma = np.array(three_sigma)
    stamps = np.array(stamps)
    plot_three_sigma(stamps, delta_xi, three_sigma)


def plot_three_sigma(
    stamps: np.ndarray,
    error: np.ndarray,
    three_sigma: np.ndarray,
    ax: plt.Axes = None,
    error_color=None,
    enable_sigma_bounds: bool = True,
    error_alpha: float = 0.5,
    three_sigma_color: str = "black",
    error_label: str = None,
) -> typing.Tuple[plt.Figure, plt.Axes]:
    """Plots the error and three sigma bounds directly."""

    dim = error.shape[1]

    if dim < 3:
        n_rows = dim
    else:
        n_rows = 3

    n_cols = int(np.ceil(dim / 3))

    if ax is None:
        fig, ax = plt.subplots(n_rows, n_cols, sharex=True)
    else:
        fig: plt.Figure = ax.ravel("F")[0].get_figure()

    if error_color is None:
        error_color = "tab:blue"

    ax_og = ax
    # Plot the error and three sigma bounds
    ax = ax.ravel("F")
    for i in range(three_sigma.shape[1]):
        ax[i].plot(
            stamps,
            error[:, i],
            label=error_label,
            color=error_color,
            alpha=error_alpha,
        )
        if enable_sigma_bounds:
            ax[i].plot(
                stamps,
                three_sigma[:, i],
                color=three_sigma_color,
                linewidth=1.5,
                linestyle="--",
            )
            ax[i].plot(
                stamps,
                -three_sigma[:, i],
                color=three_sigma_color,
                linewidth=1.5,
                linestyle="--",
            )

        if i == 0 and error_label is not None:
            ax[i].legend()

    return fig, ax_og
