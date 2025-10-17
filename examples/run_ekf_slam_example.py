import os
import subprocess
import typing
import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns
import logging

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

def run_ekf_slam_example(executable_path: str):
    if not os.path.exists(executable_path):
        raise FileNotFoundError(f"Executable not found at {executable_path}")
    logging.info(f"Running EKF SLAM example from {executable_path}")

    try:
        result = subprocess.run([executable_path], check=True)
    except subprocess.CalledProcessError as e:
        logging.error(f"Execution failed: {e.stderr}")
        raise

if __name__ == "__main__":
    executable_path = os.path.join(cur_dir, "../build/examples/ekf_slam_example")
    run_ekf_slam_example(executable_path)
    