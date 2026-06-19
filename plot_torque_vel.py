#!/usr/bin/env python3
"""Plot dof_vel vs torque scatter plots for 12 joints with theoretical motor TN curves.

Usage:
    python plot_torque_vel.py                          # latest CSV in records/
    python plot_torque_vel.py records/record_xxx.csv
"""
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_knee",
    "FR_hip", "FR_thigh", "FR_knee",
    "RL_hip", "RL_thigh", "RL_knee",
    "RR_hip", "RR_thigh", "RR_knee",
]

KNEE_INDICES = [2, 5, 8, 11]

RPM_TO_RAD_S = 2.0 * np.pi / 60.0


def motor_torque_curve_from_rpm(motor_rpm, max_torque=17.0, knee_rpm=80.0, max_rpm=210.0):
    motor_rpm = np.asarray(motor_rpm, dtype=np.float64)
    torque = np.zeros_like(motor_rpm)
    flat_mask = motor_rpm <= knee_rpm
    slope_mask = (motor_rpm > knee_rpm) & (motor_rpm <= max_rpm)
    torque[flat_mask] = max_torque
    torque[slope_mask] = max_torque * (max_rpm - motor_rpm[slope_mask]) / (max_rpm - knee_rpm)
    return np.clip(torque, 0.0, None)


def joint_side_torque_curve(joint_rad_s, gear_ratio=1.0, max_torque=17.0, knee_rpm=80.0, max_rpm=210.0):
    joint_rad_s = np.asarray(joint_rad_s, dtype=np.float64)
    motor_rpm = np.abs(joint_rad_s) * gear_ratio / RPM_TO_RAD_S
    motor_torque = motor_torque_curve_from_rpm(motor_rpm, max_torque=max_torque, knee_rpm=knee_rpm, max_rpm=max_rpm)
    return motor_torque * gear_ratio


def find_latest_csv(directory="records"):
    d = Path(directory)
    if not d.exists():
        raise FileNotFoundError(f"Directory '{directory}' does not exist")
    csvs = sorted(d.glob("record_*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No record_*.csv files found in '{directory}'")
    return str(csvs[-1])


def plot_csv(csv_path, max_torque, knee_rpm, max_rpm, knee_gear_ratio):
    df = pd.read_csv(csv_path)
    n = len([c for c in df.columns if c.startswith("dof_vel_")])

    if n == 0:
        raise ValueError("No dof_vel columns found. Please re-record data with the updated code.")

    fig, axes = plt.subplots(4, 3, figsize=(15, 10))
    fig.suptitle(f"Torque vs DOF Velocity: {Path(csv_path).name}", fontsize=14)

    for i in range(n):
        row, col = divmod(i, 3)
        ax = axes[row][col]
        vel = df[f"dof_vel_{i}"].values
        torque = df[f"torque_{i}"].values
        ax.scatter(abs(vel), abs(torque), s=2, alpha=0.5, label="recorded")

        gr = knee_gear_ratio if i in KNEE_INDICES else 1.0
        max_vel = rpm_to_rad_s(max_rpm) / gr * 1.1
        speed = np.linspace(0.0, max_vel, 500)
        tn = joint_side_torque_curve(speed, gear_ratio=gr, max_torque=max_torque, knee_rpm=knee_rpm, max_rpm=max_rpm)
        ax.plot(speed, tn, linewidth=2.0, color="red", label="TN curve")

        name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"joint_{i}"
        ax.set_title(name)
        ax.set_xlabel("|dof_vel| (rad/s)")
        ax.set_ylabel("|torque| (Nm)")
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


def rpm_to_rad_s(rpm):
    return np.asarray(rpm, dtype=np.float64) * RPM_TO_RAD_S


def main():
    parser = argparse.ArgumentParser(description="Plot torque vs DOF velocity scatter with motor TN curves")
    parser.add_argument("csv", nargs="?", help="Path to CSV file (default: latest in records/)")
    parser.add_argument("--max-torque", type=float, default=17.0)
    parser.add_argument("--knee-rpm", type=float, default=80.0)
    parser.add_argument("--max-rpm", type=float, default=210.0)
    parser.add_argument("--knee-gear-ratio", type=float, default=1.667)
    args = parser.parse_args()
    csv_path = args.csv or find_latest_csv()
    print(f"Loading: {csv_path}")
    plot_csv(csv_path, args.max_torque, args.knee_rpm, args.max_rpm, args.knee_gear_ratio)


if __name__ == "__main__":
    main()
