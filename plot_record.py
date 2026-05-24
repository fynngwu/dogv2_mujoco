#!/usr/bin/env python3
"""Plot recorded DOF pos vs target DOF pos from a CSV file.

Usage:
    python plot_record.py                    # latest CSV in records/
    python plot_record.py records/record_xxx.csv
"""
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import pandas as pd

JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_knee",
    "FR_hip", "FR_thigh", "FR_knee",
    "RL_hip", "RL_thigh", "RL_knee",
    "RR_hip", "RR_thigh", "RR_knee",
]


def find_latest_csv(directory="records"):
    d = Path(directory)
    if not d.exists():
        raise FileNotFoundError(f"Directory '{directory}' does not exist")
    csvs = sorted(d.glob("record_*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No record_*.csv files found in '{directory}'")
    return str(csvs[-1])


def plot_csv(csv_path):
    df = pd.read_csv(csv_path)
    n = len([c for c in df.columns if c.startswith("dof_pos_")])
    step = df["step"].values

    fig, axes = plt.subplots(4, 3, figsize=(15, 10), sharex=True)
    fig.suptitle(f"Recording: {Path(csv_path).name}", fontsize=14)

    for i in range(n):
        row, col = divmod(i, 3)
        ax = axes[row][col]
        ax.plot(step, df[f"dof_pos_{i}"], label="dof_pos", linewidth=1)
        ax.plot(step, df[f"target_dof_pos_{i}"], label="target", linestyle="--", linewidth=1)
        name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"joint_{i}"
        ax.set_title(name)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    for ax in axes[-1]:
        ax.set_xlabel("Step")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot recorded joint data")
    parser.add_argument("csv", nargs="?", help="Path to CSV file (default: latest in records/)")
    args = parser.parse_args()
    csv_path = args.csv or find_latest_csv()
    print(f"Loading: {csv_path}")
    plot_csv(csv_path)


if __name__ == "__main__":
    main()
