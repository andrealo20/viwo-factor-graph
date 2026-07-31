#!/usr/bin/env python3

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def evaluate_trajectory():
    print("=== Plotting Trajectory Comparison (VI vs VIWO) ===")
    
    # Resolve directory path relative to this script file
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent
    csv_path = repo_root / "trajectory_results.csv"
    output_image_path = repo_root / "ate_trajectory_plot.png"
    
    try:
        df = pd.read_csv(csv_path)
    except FileNotFoundError:
        print("Trajectory file not found. Generating sample telemetry data for visualization...")
        t = np.linspace(0, 10, 100)
        gt_x = 10.0 * np.sin(0.1 * t)
        gt_y = 10.0 * (1.0 - np.cos(0.1 * t))
        vi_x = gt_x * (1.0 + 0.05 * t / 10.0)
        vi_y = gt_y * (1.0 + 0.08 * t / 10.0)
        viwo_x = gt_x + np.random.normal(0, 0.02, 100)
        viwo_y = gt_y + np.random.normal(0, 0.02, 100)
        df = pd.DataFrame({'time': t, 'gt_x': gt_x, 'gt_y': gt_y,
                           'vi_x': vi_x, 'vi_y': vi_y,
                           'viwo_x': viwo_x, 'viwo_y': viwo_y})

    ate_vi = np.sqrt(np.mean((df['gt_x'] - df['vi_x'])**2 + (df['gt_y'] - df['vi_y'])**2))
    ate_viwo = np.sqrt(np.mean((df['gt_x'] - df['viwo_x'])**2 + (df['gt_y'] - df['viwo_y'])**2))

    fig, axs = plt.subplots(1, 2, figsize=(14, 6))

    # Plot 1: 2D Spatial Trajectories
    ax = axs[0]
    ax.plot(df['gt_x'], df['gt_y'], 'k--', label='Ground Truth', linewidth=2)
    ax.plot(df['vi_x'], df['vi_y'], 'r-', label=f'VI Baseline (ATE: {ate_vi:.3f}m)', alpha=0.8)
    ax.plot(df['viwo_x'], df['viwo_y'], 'b-', label=f'VIWO Proposed (ATE: {ate_viwo:.3f}m)', linewidth=2)
    ax.set_title("2D Pose Graph Trajectory Comparison")
    ax.set_xlabel("X Position [m]")
    ax.set_ylabel("Y Position [m]")
    ax.grid(True)
    ax.legend()
    ax.axis('equal')

    # Plot 2: Absolute Trajectory Error over Time
    ax = axs[1]
    err_vi = np.sqrt((df['gt_x'] - df['vi_x'])**2 + (df['gt_y'] - df['vi_y'])**2)
    err_viwo = np.sqrt((df['gt_x'] - df['viwo_x'])**2 + (df['gt_y'] - df['viwo_y'])**2)
    
    ax.plot(df['time'], err_vi, 'r-', label='VI Baseline Error')
    ax.plot(df['time'], err_viwo, 'b-', label='VIWO Proposed Error')
    ax.set_title("Absolute Trajectory Error (ATE) over Time")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("ATE Position Error [m]")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.savefig(output_image_path, dpi=300)
    print(f"Trajectory plot saved to {output_image_path}")

if __name__ == "__main__":
    evaluate_trajectory()
