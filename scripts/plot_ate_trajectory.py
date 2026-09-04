#!/usr/bin/env python3
"""
Plot the benchmark trajectories and their absolute trajectory error.

Reads the CSV that ``viwo_benchmark`` writes and draws the two panels used in
the README. It has no fallback: if the CSV is not there the script says so and
stops. An earlier version generated synthetic "sample telemetry" instead, which
produced a figure carrying invented ATE values and nothing to distinguish it
from a real result. In a repository whose point is that its numbers can be
reproduced, that is worse than no figure at all.

The benchmark writes into its working directory, which is normally the build
tree, so both the repository root and the usual build directories are searched.

Usage:
    python3 scripts/plot_ate_trajectory.py [trajectory_results.csv]
"""

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

CSV_NAME = "trajectory_results.csv"
SEARCH_DIRS = (".", "build", "build-release", "build-debug", "cmake-build-debug")


def find_csv(explicit):
    """Locate the benchmark output, or explain where it was looked for."""
    if explicit:
        if os.path.isfile(explicit):
            return explicit
        raise SystemExit(f"{explicit}: not found")

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tried = []
    for d in SEARCH_DIRS:
        candidate = os.path.join(repo_root, d, CSV_NAME)
        tried.append(os.path.normpath(candidate))
        if os.path.isfile(candidate):
            return candidate

    raise SystemExit(
        f"{CSV_NAME} not found. Looked in:\n"
        + "".join(f"    {t}\n" for t in tried)
        + "Run the benchmark first:\n"
        "    cmake -S . -B build && cmake --build build\n"
        "    cd build && ./viwo_benchmark\n"
    )


def load(path):
    """Read the trajectory CSV into named float arrays."""
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"{path}: no rows")

    required = ["time", "gt_x", "gt_y", "gt_z",
                "vi_x", "vi_y", "vi_z", "viwo_x", "viwo_y", "viwo_z"]
    missing = [c for c in required if c not in rows[0]]
    if missing:
        raise SystemExit(f"{path}: missing columns {', '.join(missing)}")

    return {c: np.array([float(r[c]) for r in rows]) for c in required}


def ate(d, prefix):
    """Root mean square 3D position error, the same quantity the benchmark reports."""
    err = np.sqrt((d["gt_x"] - d[f"{prefix}_x"]) ** 2
                  + (d["gt_y"] - d[f"{prefix}_y"]) ** 2
                  + (d["gt_z"] - d[f"{prefix}_z"]) ** 2)
    return err, float(np.sqrt(np.mean(err ** 2)))


def main():
    path = find_csv(sys.argv[1] if len(sys.argv) > 1 else None)
    print(f"reading {path}")
    d = load(path)

    err_vi, ate_vi = ate(d, "vi")
    err_viwo, ate_viwo = ate(d, "viwo")
    print(f"  VI baseline    ATE RMSE {ate_vi:.3f} m")
    print(f"  VIWO proposed  ATE RMSE {ate_viwo:.3f} m")

    fig, axs = plt.subplots(1, 2, figsize=(14, 6))

    ax = axs[0]
    ax.plot(d["gt_x"], d["gt_y"], "k--", label="Ground truth", linewidth=2)
    ax.plot(d["vi_x"], d["vi_y"], "r-", alpha=0.8,
            label=f"VI baseline (ATE {ate_vi:.3f} m)")
    ax.plot(d["viwo_x"], d["viwo_y"], "b-", linewidth=2,
            label=f"VIWO proposed (ATE {ate_viwo:.3f} m)")
    ax.set_title("Trajectory in the ground plane")
    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.grid(True)
    ax.legend()
    ax.axis("equal")

    ax = axs[1]
    ax.plot(d["time"], err_vi, "r-", label="VI baseline")
    ax.plot(d["time"], err_viwo, "b-", label="VIWO proposed")
    ax.set_title("Absolute trajectory error over time")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position error [m]")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(repo_root, "docs")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "ate_trajectory_plot.png")
    plt.savefig(out, dpi=300)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
