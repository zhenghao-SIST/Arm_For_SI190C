#!/usr/bin/env python3
"""
File: plot_fk.py
Author: Zhenghao Li
Email: lizhenghao@shanghaitech.edu.cn
Institute: SIST
Created: 2026-07-16

Description: Reads the CSV file saved by fk_subscriber and plots the
             end-effector trajectory (3D position + orientation over time).
             Given 3 target points forming a triangle, the FK data points
             are compared against the 3 edges and the RMS error is reported.
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("TkAgg")  # interactive backend — change to "Agg" for headless
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
# Default triangle vertices (from pose_publisher.cpp waypoints)
# ---------------------------------------------------------------------------
DEFAULT_POINTS = [
                  (0.101, -0.112, 0.164),
                  (0.025, -0.137, 0.147),
                  (0.081, -0.103, 0.139),
]


# ---------------------------------------------------------------------------
# CSV loader
# ---------------------------------------------------------------------------
def load_csv(path: str) -> dict:
    """Load fk_poses.csv and return columns as numpy arrays."""
    if not os.path.exists(path):
        print(f"Error: file not found — {path}")
        sys.exit(1)

    cols = {
        "sec": [], "nsec": [],
        "x": [], "y": [], "z": [],
        "qx": [], "qy": [], "qz": [], "qw": [],
    }

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k in cols:
                cols[k].append(float(row[k]))

    for k in cols:
        cols[k] = np.array(cols[k])

    # Build elapsed time in seconds from the first sample
    t_ns = (cols["sec"] - cols["sec"][0]) * 1e9 + (cols["nsec"] - cols["nsec"][0])
    cols["t"] = t_ns / 1e9  # seconds
    return cols


# ---------------------------------------------------------------------------
# Argument parsing helpers
# ---------------------------------------------------------------------------
def parse_points(s: str) -> list[tuple[float, float, float]]:
    """Parse a semicolon-separated string of 3 points.

    Format: "x1,y1,z1;x2,y2,z2;x3,y3,z3"
    """
    parts = s.split(";")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"Expected 3 points separated by ';', got {len(parts)}"
        )
    points = []
    for i, p in enumerate(parts):
        vals = p.split(",")
        if len(vals) != 3:
            raise argparse.ArgumentTypeError(
                f"Point {i + 1}: expected 'x,y,z', got '{p}'"
            )
        try:
            points.append(tuple(float(v) for v in vals))
        except ValueError:
            raise argparse.ArgumentTypeError(
                f"Point {i + 1}: non-numeric value in '{p}'"
            )
    return points


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def point_to_segment_distance(
    p: np.ndarray, a: np.ndarray, b: np.ndarray
) -> float:
    """Shortest distance from point p to line segment ab (in 3D)."""
    ab = b - a
    ap = p - a
    t = np.dot(ap, ab) / np.dot(ab, ab) if np.dot(ab, ab) > 1e-15 else 0.0
    t = np.clip(t, 0.0, 1.0)
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def compute_errors(
    data: dict, triangle_vertices: list[tuple[float, float, float]]
) -> dict:
    """Compute per-point distance to nearest triangle edge and RMS.

    Returns a dict with keys: 'distances' (np.array), 'rms', 'max', 'mean'.
    """
    a = np.array(triangle_vertices[0])
    b = np.array(triangle_vertices[1])
    c = np.array(triangle_vertices[2])

    # triangle edges: A→B, B→C, C→A
    segments = [(a, b), (b, c), (c, a)]

    x = data["x"]
    y = data["y"]
    z = data["z"]
    n = len(x)

    distances = np.empty(n)
    for i in range(n):
        p = np.array([x[i], y[i], z[i]])
        # distance to nearest of the 3 edges
        d = min(point_to_segment_distance(p, s, e) for s, e in segments)
        distances[i] = d

    rms = float(np.sqrt(np.mean(distances ** 2)))
    return {
        "distances": distances,
        "rms": rms,
        "max": float(np.max(distances)),
        "mean": float(np.mean(distances)),
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_3d_trajectory(
    data: dict,
    triangle_vertices: list[tuple[float, float, float]],
    errors: dict,
    output: str | None,
):
    """3D scatter + line plot of the end-effector position, with triangle edges."""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    x, y, z = data["x"], data["y"], data["z"]
    d = errors["distances"]

    # ---- triangle edges ----
    tv = triangle_vertices
    # close the loop: A→B→C→A
    loop = np.array([tv[0], tv[1], tv[2], tv[0]])
    ax.plot(
        loop[:, 0], loop[:, 1], loop[:, 2],
        "k--", linewidth=2, alpha=0.7, label="target triangle edges",
    )
    # vertex markers
    labels = ["A", "B", "C"]
    for i, (vx, vy, vz) in enumerate(tv):
        ax.scatter(vx, vy, vz, c="black", s=100, marker="D", zorder=6)
        ax.text(vx, vy, vz, f"  {labels[i]}", fontsize=11, fontweight="bold",
                color="black", zorder=6)

    # ---- FK trajectory line ----
    ax.plot(x, y, z, "b-", linewidth=1, alpha=0.6, label="FK trajectory")

    # ---- color-coded scatter by distance-to-edge (error) ----
    sc = ax.scatter(x, y, z, c=d, cmap="hot_r", s=20, alpha=0.8, vmin=0)

    # ---- start / end markers ----
    ax.scatter(x[0],  y[0],  z[0],  c="green", s=80, marker="o",
               label="start", zorder=5)
    ax.scatter(x[-1], y[-1], z[-1], c="red",   s=80, marker="s",
               label="end",   zorder=5)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(
        f"FK End-Effector Trajectory vs Triangle\n"
        f"RMS = {errors['rms']*1000:.2f} mm  |  "
        f"max = {errors['max']*1000:.2f} mm  |  "
        f"mean = {errors['mean']*1000:.2f} mm"
    )
    ax.legend(loc="upper left")
    cbar = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label("Distance to nearest edge (m)")

    if output:
        fig.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Saved 3D trajectory plot to {output}")
    else:
        plt.show()


def plot_error_vs_time(data: dict, errors: dict, output: str | None):
    """Distance-to-edge over time."""
    fig, ax = plt.subplots(figsize=(12, 4))

    t = data["t"]
    d_mm = errors["distances"] * 1000  # convert to mm

    ax.plot(t, d_mm, linewidth=1, color="tab:red")
    ax.axhline(y=errors["rms"] * 1000, color="black", linestyle="--",
               linewidth=1, label=f"RMS = {errors['rms']*1000:.2f} mm")
    ax.fill_between(t, 0, d_mm, alpha=0.15, color="tab:red")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to nearest edge (mm)")
    ax.set_title("FK Position Error vs Time (distance to nearest triangle edge)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if output:
        err_out = output.replace(".png", "_err.png") if output.endswith(".png") else output + "_err.png"
        fig.savefig(err_out, dpi=150, bbox_inches="tight")
        print(f"Saved error time-series to {err_out}")
    else:
        plt.show()


def plot_position_vs_time(data: dict, output: str | None):
    """Position components over time."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    t = data["t"]
    for ax, comp, label in zip(
        axes, [data["x"], data["y"], data["z"]], ["X", "Y", "Z"]
    ):
        ax.plot(t, comp, linewidth=1)
        ax.set_ylabel(f"{label} (m)")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title("End-Effector Position vs Time")
    fig.tight_layout()

    if output:
        pos_out = output.replace(".png", "_pos.png") if output.endswith(".png") else output + "_pos.png"
        fig.savefig(pos_out, dpi=150, bbox_inches="tight")
        print(f"Saved position time-series to {pos_out}")
    else:
        plt.show()


def plot_orientation_vs_time(data: dict, output: str | None):
    """Orientation quaternion components over time."""
    fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True)

    t = data["t"]
    for ax, comp, label in zip(
        axes,
        [data["qx"], data["qy"], data["qz"], data["qw"]],
        ["qx", "qy", "qz", "qw"],
    ):
        ax.plot(t, comp, linewidth=1)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title("End-Effector Orientation (Quaternion) vs Time")
    fig.tight_layout()

    if output:
        ori_out = output.replace(".png", "_ori.png") if output.endswith(".png") else output + "_ori.png"
        fig.savefig(ori_out, dpi=150, bbox_inches="tight")
        print(f"Saved orientation time-series to {ori_out}")
    else:
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    default_str = ";".join(f"{x},{y},{z}" for x, y, z in DEFAULT_POINTS)

    parser = argparse.ArgumentParser(
        description="Plot FK end-effector data and compute RMS error "
                    "against a target triangle."
    )
    parser.add_argument(
        "csv", nargs="?", default="/tmp/fk_poses.csv",
        help="Path to the CSV file (default: /tmp/fk_poses.csv)"
    )
    parser.add_argument(
        "-p", "--points", type=parse_points, default=DEFAULT_POINTS,
        help=f"3 triangle vertices: 'x1,y1,z1;x2,y2,z2;x3,y3,z3' "
             f"(default: {default_str})"
    )
    parser.add_argument(
        "-o", "--output", type=str, default=None,
        help="Save plots to file(s) instead of displaying (e.g. fk_plot.png)"
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Do not display plots (useful with -o)"
    )
    args = parser.parse_args()

    data = load_csv(args.csv)
    print(f"Loaded {len(data['t'])} FK pose records "
          f"(duration: {data['t'][-1]:.2f}s)")

    triangle = args.points
    print(f"Triangle vertices: A={triangle[0]}  B={triangle[1]}  C={triangle[2]}")

    errors = compute_errors(data, triangle)
    print(f"\n  RMS  = {errors['rms']*1000:.3f} mm  "
          f"({errors['rms']:.6f} m)")
    print(f"  Max  = {errors['max']*1000:.3f} mm")
    print(f"  Mean = {errors['mean']*1000:.3f} mm\n")

    plot_3d_trajectory(data, triangle, errors, args.output)
    plot_error_vs_time(data, errors, args.output)
    plot_position_vs_time(data, args.output)
    plot_orientation_vs_time(data, args.output)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
