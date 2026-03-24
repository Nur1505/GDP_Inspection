#!/usr/bin/env python3
import os
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_occupancy_map(json_file):
    with open(json_file, "r") as f:
        d = json.load(f)

    width = int(d["width"])
    height = int(d["height"])
    resolution = float(d["resolution"])
    ox = float(d["origin"]["x"])
    oy = float(d["origin"]["y"])

    data = np.array(d["data"], dtype=np.int16).reshape((height, width))
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": ox,
        "origin_y": oy,
        "data": data
    }


def build_display_map(raw):
    img = np.full(raw.shape, 0.5, dtype=np.float32)  # unknown gray
    img[raw == 0] = 1.0                              # free white
    img[raw > 0] = 0.0                               # occupied black
    return img


def load_csv_xy(csv_file):
    df = pd.read_csv(csv_file)
    return df


def transform_map_to_world(point_map, map_origin_file):
    """
    Translation-only conversion for now.
    If later needed, upgrade to full rotation transform.
    """
    with open(map_origin_file, "r") as f:
        origin = json.load(f)

    wx = float(origin["x"]) + float(point_map["x"])
    wy = float(origin["y"]) + float(point_map["y"])
    wz = float(origin["z"]) + float(point_map.get("z", 0.0))
    return {"x": wx, "y": wy, "z": wz}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_json", required=True)
    parser.add_argument("--uav1_gt_csv", required=True)
    parser.add_argument("--uav1_est_csv", required=True)
    parser.add_argument("--foi_detected_map_json", required=True)
    parser.add_argument("--map_origin_json", required=True)
    parser.add_argument("--foi_gt_x", type=float, default=None)
    parser.add_argument("--foi_gt_y", type=float, default=None)
    parser.add_argument("--uav2_csv", default=None)
    parser.add_argument("--out", default="trajectory_map_foi.png")
    parser.add_argument("--title", default="Trajectory vs Map vs FOI")
    args = parser.parse_args()

    m = load_occupancy_map(args.map_json)
    raw = m["data"]
    img = build_display_map(raw)

    ox = m["origin_x"]
    oy = m["origin_y"]
    res = m["resolution"]
    width = m["width"]
    height = m["height"]

    extent = [
        ox,
        ox + width * res,
        oy,
        oy + height * res
    ]

    gt_df = load_csv_xy(args.uav1_gt_csv)
    est_df = load_csv_xy(args.uav1_est_csv)

    with open(args.foi_detected_map_json, "r") as f:
        foi_map = json.load(f)

    foi_world = transform_map_to_world(foi_map, args.map_origin_json)

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(img, cmap="gray", origin="lower", extent=extent, interpolation="nearest")

    if {"x", "y"}.issubset(gt_df.columns):
        ax.plot(gt_df["x"].values, gt_df["y"].values, linewidth=2, label="UAV1 Ground Truth")

    if {"x", "y"}.issubset(est_df.columns):
        ax.plot(est_df["x"].values, est_df["y"].values, linewidth=2, label="UAV1 Estimated")

    ax.scatter([foi_world["x"]], [foi_world["y"]], s=100, marker="x", label="Detected FOI")

    if args.foi_gt_x is not None and args.foi_gt_y is not None:
        ax.scatter([args.foi_gt_x], [args.foi_gt_y], s=100, marker="o", label="FOI Ground Truth")

    if args.uav2_csv is not None and os.path.exists(args.uav2_csv):
        uav2_df = pd.read_csv(args.uav2_csv)
        if {"x", "y"}.issubset(uav2_df.columns):
            ax.plot(uav2_df["x"].values, uav2_df["y"].values, linewidth=2, label="UAV2 Path")

    ax.set_title(args.title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(args.out, dpi=200)
    print("Saved:", args.out)
    plt.show()


if __name__ == "__main__":
    main()