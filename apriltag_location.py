#!/usr/bin/env python3
import cv2
import numpy as np
import pandas as pd
from pathlib import Path
from pupil_apriltags import Detector

RGB_DIR = "./rtabmap_rgb_export/rtabmap_rgb"
POSES_FILE = "./rtabmap_rgb_export/rtabmap_camera_poses.txt"
OUTPUT_CSV = "./rtabmap_rgb_export/apriltag_results.csv"

TAG_SIZE = 0.5  # tag plane size in Gazebo, meters

# Update these for your CURRENT camera resolution/intrinsics
FX = 320.255
FY = 320.255
CX = 320.5
CY = 240.5

TAG_FAMILY = "tag36h11"
TARGET_TAG_ID = None   # keep None while debugging
UPSCALE = 2.0          # helps tiny tags a bit
DEBUG_PRINT_ALL = True


def quat_to_rot(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3, dtype=np.float64)

    q = q / n
    x, y, z, w = q

    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R


def load_poses_by_id(file_path):
    poses = {}

    with open(file_path, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) != 9:
                print("Skipping line {} (unexpected {} columns): {}".format(
                    line_num, len(parts), line
                ))
                continue

            try:
                # Format:
                # timestamp x y z qx qy qz qw id
                timestamp = float(parts[0])   # not used for matching
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                qx = float(parts[4])
                qy = float(parts[5])
                qz = float(parts[6])
                qw = float(parts[7])
                pose_id = int(parts[8])
            except ValueError:
                print("Skipping line {} (parse error): {}".format(line_num, line))
                continue

            R = quat_to_rot(qx, qy, qz, qw)

            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]

            poses[pose_id] = T

    return poses


def solve_tag_pose(corners, tag_size, fx, fy, cx, cy, upscale=1.0):
    s = tag_size / 2.0

    obj_pts = np.array([
        [-s, -s, 0.0],
        [ s, -s, 0.0],
        [ s,  s, 0.0],
        [-s,  s, 0.0]
    ], dtype=np.float64)

    img_pts = np.array(corners, dtype=np.float64)

    # If image was upscaled before detection, corners are in upscaled pixels
    fx_u = fx * upscale
    fy_u = fy * upscale
    cx_u = cx * upscale
    cy_u = cy * upscale

    K = np.array([
        [fx_u, 0.0, cx_u],
        [0.0, fy_u, cy_u],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    dist = np.zeros((5, 1), dtype=np.float64)

    flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)

    ok, rvec, tvec = cv2.solvePnP(
        obj_pts,
        img_pts,
        K,
        dist,
        flags=flag
    )

    if not ok:
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            K,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)

    T_camera_tag = np.eye(4, dtype=np.float64)
    T_camera_tag[:3, :3] = R
    T_camera_tag[:3, 3] = tvec.reshape(3)

    return T_camera_tag


def main():
    poses = load_poses_by_id(POSES_FILE)
    print("Loaded poses:", len(poses))

    detector = Detector(
        families=TAG_FAMILY,
        nthreads=1,            # safer than 4, helps avoid exit segfaults
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.5
    )

    results = []

    rgb_files = sorted(
        list(Path(RGB_DIR).glob("*.jpg")) +
        list(Path(RGB_DIR).glob("*.png")) +
        list(Path(RGB_DIR).glob("*.jpeg"))
    )

    print("Found images:", len(rgb_files))

    num_with_detections = 0
    num_missing_pose = 0
    num_pose_success = 0

    for img_path in rgb_files:
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if UPSCALE != 1.0:
            gray = cv2.resize(
                gray,
                None,
                fx=UPSCALE,
                fy=UPSCALE,
                interpolation=cv2.INTER_CUBIC
            )

        detections = detector.detect(gray, estimate_tag_pose=False)

        if DEBUG_PRINT_ALL:
            print("{} -> {} detections".format(img_path.name, len(detections)))
            for det in detections:
                print("   id={}, margin={:.2f}".format(
                    int(det.tag_id), float(det.decision_margin))
                )

        if len(detections) == 0:
            continue

        num_with_detections += 1

        try:
            image_id = int(img_path.stem)
        except ValueError:
            continue

        if image_id not in poses:
            print("No pose for image id {}".format(image_id))
            num_missing_pose += 1
            continue

        T_map_camera = poses[image_id]

        for det in detections:
            tag_id = int(det.tag_id)

            if TARGET_TAG_ID is not None and tag_id != TARGET_TAG_ID:
                continue

            T_camera_tag = solve_tag_pose(
                det.corners,
                TAG_SIZE,
                FX, FY, CX, CY,
                upscale=UPSCALE
            )

            if T_camera_tag is None:
                continue

            T_map_tag = T_map_camera @ T_camera_tag
            x, y, z = T_map_tag[:3, 3]

            print("{} | tag {} | map = ({:.3f}, {:.3f}, {:.3f})".format(
                img_path.name, tag_id, x, y, z
            ))

            results.append([
                img_path.name,
                image_id,
                tag_id,
                float(det.decision_margin),
                x, y, z
            ])
            num_pose_success += 1

    df = pd.DataFrame(
        results,
        columns=["image", "image_id", "tag_id", "decision_margin", "x", "y", "z"]
    )
    df.to_csv(OUTPUT_CSV, index=False)

    print("\nSaved results to", OUTPUT_CSV)
    print("Images with detections:", num_with_detections)
    print("Detections with missing poses:", num_missing_pose)
    print("Successful tag poses:", num_pose_success)

    if len(df) > 0:
        med = df[["x", "y", "z"]].median()
        print("\nFinal AprilTag position (median):")
        print("x = {:.3f}".format(med["x"]))
        print("y = {:.3f}".format(med["y"]))
        print("z = {:.3f}".format(med["z"]))
    else:
        print("No results saved.")


if __name__ == "__main__":
    main()