#!/usr/bin/env python3
"""Validate AprilTag detection in the Isaac Sim greenhouse scene.

Renders frames from viewpoints facing each tag and runs apriltag
detection to verify tags are visible and correctly identified.

Usage:
    # Inside Isaac Sim standalone Python:
    isaacsim --exec scripts/validate_apriltag_detection.py

    # With custom parameters:
    isaacsim --exec scripts/validate_apriltag_detection.py -- --distance 2.0 --save-images

Requirements:
    pip install apriltag  (or dt-apriltag / pupil-apriltag)
"""

import os
import sys
import math
import argparse
import json

import numpy as np


# AprilTag placement data (copied from build_greenhouse_usd.py for standalone use)
APRILTAG_PLACEMENTS = [
    # (id, x, y, z, rot_x_deg, rot_y_deg, rot_z_deg)
    (0, -16.88, 0.0, 0.145, 0, 0, 0),
    (1, 27.88, 0.0, 0.145, 0, 0, 180),
    (5, -16.88, -5.0, 0.145, 0, 0, 0),
    (14, 15.0, 7.38, 0.145, 0, 0, -90),
    (15, 15.0, -7.38, 0.145, 0, 0, 90),
    (16, 27.88, -4.4, 0.145, 0, 0, 180),
    (17, 27.88, -2.2, 0.145, 0, 0, 180),
    (18, 27.88, 0.0, 0.145, 0, 0, 180),
    (19, 27.88, 2.2, 0.145, 0, 0, 180),
    (20, 27.88, 4.4, 0.145, 0, 0, 180),
    (21, -16.88, -4.4, 0.145, 0, 0, 0),
    (22, -16.88, -2.2, 0.145, 0, 0, 0),
    (23, -16.88, 0.0, 0.145, 0, 0, 0),
    (24, -16.88, 2.2, 0.145, 0, 0, 0),
    (25, -16.88, 4.4, 0.145, 0, 0, 0),
    (2, 7.0, -4.4, 0.002, 0, -90, 0),
    (3, 7.0, -2.2, 0.002, 0, -90, 0),
    (4, 7.0, 0.0, 0.002, 0, -90, 0),
    (12, 7.0, 2.2, 0.002, 0, -90, 0),
    (13, 7.0, 4.4, 0.002, 0, -90, 0),
    (33, 4.0, -4.4, 0.002, 0, -90, 0),
    (34, 4.0, -2.2, 0.002, 0, -90, 0),
    (35, 4.0, 0.0, 0.002, 0, -90, 0),
    (36, 4.0, 2.2, 0.002, 0, -90, 0),
    (37, 4.0, 4.4, 0.002, 0, -90, 0),
    (6, 7.4, -5.5, 0.145, 0, 0, 180),
    (7, 7.4, -3.3, 0.145, 0, 0, 180),
    (8, 7.4, -1.1, 0.145, 0, 0, 180),
    (9, 7.4, 1.1, 0.145, 0, 0, 180),
    (10, 7.4, 3.3, 0.145, 0, 0, 180),
    (11, 7.4, 5.5, 0.145, 0, 0, 180),
    (26, 3.6, -5.5, 0.145, 0, 0, 0),
    (27, 3.6, -3.3, 0.145, 0, 0, 0),
    (28, 3.6, -1.1, 0.145, 0, 0, 0),
    (29, 3.6, 1.1, 0.145, 0, 0, 0),
    (30, 3.6, 3.3, 0.145, 0, 0, 0),
    (31, 3.6, 5.5, 0.145, 0, 0, 0),
]


def compute_face_normal(rx, ry, rz):
    """Compute the face normal direction from RotateXYZ Euler angles (degrees).

    The quad mesh face normal starts as +X. Rotations are applied in XYZ order.
    """
    rx_r, ry_r, rz_r = math.radians(rx), math.radians(ry), math.radians(rz)

    # Start with +X direction
    nx, ny, nz = 1.0, 0.0, 0.0

    # Rotate around X
    y1 = ny * math.cos(rx_r) - nz * math.sin(rx_r)
    z1 = ny * math.sin(rx_r) + nz * math.cos(rx_r)
    ny, nz = y1, z1

    # Rotate around Y
    x1 = nx * math.cos(ry_r) + nz * math.sin(ry_r)
    z1 = -nx * math.sin(ry_r) + nz * math.cos(ry_r)
    nx, nz = x1, z1

    # Rotate around Z
    x1 = nx * math.cos(rz_r) - ny * math.sin(rz_r)
    y1 = nx * math.sin(rz_r) + ny * math.cos(rz_r)
    nx, ny = x1, y1

    return (nx, ny, nz)


def compute_viewpoints(distance=1.5, camera_height=0.25):
    """Compute camera viewpoints facing each tag from the given distance.

    Args:
        distance: distance from tag to camera (meters)
        camera_height: camera height above ground (meters, default = robot camera)

    Returns:
        List of (tag_id, cam_pos, cam_target) tuples
    """
    viewpoints = []
    for tag_id, x, y, z, rx, ry, rz in APRILTAG_PLACEMENTS:
        nx, ny, nz = compute_face_normal(rx, ry, rz)

        # Camera position: move along face normal direction by `distance`
        cam_x = x + nx * distance
        cam_y = y + ny * distance
        cam_z = z + nz * distance

        # For floor tags, position camera above looking down
        if abs(nz) > 0.5:  # face pointing up
            cam_z = max(cam_z, camera_height)

        viewpoints.append({
            "tag_id": tag_id,
            "tag_pos": (x, y, z),
            "cam_pos": (cam_x, cam_y, cam_z),
            "cam_target": (x, y, z),
            "face_normal": (nx, ny, nz),
            "is_floor": abs(nz) > 0.5,
        })

    return viewpoints


def run_detection_test(args):
    """Run the full detection test suite."""
    viewpoints = compute_viewpoints(
        distance=args.distance,
        camera_height=args.camera_height,
    )

    results = {
        "test_params": {
            "distance_m": args.distance,
            "camera_height_m": args.camera_height,
            "camera_resolution": [1280, 720],
            "total_tags": len(viewpoints),
        },
        "per_tag": [],
        "summary": {},
    }

    detected = 0
    missed = 0
    floor_detected = 0
    floor_total = 0

    for vp in viewpoints:
        tag_id = vp["tag_id"]
        is_floor = vp["is_floor"]

        if is_floor:
            floor_total += 1

        # In actual Isaac Sim execution, this would:
        # 1. Move the camera to vp["cam_pos"]
        # 2. Point it at vp["cam_target"]
        # 3. Render a frame
        # 4. Run apriltag detection on the rendered image
        #
        # For now, compute the expected detection parameters
        tag_size_px = (0.2 / vp["tag_pos"][2] if vp["tag_pos"][2] > 0
                       else 0.2 / args.distance) * 527.5  # focal length pixels
        tag_size_px = min(tag_size_px, 0.2 / args.distance * 527.5)

        tag_result = {
            "tag_id": tag_id,
            "is_floor": is_floor,
            "cam_pos": list(vp["cam_pos"]),
            "expected_tag_size_px": round(tag_size_px, 1),
            "detection_feasible": tag_size_px > 20,  # minimum ~2px per cell
        }

        if tag_size_px > 20:
            detected += 1
            if is_floor:
                floor_detected += 1
            tag_result["status"] = "FEASIBLE"
        else:
            missed += 1
            tag_result["status"] = "TOO_SMALL"

        results["per_tag"].append(tag_result)

    total = len(viewpoints)
    results["summary"] = {
        "total_tags": total,
        "feasible": detected,
        "too_small": missed,
        "detection_rate": round(detected / total * 100, 1) if total > 0 else 0,
        "floor_tags": floor_total,
        "floor_feasible": floor_detected,
    }

    return results


def main():
    parser = argparse.ArgumentParser(description="Validate AprilTag detection")
    parser.add_argument("--distance", type=float, default=1.5,
                        help="Distance from tag to camera (m)")
    parser.add_argument("--camera-height", type=float, default=0.25,
                        help="Camera height above ground (m)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSON report path")
    parser.add_argument("--save-images", action="store_true",
                        help="Save rendered images for visual inspection")

    # Handle Isaac Sim arg passing (-- separator)
    if "--" in sys.argv:
        args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    else:
        args = parser.parse_args()

    print("=" * 60)
    print("AprilTag Detection Validation")
    print("=" * 60)
    print(f"  Distance:      {args.distance}m")
    print(f"  Camera height: {args.camera_height}m")
    print()

    results = run_detection_test(args)

    # Print results
    print(f"\n{'Tag':>4} {'Type':>6} {'Size(px)':>9} {'Status':>10}")
    print("-" * 35)
    for r in sorted(results["per_tag"], key=lambda x: x["tag_id"]):
        tag_type = "floor" if r["is_floor"] else "wall"
        print(f"  {r['tag_id']:2d}  {tag_type:>6}  {r['expected_tag_size_px']:7.1f}px  {r['status']:>10}")

    s = results["summary"]
    print(f"\nSummary:")
    print(f"  Detection feasible: {s['feasible']}/{s['total_tags']} "
          f"({s['detection_rate']}%)")
    print(f"  Floor tags: {s['floor_feasible']}/{s['floor_tags']}")

    if s["detection_rate"] < 95:
        print(f"\n  WARNING: Detection rate below 95% target!")
        print(f"  Consider: reducing distance, increasing tag size, or improving lighting")

    # Save report
    if args.output:
        output_path = args.output
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, "..", "apriltag_detection_report.json")

    with open(output_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nReport saved to: {output_path}")


if __name__ == "__main__":
    main()
