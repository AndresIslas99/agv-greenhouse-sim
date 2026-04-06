#!/usr/bin/env python3
"""Build complete greenhouse simulation: world + robot in a single USD.

Run inside Isaac Sim standalone Python:
    isaacsim --exec build_greenhouse_with_robot.py

This is the unified build script that produces the definitive
`greenhouse_with_robot.usd` containing:
- Greenhouse environment (ground, walls, crop rows, rails, beams, tags, lights)
- Robot (URDF-imported, diff-drive, ZED 2i cameras, IMU)
- PhysX scene with calibrated physics materials
- All sensor OmniGraph configurations

Output: worlds/greenhouse_with_robot.usd
"""

import os
import sys

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom

# World modules
from world.config import load_world_config, texture_path
from world.primitives import create_pbr_material
from world.physics_setup import create_physics_scene
from world.greenhouse_structure import (
    create_ground, create_walls, create_crop_rows, create_rails, create_roof_beams)
from world.greenhouse_objects import create_apriltags, create_obstacles
from world.greenhouse_lighting import create_lighting

# Robot modules
from robot.config import TOPICS, OUTPUT_PATH, TEMP_URDF
from robot.urdf_import import process_xacro, import_urdf
from robot.diff_drive import setup_diff_drive_controller, setup_odometry_publisher
from robot.joint_states import setup_joint_state_publisher
from robot.sensors import setup_camera, setup_imu
from robot.physics_materials import setup_physics_materials


def create_materials(stage, cfg):
    """Create all PBR materials from config."""
    root = "/World/Materials"
    mat = {}

    def _mat(name, mat_cfg):
        tex = texture_path(cfg, mat_cfg["texture"]) if mat_cfg.get("texture") else ""
        return create_pbr_material(stage, f"{root}/{name}", tex,
                                   metallic=mat_cfg.get("metallic", 0.0),
                                   roughness=mat_cfg.get("roughness", 0.8))

    mat["soil"] = _mat("Soil", cfg["ground"]["material"])
    mat["wall"] = _mat("Polycarbonate", cfg["walls"]["material"])
    mat["crop_a"] = _mat("CropLeaves", cfg["crop_rows"]["materials"][0])
    mat["crop_b"] = _mat("CropLeavesAlt", cfg["crop_rows"]["materials"][1])
    mat["rail"] = _mat("Steel", cfg["rails"]["material"])
    mat["crate"] = _mat("CrateWood", cfg["obstacles"]["material"])
    mat["beam"] = _mat("SteelBeam", cfg["roof_beams"]["material"])
    return mat


def validate_textures(cfg):
    """Check all referenced textures exist before building."""
    print("\nValidating textures:")
    all_ok = True
    textures = []

    for section in ["ground", "walls", "rails", "obstacles"]:
        mat = cfg[section].get("material", {})
        if mat.get("texture"):
            textures.append(("textures", mat["texture"]))
    for mat in cfg["crop_rows"].get("materials", []):
        if mat.get("texture"):
            textures.append(("textures", mat["texture"]))
    beam_mat = cfg["roof_beams"].get("material", {})
    if beam_mat.get("texture"):
        textures.append(("textures", beam_mat["texture"]))
    for tag_cfg in cfg["apriltags"]["tags"]:
        textures.append(("tags", f"tag36h11_id{tag_cfg['id']}.png"))

    for kind, filename in textures:
        if kind == "textures":
            path = os.path.join(cfg["_textures_dir"], filename)
        else:
            path = os.path.join(cfg["_tag_textures_dir"], filename)
        exists = os.path.exists(path)
        status = "OK" if exists else "MISSING"
        print(f"  [{status}] {filename} -> {path}")
        if not exists:
            all_ok = False

    if not all_ok:
        print("\n  WARNING: Some textures missing! Surfaces will render MAGENTA.")
    else:
        print(f"  All {len(textures)} textures found.")
    return all_ok


def build_world(stage, cfg):
    """Build the greenhouse environment."""
    print("\n=== Building greenhouse world ===")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    materials = create_materials(stage, cfg)
    create_physics_scene(stage, cfg)
    create_ground(stage, materials, cfg)
    create_walls(stage, materials, cfg)
    create_crop_rows(stage, materials, cfg)
    create_rails(stage, materials, cfg)
    create_roof_beams(stage, materials, cfg)
    create_apriltags(stage, materials, cfg)
    create_obstacles(stage, materials, cfg)
    create_lighting(stage, cfg)
    print("World complete.")


def build_robot(stage):
    """Import robot URDF and configure all sensors + actuators."""
    print("\n=== Building robot ===")

    # Import URDF
    urdf_path = process_xacro()
    robot_path = import_urdf(urdf_path)

    # Actuators
    setup_diff_drive_controller(robot_path)
    setup_odometry_publisher(robot_path)
    setup_joint_state_publisher(robot_path)

    # Left camera (depth + RGB + point cloud)
    setup_camera(stage, robot_path,
                 cam_name="ZedLeftCamera",
                 parent_link="zed_left_camera_frame",
                 frame_id="zed_left_camera_frame_optical",
                 topics={
                     "rgb": TOPICS["left_rgb"],
                     "depth": TOPICS["left_depth"],
                     "camera_info": TOPICS["left_camera_info"],
                     "pointcloud": TOPICS["left_pointcloud"],
                 },
                 enable_depth=True)

    # Right camera (RGB only)
    setup_camera(stage, robot_path,
                 cam_name="ZedRightCamera",
                 parent_link="zed_right_camera_frame",
                 frame_id="zed_right_camera_frame_optical",
                 topics={
                     "rgb": TOPICS["right_rgb"],
                     "camera_info": TOPICS["right_camera_info"],
                 },
                 enable_depth=False)

    # IMU
    setup_imu(stage, robot_path)

    # Physics materials
    setup_physics_materials(stage, robot_path)

    # Cleanup temp URDF
    if os.path.exists(TEMP_URDF):
        os.remove(TEMP_URDF)

    print("Robot complete.")


def main():
    cfg = load_world_config()
    output = os.path.join(cfg["_pkg_dir"], "worlds", "greenhouse_with_robot.usd")

    print("=" * 60)
    print("  AGV Greenhouse + Robot — Unified USD Builder")
    print("=" * 60)
    print(f"  Src dir:  {cfg['_src_dir']}")
    print(f"  Textures: {cfg['_textures_dir']}")
    print(f"  Tags:     {cfg['_tag_textures_dir']}")
    print(f"  Output:   {output}")

    validate_textures(cfg)

    # Build everything in one stage
    stage = Usd.Stage.CreateNew(output)
    omni.usd.get_context().attach_stage_with_callback(stage)

    build_world(stage, cfg)
    build_robot(stage)

    stage.GetRootLayer().Save()
    print(f"\n{'=' * 60}")
    print(f"  DONE: {output}")
    print(f"  Open this file in Isaac Sim to run the simulation.")
    print(f"{'=' * 60}")

    simulation_app.close()


if __name__ == "__main__":
    main()
