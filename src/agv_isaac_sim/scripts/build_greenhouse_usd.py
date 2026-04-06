#!/usr/bin/env python3
"""Procedural USD world builder for the AGV greenhouse simulation.

Run inside Isaac Sim standalone Python:
    isaacsim --exec build_greenhouse_usd.py

Builds the greenhouse as a USD stage from config/world_config.yaml.
All geometry, materials, lighting, and physics are config-driven.
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom

from world.config import load_world_config, texture_path
from world.primitives import create_pbr_material
from world.physics_setup import create_physics_scene
from world.greenhouse_structure import (
    create_ground, create_walls, create_crop_rows, create_rails, create_roof_beams)
from world.greenhouse_objects import create_apriltags, create_obstacles
from world.greenhouse_lighting import create_lighting


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


def build_greenhouse(stage, cfg):
    """Build the complete greenhouse environment from config."""
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


def validate_textures(cfg):
    """Check all referenced textures exist before building."""
    print("\nValidating textures:")
    all_ok = True

    # Collect all texture references from config
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
        print("\n  WARNING: Some textures are missing! USD will use magenta fallback.")
        print(f"  Texture dir: {cfg['_textures_dir']}")
        print(f"  Tag tex dir: {cfg['_tag_textures_dir']}")
    else:
        print(f"  All {len(textures)} textures found.")

    return all_ok


def main():
    cfg = load_world_config()
    output = cfg["_output_path"]

    print(f"Building greenhouse USD world...")
    print(f"  Src dir:  {cfg['_src_dir']}")
    print(f"  Textures: {cfg['_textures_dir']}")
    print(f"  Tags:     {cfg['_tag_textures_dir']}")
    print(f"  Output:   {output}")

    validate_textures(cfg)

    stage = Usd.Stage.CreateNew(output)
    build_greenhouse(stage, cfg)
    stage.GetRootLayer().Save()

    print(f"Greenhouse USD saved to: {output}")
    simulation_app.close()


if __name__ == "__main__":
    main()
