#!/usr/bin/env python3
"""Procedural USD world builder for the AGV greenhouse simulation.

Run inside Isaac Sim standalone Python:
    isaacsim --exec build_greenhouse_usd.py

Or from the Isaac Sim script editor.

Recreates greenhouse_simple.sdf as a USD stage with:
- Ground plane (34×19m, soil texture)
- 4 enclosure walls (3m high, polycarbonate)
- 6 crop rows (20×1.0×1.5m)
- 10 heating pipe rails (51mm cylinders)
- 10 roof beams (steel)
- 6 AprilTag markers (tag36h11 textures)
- 3 static crate obstacles
- 3 directional lights
- PhysX scene (1ms timestep, GPU broadphase)
"""

import os
import math

# Isaac Sim imports
from isaacsim import SimulationApp

# Launch headless Isaac Sim for USD generation
simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, UsdLux

# Resolve texture paths relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
WORLDS_PKG = os.path.join(os.path.dirname(PACKAGE_DIR), "agv_sim_worlds")
TEXTURES_DIR = os.path.join(WORLDS_PKG, "models", "textures")
TAG_TEXTURES_DIR = os.path.join(WORLDS_PKG, "materials", "textures")
OUTPUT_PATH = os.path.join(PACKAGE_DIR, "worlds", "greenhouse_simple.usd")


def create_physics_scene(stage):
    """Configure PhysX scene: 1ms timestep, GPU acceleration."""
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    # PhysX GPU settings
    physx_scene = stage.GetPrimAtPath("/World/PhysicsScene")
    physx_scene.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(True)
    physx_scene.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("GPU")
    physx_scene.CreateAttribute("physxScene:gpuMaxRigidContactCount", Sdf.ValueTypeNames.Int).Set(524288)
    physx_scene.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(1000)


def create_pbr_material(stage, mat_path, texture_file, metallic=0.0, roughness=0.8):
    """Create a PBR material with an albedo texture."""
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)

    if texture_file and os.path.exists(texture_file):
        tex_reader = UsdShade.Shader.Define(stage, f"{mat_path}/DiffuseTexture")
        tex_reader.CreateIdAttr("UsdUVTexture")
        tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_file)
        tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex_reader.ConnectableAPI(), "rgb")
    else:
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.5))

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def create_box(stage, path, size, position, rotation_deg=(0, 0, 0), material=None, is_static=True):
    """Create a box prim with collision and optional material."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))

    cube = UsdGeom.Cube.Define(stage, f"{path}/Mesh")
    cube.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))

    # Physics collision
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if is_static:
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        xform.GetPrim().CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)

    if material:
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

    return xform


def create_cylinder(stage, path, radius, height, position, rotation_deg=(0, 0, 0), material=None):
    """Create a cylinder prim with collision."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))

    cyl = UsdGeom.Cylinder.Define(stage, f"{path}/Mesh")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")

    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    if material:
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(material)

    return xform


def create_plane(stage, path, width, height, position, rotation_deg=(0, 0, 0), material=None):
    """Create a thin plane (quad) for AprilTag markers."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))

    # Use a very thin cube to represent a plane
    cube = UsdGeom.Cube.Define(stage, f"{path}/Mesh")
    cube.AddScaleOp().Set(Gf.Vec3f(0.001, width / 2.0, height / 2.0))

    if material:
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

    return xform


def build_greenhouse(stage):
    """Build the complete greenhouse environment."""

    # Root xform
    world = UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Physics scene
    create_physics_scene(stage)

    # ── Materials ──────────────────────────────────────────────
    materials_root = "/World/Materials"

    mat_soil = create_pbr_material(
        stage, f"{materials_root}/Soil",
        os.path.join(TEXTURES_DIR, "ground_soil.png"),
        roughness=0.95)

    mat_wall = create_pbr_material(
        stage, f"{materials_root}/Polycarbonate",
        os.path.join(TEXTURES_DIR, "wall_polycarbonate.png"),
        roughness=0.65)

    mat_crop_a = create_pbr_material(
        stage, f"{materials_root}/CropLeaves",
        os.path.join(TEXTURES_DIR, "crop_leaves.png"),
        roughness=0.9)

    mat_crop_b = create_pbr_material(
        stage, f"{materials_root}/CropLeavesAlt",
        os.path.join(TEXTURES_DIR, "crop_leaves_alt.png"),
        roughness=0.9)

    mat_rail = create_pbr_material(
        stage, f"{materials_root}/Steel",
        os.path.join(TEXTURES_DIR, "rail_steel.png"),
        metallic=0.3, roughness=0.6)

    mat_crate = create_pbr_material(
        stage, f"{materials_root}/CrateWood",
        os.path.join(TEXTURES_DIR, "crate_wood.png"),
        roughness=0.85)

    mat_beam = create_pbr_material(
        stage, f"{materials_root}/SteelBeam", "",
        metallic=0.5, roughness=0.4)

    # AprilTag materials (one per tag ID)
    tag_materials = {}
    for tag_id in range(6):
        tex_path = os.path.join(TAG_TEXTURES_DIR, f"tag36h11_id{tag_id}.png")
        tag_materials[tag_id] = create_pbr_material(
            stage, f"{materials_root}/AprilTag{tag_id}",
            tex_path, roughness=0.9)

    # ── Ground Plane ──────────────────────────────────────────
    create_box(stage, "/World/Ground",
               size=(34.0, 19.0, 0.1),
               position=(15.0, 0.0, -0.05),
               material=mat_soil)

    # ── Walls ─────────────────────────────────────────────────
    walls = UsdGeom.Xform.Define(stage, "/World/Walls")

    wall_specs = [
        ("North", (15.0, 7.5, 1.5), (30.0, 0.2, 3.0)),
        ("South", (15.0, -7.5, 1.5), (30.0, 0.2, 3.0)),
        ("West", (0.0, 0.0, 1.5), (0.2, 15.0, 3.0)),
        ("East", (30.0, 0.0, 1.5), (0.2, 15.0, 3.0)),
    ]
    for name, pos, size in wall_specs:
        create_box(stage, f"/World/Walls/{name}",
                   size=size, position=pos, material=mat_wall)

    # ── Crop Rows ─────────────────────────────────────────────
    rows = UsdGeom.Xform.Define(stage, "/World/CropRows")

    row_y_positions = [-5.5, -3.3, -1.1, 1.1, 3.3, 5.5]
    for i, y in enumerate(row_y_positions):
        mat = mat_crop_a if i % 2 == 0 else mat_crop_b
        create_box(stage, f"/World/CropRows/Row{i + 1}",
                   size=(20.0, 1.0, 1.5),
                   position=(16.0, y, 0.75),
                   material=mat)

    # ── Heating Pipe Rails ────────────────────────────────────
    rails = UsdGeom.Xform.Define(stage, "/World/Rails")

    aisle_centers_y = [-4.4, -2.2, 0.0, 2.2, 4.4]
    rail_offset = 0.225  # half of 0.45m spacing
    rail_radius = 0.0255  # 51mm diameter / 2

    for i, y_center in enumerate(aisle_centers_y):
        for side, offset in [("Left", -rail_offset), ("Right", rail_offset)]:
            create_cylinder(
                stage, f"/World/Rails/Aisle{i + 1}_{side}",
                radius=rail_radius, height=20.0,
                position=(16.0, y_center + offset, rail_radius),
                rotation_deg=(0.0, 90.0, 0.0),  # Rotate to lie along X
                material=mat_rail)

    # ── Roof Beams ────────────────────────────────────────────
    beams = UsdGeom.Xform.Define(stage, "/World/RoofBeams")

    # Transverse beams (Y-direction)
    for i, x in enumerate([5.0, 10.0, 15.0, 20.0, 25.0]):
        create_box(stage, f"/World/RoofBeams/Transverse{i + 1}",
                   size=(0.08, 15.0, 0.08),
                   position=(x, 0.0, 3.0),
                   material=mat_beam)

    # Longitudinal beams (X-direction)
    for i, y in enumerate([-3.5, 3.5]):
        create_box(stage, f"/World/RoofBeams/Longitudinal{i + 1}",
                   size=(30.0, 0.08, 0.08),
                   position=(15.0, y, 3.0),
                   material=mat_beam)

    # ── AprilTag Markers ──────────────────────────────────────
    tags = UsdGeom.Xform.Define(stage, "/World/AprilTags")

    # (id, x, y, z, yaw_degrees)
    tag_placements = [
        (0, 1.0, 0.0, 0.25, 90.0),     # West corridor, facing +X
        (1, 29.0, 0.0, 0.25, -90.0),    # East corridor, facing -X
        (2, 6.0, -4.4, 0.25, -90.0),    # Aisle 1-2 entrance
        (3, 6.0, 0.0, 0.25, -90.0),     # Aisle 3-4 entrance
        (4, 6.0, 4.4, 0.25, -90.0),     # Aisle 5-6 entrance
        (5, 2.5, -5.0, 0.25, 0.0),      # Starting area
    ]

    for tag_id, x, y, z, yaw in tag_placements:
        create_plane(stage, f"/World/AprilTags/Tag{tag_id}",
                     width=0.2, height=0.2,
                     position=(x, y, z),
                     rotation_deg=(0.0, 0.0, yaw),
                     material=tag_materials[tag_id])

    # ── Static Crate Obstacles ────────────────────────────────
    crates = UsdGeom.Xform.Define(stage, "/World/Crates")

    crate_specs = [
        ("Crate1", (3.5, -2.0, 0.15), (0.5, 0.4, 0.3), 17.2),   # 0.3 rad
        ("Crate2", (4.0, 3.5, 0.15), (0.5, 0.4, 0.3), -11.5),   # -0.2 rad
        ("Crate3", (25.0, 0.0, 0.15), (0.5, 0.4, 0.3), 28.6),   # 0.5 rad
    ]
    for name, pos, size, yaw_deg in crate_specs:
        create_box(stage, f"/World/Crates/{name}",
                   size=size, position=pos,
                   rotation_deg=(0.0, 0.0, yaw_deg),
                   material=mat_crate)

    # ── Lighting ──────────────────────────────────────────────
    lights = UsdGeom.Xform.Define(stage, "/World/Lights")

    # Sun (main directional, casts shadows)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 0.95))
    sun.CreateAngleAttr(0.53)  # Sun disk angle for soft shadows
    sun_xform = UsdGeom.Xformable(sun.GetPrim())
    sun_xform.AddRotateXYZOp().Set(Gf.Vec3f(-64.0, -27.0, 0.0))

    # Fill light (no shadows)
    fill = UsdLux.DistantLight.Define(stage, "/World/Lights/Fill")
    fill.CreateIntensityAttr(800.0)
    fill.CreateColorAttr(Gf.Vec3f(0.4, 0.4, 0.45))
    fill_xform = UsdGeom.Xformable(fill.GetPrim())
    fill_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

    # Ambient dome
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/AmbientDome")
    dome.CreateIntensityAttr(200.0)
    dome.CreateColorAttr(Gf.Vec3f(0.25, 0.25, 0.25))


def main():
    # Create new stage
    stage = Usd.Stage.CreateNew(OUTPUT_PATH)

    print(f"Building greenhouse USD world...")
    print(f"  Textures: {TEXTURES_DIR}")
    print(f"  Output:   {OUTPUT_PATH}")

    build_greenhouse(stage)

    stage.GetRootLayer().Save()
    print(f"Greenhouse USD saved to: {OUTPUT_PATH}")

    simulation_app.close()


if __name__ == "__main__":
    main()
