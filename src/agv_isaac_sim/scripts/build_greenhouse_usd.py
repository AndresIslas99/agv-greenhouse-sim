#!/usr/bin/env python3
"""Procedural USD world builder for the AGV greenhouse simulation.

Run inside Isaac Sim standalone Python:
    isaacsim --exec build_greenhouse_usd.py

Builds a photorealistic greenhouse USD stage:
- Ground plane (49x19m, OmniPBR soil with normal maps)
- 4 enclosure walls (3m, OmniGlass polycarbonate or opaque fallback)
- 12 crop rows: 6 front (X=17.5) + 6 rear (X=-6.5), OmniPBR leaf materials
- 20 heating pipe rails: 10 front + 10 rear (OmniPBR metallic steel)
- 7 roof beams (OmniPBR galvanized steel)
- 36 AprilTag markers (OmniPBR, zero specular, raw colorspace)
- 3 static crate obstacles (OmniPBR wood)
- HDRI dome + sun + fill + supplemental rect lights
- NVIDIA asset props (pallets, containers via USD references)
- PhysX scene (1ms timestep, GPU broadphase)

Environment variables:
    AGV_GLASS_WALLS=0  — disable glass walls for faster rendering
"""

import os
import sys
import math

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Vt, UsdLux

# ── Paths ─────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Ensure the world package (world/) is importable
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
# Assets now live inside agv_isaac_sim (no separate package)
ASSETS_DIR = os.path.join(PACKAGE_DIR, "assets")
TEXTURES_DIR = os.path.join(ASSETS_DIR, "greenhouse_textures")
TAG_TEXTURES_DIR = os.path.join(ASSETS_DIR, "tag_textures")
OUTPUT_PATH = os.path.join(PACKAGE_DIR, "worlds", "greenhouse_simple.usd")


def tag_texture_path(tag_id):
    """Resolve AprilTag texture path from agv_isaac_sim/assets/tag_textures."""
    return os.path.join(TAG_TEXTURES_DIR, f"tag36h11_id{tag_id}.png")


# ── Geometry Data (from greenhouse_simple.sdf) ────────────────────

# Ground & walls
GROUND_SIZE = (49.0, 19.0, 0.1)
GROUND_POS = (5.5, 0.0, -0.05)

WALL_SPECS = [
    ("North", (5.5, 7.5, 1.5), (45.0, 0.2, 3.0)),
    ("South", (5.5, -7.5, 1.5), (45.0, 0.2, 3.0)),
    ("West", (-17.0, 0.0, 1.5), (0.2, 15.0, 3.0)),
    ("East", (28.0, 0.0, 1.5), (0.2, 15.0, 3.0)),
]

# Crop rows: 6 Y positions, two sections.
# ROW_SIZE width (y) was 1.0 — too thick: gave only 0.10 m clearance per
# side for the 1.01 m wide robot navigating the central aisles. Set to
# 0.5 m so the gap between adjacent rows opens to 1.7 m. Keep this in
# sync with config/world_config.yaml crop_rows.width.
ROW_Y_POSITIONS = [-5.5, -3.3, -1.1, 1.1, 3.3, 5.5]
ROW_SIZE = (20.0, 0.5, 1.5)
ROW_Z = 0.75
FRONT_ROWS_X = 17.5
REAR_ROWS_X = -6.5

# Heating rails: 5 aisle centers, 2 rails per aisle
AISLE_CENTERS_Y = [-4.4, -2.2, 0.0, 2.2, 4.4]
RAIL_HALF_SPACING = 0.225
RAIL_RADIUS = 0.0255
RAIL_LENGTH = 20.0
FRONT_RAILS_X = 17.5
REAR_RAILS_X = -6.5

# Crates
CRATE_SPECS = [
    ("Crate1", (3.5, -2.0, 0.15), (0.5, 0.4, 0.3), 17.2),
    ("Crate2", (4.0, 3.5, 0.15), (0.5, 0.4, 0.3), -11.5),
    ("Crate3", (25.0, 0.0, 0.15), (0.5, 0.4, 0.3), 28.6),
]

# AprilTag placements: (id, x, y, z, rot_x_deg, rot_y_deg, rot_z_deg)
#
# _quad_mesh creates a quad in the YZ plane with face normal = +X.
# Rotations use RotateXYZ (degrees) applied to the parent Xform.
#
# Rotation cheat sheet (face normal starts as +X):
#   Vertical facing +X (no rotation):  (0, 0, 0)
#   Vertical facing -X:                (0, 0, 180)
#   Vertical facing +Y:                (0, 0, 90)
#   Vertical facing -Y:                (0, 0, -90)
#   Floor (face UP +Z):                (0, -90, 0)   — tilt +X toward +Z
#
APRILTAG_PLACEMENTS = [
    # Wall-mounted tags (vertical)
    (0, -16.88, 0.0, 0.145, 0, 0, 0),          # West wall center, facing +X
    (1, 27.88, 0.0, 0.145, 0, 0, 180),          # East wall center, facing -X
    (5, -16.88, -5.0, 0.145, 0, 0, 0),          # West wall south, facing +X
    (14, 15.0, 7.38, 0.145, 0, 0, -90),         # North wall, facing -Y
    (15, 15.0, -7.38, 0.145, 0, 0, 90),         # South wall, facing +Y
    # East wall per-aisle (vertical, facing -X)
    (16, 27.88, -4.4, 0.145, 0, 0, 180),
    (17, 27.88, -2.2, 0.145, 0, 0, 180),
    (18, 27.88, 0.0, 0.145, 0, 0, 180),
    (19, 27.88, 2.2, 0.145, 0, 0, 180),
    (20, 27.88, 4.4, 0.145, 0, 0, 180),
    # West wall per-aisle, rear section (vertical, facing +X)
    (21, -16.88, -4.4, 0.145, 0, 0, 0),
    (22, -16.88, -2.2, 0.145, 0, 0, 0),
    (23, -16.88, 0.0, 0.145, 0, 0, 0),
    (24, -16.88, 2.2, 0.145, 0, 0, 0),
    (25, -16.88, 4.4, 0.145, 0, 0, 0),
    # Front floor aisles (horizontal, face UP +Z) — 17 cm hacia el gap
    # del tip front (rail tip x=7.5). Pre-2026-04: 50 cm (x=7.0). Real
    # Opalina post-visit measurement: 17 cm from rail tip into the open
    # zone between the two rail sections (gap x∈[3.5, 7.5]). Brain
    # rail_approach setpoint depends on this geometry — keep in sync
    # with markers_registry.yaml on the brain side.
    (2,  7.33, -4.4, 0.002, 0, -90, 0),
    (3,  7.33, -2.2, 0.002, 0, -90, 0),
    (4,  7.33,  0.0, 0.002, 0, -90, 0),
    (12, 7.33,  2.2, 0.002, 0, -90, 0),
    (13, 7.33,  4.4, 0.002, 0, -90, 0),
    # Rear floor aisles (horizontal, face UP +Z) — 17 cm hacia el gap
    # del tip rear (rail tip x=3.5). Pre-2026-04: 50 cm (x=4.0).
    (33, 3.67, -4.4, 0.002, 0, -90, 0),
    (34, 3.67, -2.2, 0.002, 0, -90, 0),
    (35, 3.67,  0.0, 0.002, 0, -90, 0),
    (36, 3.67,  2.2, 0.002, 0, -90, 0),
    (37, 3.67,  4.4, 0.002, 0, -90, 0),
    # Front row starts (vertical, facing -X toward approaching robot)
    (6, 7.4, -5.5, 0.145, 0, 0, 180),
    (7, 7.4, -3.3, 0.145, 0, 0, 180),
    (8, 7.4, -1.1, 0.145, 0, 0, 180),
    (9, 7.4, 1.1, 0.145, 0, 0, 180),
    (10, 7.4, 3.3, 0.145, 0, 0, 180),
    (11, 7.4, 5.5, 0.145, 0, 0, 180),
    # Rear row starts (vertical, facing +X toward approaching robot)
    (26, 3.6, -5.5, 0.145, 0, 0, 0),
    (27, 3.6, -3.3, 0.145, 0, 0, 0),
    (28, 3.6, -1.1, 0.145, 0, 0, 0),
    (29, 3.6, 1.1, 0.145, 0, 0, 0),
    (30, 3.6, 3.3, 0.145, 0, 0, 0),
    (31, 3.6, 5.5, 0.145, 0, 0, 0),
]


# ── Helper Functions ──────────────────────────────────────────────

def create_physics_scene(stage):
    """Configure PhysX scene: 1ms timestep, GPU acceleration."""
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    physx_scene = stage.GetPrimAtPath("/World/PhysicsScene")
    physx_scene.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(True)
    physx_scene.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("GPU")
    physx_scene.CreateAttribute("physxScene:gpuMaxRigidContactCount", Sdf.ValueTypeNames.Int).Set(524288)
    physx_scene.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(1000)


def _resolve_normal_path(albedo_path):
    """Derive the normal map path from an albedo texture path.

    Convention: ground_soil.png -> ground_soil_normal.png
    Returns the path if the file exists, else None.
    """
    if not albedo_path:
        return None
    base, ext = os.path.splitext(albedo_path)
    normal_path = f"{base}_normal{ext}"
    return normal_path if os.path.exists(normal_path) else None


def make_material(stage, mat_path, texture_file, metallic=0.0, roughness=0.8,
                  diffuse_color=None, specular_level=0.5,
                  emissive_intensity=0.0, source_color_space="auto"):
    """Create an OmniPBR MDL material (primary) for RTX rendering.

    Automatically picks up a companion *_normal.png if present.
    Texture paths are stored relative to the USD file for portability.
    """
    from world.primitives import create_omnipbr_material

    normal_tex = _resolve_normal_path(texture_file)
    output_dir = os.path.dirname(OUTPUT_PATH)

    return create_omnipbr_material(
        stage, mat_path,
        albedo_texture=texture_file if (texture_file and os.path.exists(texture_file)) else None,
        normal_texture=normal_tex,
        roughness=roughness,
        metallic=metallic,
        diffuse_color=diffuse_color,
        specular_level=specular_level,
        emissive_intensity=emissive_intensity,
        source_color_space=source_color_space,
        output_path=output_dir,
    )


def make_glass_material(stage, mat_path, ior=1.59, glass_color=None,
                        frosting_roughness=0.3, thin_walled=True):
    """Create an OmniGlass MDL material for transparent surfaces."""
    from world.primitives import create_omniglass_material

    return create_omniglass_material(
        stage, mat_path,
        ior=ior,
        glass_color=glass_color,
        frosting_roughness=frosting_roughness,
        thin_walled=thin_walled,
    )


def _box_mesh_data(sx, sy, sz):
    """Generate vertices, face indices, normals, and UVs for an axis-aligned box."""
    # 8 corner vertices of a box centered at origin
    pts = [
        (-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),  # bottom
        (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz),      # top
    ]
    # 6 quad faces (vertex indices)
    faces = [
        (4, 5, 6, 7),  # +Z top
        (0, 3, 2, 1),  # -Z bottom
        (0, 1, 5, 4),  # -Y front
        (2, 3, 7, 6),  # +Y back
        (0, 4, 7, 3),  # -X left
        (1, 2, 6, 5),  # +X right
    ]
    normals = [
        (0, 0, 1), (0, 0, -1), (0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0),
    ]
    # UVs per face-vertex (faceVarying), tiling proportional to face size
    face_sizes = [
        (2*sx, 2*sy), (2*sx, 2*sy),  # top/bottom: X×Y
        (2*sx, 2*sz), (2*sx, 2*sz),  # front/back: X×Z
        (2*sy, 2*sz), (2*sy, 2*sz),  # left/right: Y×Z
    ]

    all_indices = []
    all_normals = []
    all_uvs = []
    face_counts = []
    for i, face in enumerate(faces):
        all_indices.extend(face)
        face_counts.append(4)
        n = normals[i]
        for _ in range(4):
            all_normals.append(n)
        w, h = face_sizes[i]
        all_uvs.extend([(0, 0), (w, 0), (w, h), (0, h)])

    return pts, all_indices, face_counts, all_normals, all_uvs


def create_box(stage, path, size, position, rotation_deg=(0, 0, 0), material=None, is_static=True):
    """Create a box mesh with UVs, collision, and material."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))

    sx, sy, sz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    pts, indices, face_counts, normals, uvs = _box_mesh_data(sx, sy, sz)

    mesh = UsdGeom.Mesh.Define(stage, f"{path}/Mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)

    # UV coordinates for texture mapping
    pv_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    uv_pv = pv_api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                 UsdGeom.Tokens.faceVarying)
    uv_pv.Set(Vt.Vec2fArray([Gf.Vec2f(*u) for u in uvs]))

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    if is_static:
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        xform.GetPrim().CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)

    if material:
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)

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


def _quad_mesh(stage, prim_path, width, height, material=None):
    """Create a single-sided quad mesh with proper UVs facing +X."""
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    hw, hh = width / 2.0, height / 2.0
    # Quad in YZ plane, facing +X
    mesh.CreatePointsAttr([
        Gf.Vec3f(0, -hw, -hh), Gf.Vec3f(0, hw, -hh),
        Gf.Vec3f(0, hw, hh), Gf.Vec3f(0, -hw, hh),
    ])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateNormalsAttr([Gf.Vec3f(1, 0, 0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)

    pv_api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    uv_pv = pv_api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                 UsdGeom.Tokens.faceVarying)
    uv_pv.Set(Vt.Vec2fArray([
        Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1),
    ]))

    if material:
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)
    return mesh


def create_apriltag(stage, path, position, rotation_deg, tag_material, border_material):
    """Create a two-layer AprilTag: white border (0.25×0.25) + textured tag (0.2×0.2).

    Uses flat quad meshes with UVs so textures render correctly.
    Tag face normal is along local +X axis before rotation.
    """
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))

    # White border quad: 0.25×0.25
    _quad_mesh(stage, f"{path}/Border", 0.25, 0.25, border_material)

    # Tag texture quad: 0.2×0.2, slightly in front of border
    tag_xform = UsdGeom.Xform.Define(stage, f"{path}/TagFace")
    tag_xform.AddTranslateOp().Set(Gf.Vec3d(0.001, 0.0, 0.0))
    _quad_mesh(stage, f"{path}/TagFace/Quad", 0.2, 0.2, tag_material)

    return xform


# ── Main Builder ──────────────────────────────────────────────────

def build_greenhouse(stage):
    """Build the complete greenhouse environment matching SDF parity."""

    # Root xform
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Physics scene
    create_physics_scene(stage)

    # ── Materials (OmniPBR MDL for RTX rendering) ──────────────
    M = "/World/Materials"

    mat_soil = make_material(
        stage, f"{M}/Soil",
        os.path.join(TEXTURES_DIR, "ground_soil.png"),
        roughness=0.95)

    # Walls: OmniGlass (polycarbonate, IoR 1.59, frosted) or opaque fallback
    ENABLE_GLASS_WALLS = os.environ.get("AGV_GLASS_WALLS", "1") == "1"
    if ENABLE_GLASS_WALLS:
        mat_wall = make_glass_material(
            stage, f"{M}/Polycarbonate",
            ior=1.59,
            glass_color=Gf.Vec3f(0.92, 0.95, 0.90),
            frosting_roughness=0.35,
            thin_walled=True)
        print("  Walls: OmniGlass (polycarbonate, frosted)")
    else:
        mat_wall = make_material(
            stage, f"{M}/Polycarbonate",
            os.path.join(TEXTURES_DIR, "wall_polycarbonate.png"),
            roughness=0.65)
        print("  Walls: OmniPBR (opaque fallback)")

    mat_crop_a = make_material(
        stage, f"{M}/CropLeaves",
        os.path.join(TEXTURES_DIR, "crop_leaves.png"),
        roughness=0.9, specular_level=0.2)

    mat_crop_b = make_material(
        stage, f"{M}/CropLeavesAlt",
        os.path.join(TEXTURES_DIR, "crop_leaves_alt.png"),
        roughness=0.9, specular_level=0.2)

    mat_rail = make_material(
        stage, f"{M}/Steel",
        os.path.join(TEXTURES_DIR, "rail_steel.png"),
        metallic=0.85, roughness=0.35)

    mat_crate = make_material(
        stage, f"{M}/CrateWood",
        os.path.join(TEXTURES_DIR, "crate_wood.png"),
        roughness=0.85)

    mat_beam = make_material(
        stage, f"{M}/SteelBeam", "",
        metallic=0.7, roughness=0.4,
        diffuse_color=Gf.Vec3f(0.6, 0.6, 0.62))

    # AprilTag border material (white, perfectly matte, zero specular)
    mat_tag_border = make_material(
        stage, f"{M}/TagBorder", "",
        roughness=1.0, metallic=0.0, specular_level=0.0,
        diffuse_color=Gf.Vec3f(0.95, 0.95, 0.95))

    # AprilTag texture materials (one per tag ID, matte + raw colorspace)
    tag_ids_needed = sorted(set(t[0] for t in APRILTAG_PLACEMENTS))
    tag_materials = {}
    print(f"\n  AprilTag textures ({len(tag_ids_needed)} tags):")
    for tag_id in tag_ids_needed:
        tex_path = tag_texture_path(tag_id)
        exists = os.path.exists(tex_path)
        print(f"    Tag {tag_id:2d}: {'OK' if exists else 'MISSING'} -> {tex_path}")
        tag_materials[tag_id] = make_material(
            stage, f"{M}/AprilTag{tag_id}",
            tex_path, roughness=1.0, metallic=0.0,
            specular_level=0.0,
            source_color_space="raw")  # raw prevents sRGB gamma on B/W data

    # ── Ground Plane ──────────────────────────────────────────
    create_box(stage, "/World/Ground",
               size=GROUND_SIZE,
               position=GROUND_POS,
               material=mat_soil)

    # Physics material for the ground (high friction so wheels don't slip).
    # Bound with purpose='physics' so it doesn't conflict with the visual
    # OmniPBR binding above.
    ground_phys_mat = UsdShade.Material.Define(stage, f"{M}/GroundPhys").GetPrim()
    gp_api = UsdPhysics.MaterialAPI.Apply(ground_phys_mat)
    gp_api.CreateStaticFrictionAttr(1.5)
    gp_api.CreateDynamicFrictionAttr(1.3)
    gp_api.CreateRestitutionAttr(0.0)
    try:
        from pxr import PhysxSchema
        px = PhysxSchema.PhysxMaterialAPI.Apply(ground_phys_mat)
        px.CreateFrictionCombineModeAttr('max')
    except Exception:
        pass
    ground_mesh = stage.GetPrimAtPath("/World/Ground/Mesh")
    if ground_mesh and ground_mesh.IsValid():
        binding = UsdShade.MaterialBindingAPI.Apply(ground_mesh)
        binding.Bind(UsdShade.Material(ground_phys_mat),
                     UsdShade.Tokens.weakerThanDescendants, 'physics')
        print("  Ground: physics material bound (μs=1.5 / μd=1.3)")

    # ── Walls ─────────────────────────────────────────────────
    UsdGeom.Xform.Define(stage, "/World/Walls")
    for name, pos, size in WALL_SPECS:
        create_box(stage, f"/World/Walls/{name}",
                   size=size, position=pos, material=mat_wall)

    # ── Crop Rows (front + rear) ──────────────────────────────
    UsdGeom.Xform.Define(stage, "/World/CropRows")

    for i, y in enumerate(ROW_Y_POSITIONS):
        mat = mat_crop_a if i % 2 == 0 else mat_crop_b
        # Front section
        create_box(stage, f"/World/CropRows/Row{i + 1}",
                   size=ROW_SIZE,
                   position=(FRONT_ROWS_X, y, ROW_Z),
                   material=mat)
        # Rear section
        create_box(stage, f"/World/CropRows/RearRow{i + 1}",
                   size=ROW_SIZE,
                   position=(REAR_ROWS_X, y, ROW_Z),
                   material=mat)

    # ── Heating Pipe Rails (front + rear) ─────────────────────
    UsdGeom.Xform.Define(stage, "/World/Rails")

    for i, y_center in enumerate(AISLE_CENTERS_Y):
        for side, offset in [("Left", -RAIL_HALF_SPACING), ("Right", RAIL_HALF_SPACING)]:
            # Front section
            create_cylinder(
                stage, f"/World/Rails/Aisle{i + 1}_{side}",
                radius=RAIL_RADIUS, height=RAIL_LENGTH,
                position=(FRONT_RAILS_X, y_center + offset, RAIL_RADIUS),
                rotation_deg=(0.0, 90.0, 0.0),
                material=mat_rail)
            # Rear section
            create_cylinder(
                stage, f"/World/Rails/RearAisle{i + 1}_{side}",
                radius=RAIL_RADIUS, height=RAIL_LENGTH,
                position=(REAR_RAILS_X, y_center + offset, RAIL_RADIUS),
                rotation_deg=(0.0, 90.0, 0.0),
                material=mat_rail)

    # ── Roof Beams ────────────────────────────────────────────
    UsdGeom.Xform.Define(stage, "/World/RoofBeams")

    for i, x in enumerate([5.0, 10.0, 15.0, 20.0, 25.0]):
        create_box(stage, f"/World/RoofBeams/Transverse{i + 1}",
                   size=(0.08, 15.0, 0.08),
                   position=(x, 0.0, 3.0),
                   material=mat_beam)

    for i, y in enumerate([-3.5, 3.5]):
        create_box(stage, f"/World/RoofBeams/Longitudinal{i + 1}",
                   size=(30.0, 0.08, 0.08),
                   position=(15.0, y, 3.0),
                   material=mat_beam)

    # ── AprilTag Markers (all 36 from SDF) ────────────────────
    UsdGeom.Xform.Define(stage, "/World/AprilTags")

    for tag_id, x, y, z, rx, ry, rz in APRILTAG_PLACEMENTS:
        create_apriltag(
            stage, f"/World/AprilTags/Tag{tag_id}",
            position=(x, y, z),
            rotation_deg=(rx, ry, rz),
            tag_material=tag_materials[tag_id],
            border_material=mat_tag_border)

    # ── Static Crate Obstacles ────────────────────────────────
    UsdGeom.Xform.Define(stage, "/World/Crates")

    for name, pos, size, yaw_deg in CRATE_SPECS:
        create_box(stage, f"/World/Crates/{name}",
                   size=size, position=pos,
                   rotation_deg=(0.0, 0.0, yaw_deg),
                   material=mat_crate)

    # ── Lighting (HDRI dome + sun + fill + supplemental rect lights) ──
    from world.greenhouse_lighting import create_lighting
    import yaml

    config_path = os.path.join(PACKAGE_DIR, "config", "world_config.yaml")
    with open(config_path) as f:
        world_cfg = yaml.safe_load(f)

    # Resolve HDRI path if configured (relative to textures/ dir)
    hdri_rel = world_cfg["lighting"]["ambient_dome"].get("hdri", "")
    if hdri_rel and not os.path.isabs(hdri_rel):
        hdri_abs = os.path.join(PACKAGE_DIR, "textures", hdri_rel)
        if os.path.exists(hdri_abs):
            world_cfg["lighting"]["ambient_dome"]["hdri"] = hdri_abs

    create_lighting(stage, world_cfg)

    # ── NVIDIA Asset Props (pallets, containers, etc.) ──────────
    from world.nvidia_assets import add_props
    add_props(stage, world_cfg)


def main():
    stage = Usd.Stage.CreateNew(OUTPUT_PATH)

    print("Building greenhouse USD world (full SDF parity)...")
    print(f"  Textures: {TEXTURES_DIR}")
    print(f"  Output:   {OUTPUT_PATH}")

    build_greenhouse(stage)

    stage.GetRootLayer().Save()

    # Count elements
    n_tags = len(APRILTAG_PLACEMENTS)
    n_rows = len(ROW_Y_POSITIONS) * 2
    n_rails = len(AISLE_CENTERS_Y) * 2 * 2
    print(f"  Ground:    49×19m")
    print(f"  Walls:     4")
    print(f"  Crop rows: {n_rows} (front + rear)")
    print(f"  Rails:     {n_rails} (front + rear)")
    print(f"  AprilTags: {n_tags}")
    print(f"  Crates:    {len(CRATE_SPECS)}")
    print(f"Greenhouse USD saved to: {OUTPUT_PATH}")

    simulation_app.close()


if __name__ == "__main__":
    main()
