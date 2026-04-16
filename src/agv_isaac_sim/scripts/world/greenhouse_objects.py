"""Greenhouse objects: AprilTag markers (wall-mounted) and crate obstacles."""

from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

from .primitives import create_box, create_pbr_material, create_omnipbr_material
from .config import tag_texture_path


def _create_apriltag_marker(stage, path, tag_id, position, yaw_deg,
                            tag_material, white_material, cfg):
    """Create a wall-mounted AprilTag with white backing plate + textured face.

    Structure:
        Xform "TagN"
          ├── BackingPlate  (0.25×0.25×0.005m, white, collision)
          └── TagFace       (0.2×0.2×0.001m, textured, visual only)
    """
    tc = cfg["apriltags"]
    border_sz = tc["border_size"]
    tag_sz = tc["tag_size"]
    border_th = tc["border_thickness"]
    tag_th = tc["tag_thickness"]

    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if yaw_deg != 0:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw_deg))

    # Backing plate (white, with collision so robot can't drive through)
    plate = UsdGeom.Cube.Define(stage, f"{path}/BackingPlate")
    plate.AddScaleOp().Set(Gf.Vec3f(border_th / 2.0, border_sz / 2.0, border_sz / 2.0))
    UsdPhysics.CollisionAPI.Apply(plate.GetPrim())
    UsdShade.MaterialBindingAPI(plate.GetPrim()).Bind(white_material)

    # Tag face (textured, slightly in front of backing)
    face = UsdGeom.Cube.Define(stage, f"{path}/TagFace")
    face.AddTranslateOp().Set(Gf.Vec3d(border_th / 2.0 + tag_th / 2.0, 0.0, 0.0))
    face.AddScaleOp().Set(Gf.Vec3f(tag_th / 2.0, tag_sz / 2.0, tag_sz / 2.0))
    UsdShade.MaterialBindingAPI(face.GetPrim()).Bind(tag_material)

    return xform


def create_apriltags(stage, materials, cfg):
    """Create all AprilTag markers, wall-mounted with backing plates."""
    UsdGeom.Xform.Define(stage, "/World/AprilTags")

    # White material for backing plates (OmniPBR, zero specular for detection)
    white_mat = create_omnipbr_material(
        stage, "/World/Materials/TagBacking",
        roughness=1.0, metallic=0.0, specular_level=0.0,
        diffuse_color=Gf.Vec3f(0.95, 0.95, 0.95))

    # Tag materials (per ID, OmniPBR with raw colorspace for sharp B/W)
    tag_materials = {}
    for tag_cfg in cfg["apriltags"]["tags"]:
        tid = tag_cfg["id"]
        if tid not in tag_materials:
            tex = tag_texture_path(cfg, tid)
            tag_materials[tid] = create_omnipbr_material(
                stage, f"/World/Materials/AprilTag{tid}",
                albedo_texture=tex, roughness=1.0, metallic=0.0,
                specular_level=0.0, source_color_space="raw")

    # Create each tag
    for tag_cfg in cfg["apriltags"]["tags"]:
        tid = tag_cfg["id"]
        _create_apriltag_marker(
            stage, f"/World/AprilTags/Tag{tid}",
            tag_id=tid,
            position=tag_cfg["position"],
            yaw_deg=tag_cfg["yaw"],
            tag_material=tag_materials[tid],
            white_material=white_mat,
            cfg=cfg)


def create_obstacles(stage, materials, cfg):
    """Create static crate obstacles."""
    UsdGeom.Xform.Define(stage, "/World/Crates")

    for crate in cfg["obstacles"]["crates"]:
        create_box(stage, f"/World/Crates/{crate['name']}",
                   size=crate["size"], position=crate["position"],
                   rotation_deg=(0.0, 0.0, crate["yaw"]),
                   material=materials["crate"])
