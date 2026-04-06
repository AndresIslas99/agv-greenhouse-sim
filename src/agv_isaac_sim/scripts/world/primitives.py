"""Reusable USD primitive creation functions."""

import os
from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf


def create_pbr_material(stage, mat_path, texture_file, metallic=0.0, roughness=0.8):
    """Create a PBR material with an optional albedo texture."""
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
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.5, 0.5, 0.5))

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _apply_transform(xform, position, rotation_deg):
    """Apply translation and optional rotation to an Xform."""
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))


def create_box(stage, path, size, position, rotation_deg=(0, 0, 0),
               material=None, is_static=True):
    """Create a box prim with collision and optional material."""
    xform = UsdGeom.Xform.Define(stage, path)
    _apply_transform(xform, position, rotation_deg)

    cube = UsdGeom.Cube.Define(stage, f"{path}/Mesh")
    cube.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if is_static:
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        xform.GetPrim().CreateAttribute(
            "physics:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(True)

    if material:
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

    return xform


def create_cylinder(stage, path, radius, height, position,
                    rotation_deg=(0, 0, 0), material=None):
    """Create a cylinder prim with collision."""
    xform = UsdGeom.Xform.Define(stage, path)
    _apply_transform(xform, position, rotation_deg)

    cyl = UsdGeom.Cylinder.Define(stage, f"{path}/Mesh")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")

    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    if material:
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(material)

    return xform
