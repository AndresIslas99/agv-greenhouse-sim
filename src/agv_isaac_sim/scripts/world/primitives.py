"""Reusable USD primitive creation functions and material creators.

Material hierarchy:
- create_omnipbr_material()  — OmniPBR MDL: normal maps, displacement, emissive, specular control
- create_omniglass_material() — OmniGlass MDL: transparent/frosted glass (polycarbonate walls)
- create_pbr_material()       — UsdPreviewSurface fallback (headless / non-RTX)
"""

import os
from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf


# ---------------------------------------------------------------------------
# OmniPBR MDL material (Isaac Sim RTX)
# ---------------------------------------------------------------------------

def create_omnipbr_material(stage, mat_path, albedo_texture=None, normal_texture=None,
                            roughness=0.5, metallic=0.0, diffuse_color=None,
                            emissive_intensity=0.0, emissive_color=None,
                            specular_level=0.5, source_color_space="auto",
                            enable_opacity=False, opacity=1.0,
                            output_path=None):
    """Create an OmniPBR MDL material with full RTX feature set.

    Args:
        stage: USD stage
        mat_path: prim path for the material (e.g. "/World/Materials/Soil")
        albedo_texture: path to albedo/diffuse texture file (None = solid color)
        normal_texture: path to tangent-space normal map (None = flat)
        roughness: reflection roughness (0=mirror, 1=matte)
        metallic: metallic factor (0=dielectric, 1=metal)
        diffuse_color: Gf.Vec3f solid color when no texture (default mid-grey)
        emissive_intensity: emission strength (0=off)
        emissive_color: Gf.Vec3f emission color (default white)
        specular_level: specular reflection intensity (0=disabled)
        source_color_space: "auto", "raw", or "sRGB" for albedo texture
        enable_opacity: enable transparency
        opacity: opacity value (0=transparent, 1=opaque)
        output_path: directory for relative texture paths (None = absolute)
    """
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")

    # MDL source asset
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset("OmniPBR.mdl", "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")

    # Core PBR properties
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(specular_level)

    # Albedo: texture or solid color
    if albedo_texture and os.path.exists(albedo_texture):
        if output_path:
            tex_ref = os.path.relpath(albedo_texture, output_path)
        else:
            tex_ref = albedo_texture
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(tex_ref)
        if source_color_space != "auto":
            shader.CreateInput("diffuse_color_space",
                               Sdf.ValueTypeNames.Token).Set(source_color_space)
    else:
        color = diffuse_color if diffuse_color is not None else Gf.Vec3f(0.5, 0.5, 0.5)
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(color)

    # Normal map
    if normal_texture and os.path.exists(normal_texture):
        if output_path:
            norm_ref = os.path.relpath(normal_texture, output_path)
        else:
            norm_ref = normal_texture
        shader.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(norm_ref)

    # Emissive
    if emissive_intensity > 0:
        shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(emissive_intensity)
        e_color = emissive_color if emissive_color is not None else Gf.Vec3f(1.0, 1.0, 1.0)
        shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(e_color)

    # Opacity
    if enable_opacity:
        shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(opacity)

    # Wire MDL outputs
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateOutput("mdl:surface", Sdf.ValueTypeNames.Token).ConnectToSource(
        shader.ConnectableAPI(), "out")
    material.CreateOutput("mdl:displacement", Sdf.ValueTypeNames.Token).ConnectToSource(
        shader.ConnectableAPI(), "out")
    material.CreateOutput("mdl:volume", Sdf.ValueTypeNames.Token).ConnectToSource(
        shader.ConnectableAPI(), "out")

    # Also wire UsdPreviewSurface output for viewport fallback
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

    return material


# ---------------------------------------------------------------------------
# OmniGlass MDL material (transparent / frosted surfaces)
# ---------------------------------------------------------------------------

def create_omniglass_material(stage, mat_path, ior=1.5, glass_color=None,
                              frosting_roughness=0.0, thin_walled=False,
                              depth=0.001):
    """Create an OmniGlass MDL material for transparent surfaces.

    Args:
        stage: USD stage
        mat_path: prim path for the material
        ior: index of refraction (1.59 for polycarbonate, 1.52 for glass)
        glass_color: Gf.Vec3f tint color (default near-white)
        frosting_roughness: 0=clear, 0.3=frosted, 1.0=fully diffuse
        thin_walled: True for thin panels (no refraction offset)
        depth: volume absorption depth in meters
    """
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")

    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset("OmniGlass.mdl", "mdl")
    shader.SetSourceAssetSubIdentifier("OmniGlass", "mdl")

    shader.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(ior)
    color = glass_color if glass_color is not None else Gf.Vec3f(0.95, 0.97, 0.93)
    shader.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(frosting_roughness)
    shader.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(thin_walled)
    shader.CreateInput("depth", Sdf.ValueTypeNames.Float).Set(depth)

    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateOutput("mdl:surface", Sdf.ValueTypeNames.Token).ConnectToSource(
        shader.ConnectableAPI(), "out")
    material.CreateOutput("mdl:displacement", Sdf.ValueTypeNames.Token).ConnectToSource(
        shader.ConnectableAPI(), "out")
    material.CreateOutput("mdl:volume", Sdf.ValueTypeNames.Token).ConnectToSource(
        shader.ConnectableAPI(), "out")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "out")

    return material


# ---------------------------------------------------------------------------
# UsdPreviewSurface fallback (non-RTX / headless)
# ---------------------------------------------------------------------------

def create_pbr_material(stage, mat_path, texture_file, metallic=0.0, roughness=0.8,
                        diffuse_color=None):
    """Create a UsdPreviewSurface material (fallback for non-RTX rendering)."""
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
    elif diffuse_color is not None:
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(diffuse_color)
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
