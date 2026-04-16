"""Greenhouse lighting: sun, fill, ambient dome (HDRI), rect lights, and fog.

Greenhouse interior lighting is predominantly diffuse — sunlight passes
through translucent polycarbonate roof panels, scattering into soft
omnidirectional illumination with almost no hard shadows.

Lighting hierarchy:
1. Ambient dome (HDRI or solid color) — primary, simulates roof-diffused sky
2. Sun distant light — reduced intensity, wider angle for soft shadows
3. Fill distant light — cool tone for contrast and depth
4. Interior rect lights — simulate overhead supplemental / grow lights
5. Volumetric fog (optional) — humidity simulation for depth camera degradation
"""

import os

from pxr import UsdGeom, UsdLux, Gf, Sdf


def create_lighting(stage, cfg):
    """Create the full lighting setup for greenhouse interior."""
    UsdGeom.Xform.Define(stage, "/World/Lights")
    lc = cfg["lighting"]

    _create_sun(stage, lc["sun"])
    _create_fill(stage, lc["fill"])
    _create_ambient_dome(stage, lc["ambient_dome"])
    _create_supplemental_lights(stage, lc.get("supplemental_lights", {}))

    fog_cfg = lc.get("fog", {})
    if fog_cfg.get("enabled", False):
        _create_humidity_fog(stage, fog_cfg)


def _create_sun(stage, sun_cfg):
    """Sun distant light — reduced intensity for greenhouse diffusion."""
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(sun_cfg["intensity"])
    sun.CreateColorAttr(Gf.Vec3f(*sun_cfg["color"]))
    sun.CreateAngleAttr(sun_cfg["angle"])
    xf = UsdGeom.Xformable(sun.GetPrim())
    # Position above the greenhouse roof so the gizmo isn't on the ground
    xf.AddTranslateOp().Set(Gf.Vec3d(15.0, 0.0, 10.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*sun_cfg["rotation"]))


def _create_fill(stage, fill_cfg):
    """Fill distant light — cool tone for contrast."""
    fill = UsdLux.DistantLight.Define(stage, "/World/Lights/Fill")
    fill.CreateIntensityAttr(fill_cfg["intensity"])
    fill.CreateColorAttr(Gf.Vec3f(*fill_cfg["color"]))
    xf = UsdGeom.Xformable(fill.GetPrim())
    # Position above the greenhouse roof alongside the sun
    xf.AddTranslateOp().Set(Gf.Vec3d(15.0, 5.0, 10.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*fill_cfg["rotation"]))


def _create_ambient_dome(stage, dome_cfg):
    """Ambient dome light — HDRI environment map or solid color fallback.

    When an HDRI is provided, the dome light uses it for realistic sky
    gradients and ambient reflections. Without HDRI, falls back to the
    calibrated solid color.
    """
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/AmbientDome")
    dome.CreateIntensityAttr(dome_cfg["intensity"])
    # Position gizmo above the greenhouse roof (visual only, dome light is omnidirectional)
    UsdGeom.Xformable(dome.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(15.0, 0.0, 12.0))

    hdri_path = dome_cfg.get("hdri", "")
    if hdri_path and os.path.exists(hdri_path):
        dome.CreateTextureFileAttr(hdri_path)
        dome.CreateTextureFormatAttr("latlong")
        # HDRI rotation for time-of-day variation
        rotation = dome_cfg.get("hdri_rotation", 0.0)
        if rotation != 0.0:
            UsdGeom.Xformable(dome.GetPrim()).AddRotateXYZOp().Set(
                Gf.Vec3f(0.0, rotation, 0.0))
        print(f"  Dome light: HDRI ({os.path.basename(hdri_path)})")
    else:
        dome.CreateColorAttr(Gf.Vec3f(*dome_cfg["color"]))
        if hdri_path:
            print(f"  Dome light: HDRI not found ({hdri_path}), using solid color")
        else:
            print("  Dome light: solid color (no HDRI configured)")


def _create_supplemental_lights(stage, sup_cfg):
    """Interior rect lights simulating overhead supplemental / grow lights.

    Real commercial greenhouses often have HPS or LED grow lights mounted
    on the roof structure. These provide fill lighting and reduce shadow
    zones that could affect AprilTag detection.
    """
    if not sup_cfg.get("enabled", False):
        return

    UsdGeom.Xform.Define(stage, "/World/Lights/Supplemental")

    positions_x = sup_cfg.get("positions_x", [5.0, 15.0, 25.0])
    intensity = sup_cfg.get("intensity", 200.0)
    color = sup_cfg.get("color", [0.95, 0.97, 0.90])
    width = sup_cfg.get("width", 4.0)
    height = sup_cfg.get("height", 2.0)
    mount_z = sup_cfg.get("mount_z", 2.85)

    for i, x in enumerate(positions_x):
        light = UsdLux.RectLight.Define(
            stage, f"/World/Lights/Supplemental/RoofLight{i}")
        light.CreateIntensityAttr(intensity)
        light.CreateWidthAttr(width)
        light.CreateHeightAttr(height)
        light.CreateColorAttr(Gf.Vec3f(*color))

        xformable = UsdGeom.Xformable(light.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, mount_z))
        # Face downward (rotate 90 degrees around X to point -Z)
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0))

    print(f"  Supplemental lights: {len(positions_x)} rect lights at Z={mount_z}m")


def _create_humidity_fog(stage, fog_cfg):
    """Create volumetric fog to simulate greenhouse humidity.

    Real greenhouses: 40-95% RH, with periodic misting that creates
    visible haze. This degrades ZED 2i depth camera range by 30-60%.
    """
    fog_path = "/World/Environment/HumidityFog"
    UsdGeom.Xform.Define(stage, "/World/Environment")
    fog_vol = UsdGeom.Cube.Define(stage, fog_path)

    height_max = fog_cfg.get("height_max", 2.0)
    fog_vol.AddTranslateOp().Set(Gf.Vec3d(15.0, 0.0, height_max / 2.0))
    fog_vol.AddScaleOp().Set(Gf.Vec3f(17.0, 9.0, height_max / 2.0))

    prim = fog_vol.GetPrim()
    prim.CreateAttribute("purpose", Sdf.ValueTypeNames.Token).Set("render")

    density = fog_cfg.get("density", 0.02)
    color = fog_cfg.get("color", [0.95, 0.95, 1.0])

    prim.CreateAttribute("omni:volume:absorption", Sdf.ValueTypeNames.Float).Set(density)
    prim.CreateAttribute("omni:volume:scattering", Sdf.ValueTypeNames.Float).Set(density * 0.5)
    prim.CreateAttribute("omni:volume:color", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))

    print(f"  Humidity fog: density={density}, height={height_max}m")
