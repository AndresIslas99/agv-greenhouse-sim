"""Greenhouse lighting: sun, fill, ambient dome, and optional humidity fog.

Greenhouse interior lighting is predominantly diffuse — sunlight passes
through translucent polycarbonate roof panels, scattering into soft
omnidirectional illumination with almost no hard shadows.
"""

from pxr import UsdGeom, UsdLux, Gf


def create_lighting(stage, cfg):
    """Create the lighting setup for greenhouse interior."""
    UsdGeom.Xform.Define(stage, "/World/Lights")
    lc = cfg["lighting"]

    # Sun (reduced intensity for greenhouse — light diffused by roof)
    sun_cfg = lc["sun"]
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(sun_cfg["intensity"])
    sun.CreateColorAttr(Gf.Vec3f(*sun_cfg["color"]))
    sun.CreateAngleAttr(sun_cfg["angle"])
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(*sun_cfg["rotation"]))

    # Fill light (cool tone for contrast)
    fill_cfg = lc["fill"]
    fill = UsdLux.DistantLight.Define(stage, "/World/Lights/Fill")
    fill.CreateIntensityAttr(fill_cfg["intensity"])
    fill.CreateColorAttr(Gf.Vec3f(*fill_cfg["color"]))
    UsdGeom.Xformable(fill.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(*fill_cfg["rotation"]))

    # Ambient dome (primary light source — simulates roof-diffused daylight)
    dome_cfg = lc["ambient_dome"]
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/AmbientDome")
    dome.CreateIntensityAttr(dome_cfg["intensity"])
    dome.CreateColorAttr(Gf.Vec3f(*dome_cfg["color"]))

    # Volumetric fog for humidity simulation (Tier 2)
    # When enabled, degrades ZED depth camera range realistically.
    # Isaac Sim RTX supports volumetric fog via OmniPBR volume shaders.
    fog_cfg = lc.get("fog", {})
    if fog_cfg.get("enabled", False):
        _create_humidity_fog(stage, fog_cfg)


def _create_humidity_fog(stage, fog_cfg):
    """Create volumetric fog to simulate greenhouse humidity.

    Real greenhouses: 40-95% RH, with periodic misting that creates
    visible haze. This degrades ZED 2i depth camera range by 30-60%.

    Uses a large volume prim with absorption/scattering properties.
    Density is configurable: 0.01 = light haze, 0.1 = heavy misting.
    """
    from pxr import Sdf

    fog_path = "/World/Environment/HumidityFog"
    UsdGeom.Xform.Define(stage, "/World/Environment")
    fog_vol = UsdGeom.Cube.Define(stage, fog_path)

    # Fog volume covers the lower part of the greenhouse
    height_max = fog_cfg.get("height_max", 2.0)
    fog_vol.AddTranslateOp().Set(Gf.Vec3d(15.0, 0.0, height_max / 2.0))
    fog_vol.AddScaleOp().Set(Gf.Vec3f(17.0, 9.0, height_max / 2.0))

    # Set as non-renderable geometry — fog effect via volume material
    prim = fog_vol.GetPrim()
    prim.CreateAttribute("purpose", Sdf.ValueTypeNames.Token).Set("render")

    density = fog_cfg.get("density", 0.02)
    color = fog_cfg.get("color", [0.95, 0.95, 1.0])

    # Volume rendering attributes for Isaac Sim RTX
    prim.CreateAttribute("omni:volume:absorption", Sdf.ValueTypeNames.Float).Set(density)
    prim.CreateAttribute("omni:volume:scattering", Sdf.ValueTypeNames.Float).Set(density * 0.5)
    prim.CreateAttribute("omni:volume:color", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))

    print(f"  Humidity fog: density={density}, height={height_max}m")
