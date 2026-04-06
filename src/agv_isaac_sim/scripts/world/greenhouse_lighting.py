"""Greenhouse lighting: sun, fill, ambient dome."""

from pxr import UsdGeom, UsdLux, Gf


def create_lighting(stage, cfg):
    """Create the three-light setup for greenhouse interior."""
    UsdGeom.Xform.Define(stage, "/World/Lights")
    lc = cfg["lighting"]

    # Sun (main directional, casts shadows)
    sun_cfg = lc["sun"]
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
    sun.CreateIntensityAttr(sun_cfg["intensity"])
    sun.CreateColorAttr(Gf.Vec3f(*sun_cfg["color"]))
    sun.CreateAngleAttr(sun_cfg["angle"])
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(*sun_cfg["rotation"]))

    # Fill light (softer, no shadows)
    fill_cfg = lc["fill"]
    fill = UsdLux.DistantLight.Define(stage, "/World/Lights/Fill")
    fill.CreateIntensityAttr(fill_cfg["intensity"])
    fill.CreateColorAttr(Gf.Vec3f(*fill_cfg["color"]))
    UsdGeom.Xformable(fill.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(*fill_cfg["rotation"]))

    # Ambient dome
    dome_cfg = lc["ambient_dome"]
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/AmbientDome")
    dome.CreateIntensityAttr(dome_cfg["intensity"])
    dome.CreateColorAttr(Gf.Vec3f(*dome_cfg["color"]))
