"""Greenhouse structural elements: ground, walls, crop rows, rails, roof beams."""

from pxr import UsdGeom

from .primitives import create_box, create_cylinder


def create_ground(stage, materials, cfg):
    """Create the soil ground plane."""
    g = cfg["ground"]
    create_box(stage, "/World/Ground",
               size=g["size"], position=g["position"],
               material=materials["soil"])


def create_walls(stage, materials, cfg):
    """Create the 4 enclosure walls."""
    UsdGeom.Xform.Define(stage, "/World/Walls")
    for spec in cfg["walls"]["specs"]:
        create_box(stage, f"/World/Walls/{spec['name']}",
                   size=spec["size"], position=spec["position"],
                   material=materials["wall"])


def create_crop_rows(stage, materials, cfg):
    """Create the 6 parallel crop rows."""
    UsdGeom.Xform.Define(stage, "/World/CropRows")
    r = cfg["crop_rows"]
    mat_keys = ["crop_a", "crop_b"]

    for i, y in enumerate(r["y_positions"]):
        create_box(stage, f"/World/CropRows/Row{i + 1}",
                   size=(r["length"], r["width"], r["height"]),
                   position=(r["center_x"], y, r["height"] / 2.0),
                   material=materials[mat_keys[i % 2]])


def create_rails(stage, materials, cfg):
    """Create the heating pipe rails (2 per aisle, 5 aisles = 10 total)."""
    UsdGeom.Xform.Define(stage, "/World/Rails")
    r = cfg["rails"]
    half_spacing = r["pair_spacing"] / 2.0

    for i, y_center in enumerate(r["aisle_centers_y"]):
        for side, offset in [("Left", -half_spacing), ("Right", half_spacing)]:
            create_cylinder(
                stage, f"/World/Rails/Aisle{i + 1}_{side}",
                radius=r["radius"], height=r["length"],
                position=(r["center_x"], y_center + offset, r["radius"]),
                rotation_deg=(0.0, 90.0, 0.0),
                material=materials["rail"])


def create_roof_beams(stage, materials, cfg):
    """Create the roof beam grid (transverse + longitudinal)."""
    UsdGeom.Xform.Define(stage, "/World/RoofBeams")
    b = cfg["roof_beams"]
    s = b["beam_size"]
    h = b["height"]

    for i, x in enumerate(b["transverse_x"]):
        create_box(stage, f"/World/RoofBeams/Transverse{i + 1}",
                   size=(s, b["transverse_span"], s),
                   position=(x, 0.0, h),
                   material=materials["beam"])

    for i, y in enumerate(b["longitudinal_y"]):
        create_box(stage, f"/World/RoofBeams/Longitudinal{i + 1}",
                   size=(b["longitudinal_span"], s, s),
                   position=(b["longitudinal_center_x"], y, h),
                   material=materials["beam"])
