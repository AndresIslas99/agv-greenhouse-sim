"""Greenhouse structural elements: ground, walls, crop rows, rails, roof beams."""

import random

from pxr import UsdGeom, Gf

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
    """Create crop rows as segmented foliage with gaps.

    Instead of monolithic boxes, each row is built from plant segments
    with gaps between them. This gives realistic depth camera returns
    (individual plant silhouettes vs flat walls) and allows partial
    see-through at certain angles — matching real greenhouse crops.

    Segments have random height/width variation to break visual uniformity.
    """
    UsdGeom.Xform.Define(stage, "/World/CropRows")
    r = cfg["crop_rows"]
    f = r.get("foliage", {})
    mat_keys = ["crop_a", "crop_b"]

    seg_len = f.get("segment_length", 0.8)
    gap_len = f.get("gap_length", 0.15)
    h_var = f.get("height_variation", 0.15)
    w_var = f.get("width_variation", 0.05)

    stride = seg_len + gap_len
    row_start_x = r["center_x"] - r["length"] / 2.0

    # Deterministic seed for reproducible greenhouse layout
    rng = random.Random(42)

    for row_i, y in enumerate(r["y_positions"]):
        row_path = f"/World/CropRows/Row{row_i + 1}"
        UsdGeom.Xform.Define(stage, row_path)
        mat = materials[mat_keys[row_i % 2]]

        num_segments = int(r["length"] / stride)
        for seg_i in range(num_segments):
            # Per-segment random variation
            dh = rng.uniform(-h_var, h_var)
            dw = rng.uniform(-w_var, w_var)
            seg_h = r["height"] + dh
            seg_w = r["width"] + dw

            seg_x = row_start_x + seg_i * stride + seg_len / 2.0
            seg_z = seg_h / 2.0

            create_box(
                stage, f"{row_path}/Seg{seg_i}",
                size=(seg_len, seg_w, seg_h),
                position=(seg_x, y, seg_z),
                material=mat)


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
