"""Greenhouse structural elements: ground, walls, crop rows, rails, roof beams.

Crop rows use a segmented plant model with stems and canopy to produce
realistic depth camera returns (not flat walls). Each plant has:
- A thin stem cylinder at the base
- A wider canopy box at the top
- Random height/width variation for visual diversity
- Gaps between plants (air visible to depth camera)
"""

import random

from pxr import UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

from .primitives import create_box, create_cylinder


def create_ground(stage, materials, cfg):
    """Create the soil ground plane with optional drainage slope.

    Real greenhouses have 0.5-1.0% longitudinal slope for water runoff.
    This causes subtle IMU tilt and odometry drift on the real robot.
    """
    g = cfg["ground"]
    slope = g.get("slope_percent", 0.0)

    if slope > 0:
        # Sloped ground: rotate around Y axis
        # 0.5% slope = atan(0.005) ≈ 0.286 degrees
        import math
        slope_deg = math.degrees(math.atan(slope / 100.0))
        create_box(stage, "/World/Ground",
                   size=g["size"], position=g["position"],
                   rotation_deg=(0.0, slope_deg, 0.0),
                   material=materials["soil"])
    else:
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


def _create_plant(stage, path, stem_h, canopy_h, canopy_w, canopy_d,
                  stem_radius, material):
    """Create a single plant: thin stem cylinder + wider canopy box.

    Structure:
        Xform "Plant"
          ├── Stem (cylinder, r=0.015m, from ground to canopy base)
          └── Canopy (box, wider than stem, at top)
    """
    xform = UsdGeom.Xform.Define(stage, path)

    # Stem: thin cylinder from ground to canopy base
    stem_path = f"{path}/Stem"
    stem_xf = UsdGeom.Xform.Define(stage, stem_path)
    stem_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, stem_h / 2.0))
    stem = UsdGeom.Cylinder.Define(stage, f"{stem_path}/Mesh")
    stem.CreateRadiusAttr(stem_radius)
    stem.CreateHeightAttr(stem_h)
    stem.CreateAxisAttr("Z")

    # Canopy: wider box sitting on top of stem
    canopy_path = f"{path}/Canopy"
    canopy_xf = UsdGeom.Xform.Define(stage, canopy_path)
    canopy_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, stem_h + canopy_h / 2.0))
    canopy = UsdGeom.Cube.Define(stage, f"{canopy_path}/Mesh")
    canopy.AddScaleOp().Set(Gf.Vec3f(canopy_d / 2.0, canopy_w / 2.0, canopy_h / 2.0))

    # Collision on canopy only (stem too thin to matter for navigation)
    UsdPhysics.CollisionAPI.Apply(canopy.GetPrim())

    # Material on canopy (stem gets default dark green)
    if material:
        UsdShade.MaterialBindingAPI(canopy.GetPrim()).Bind(material)

    return xform


def create_crop_rows(stage, materials, cfg):
    """Create crop rows as segmented plants with stems and canopy.

    Each row is a series of plants with gaps between them, giving
    realistic depth camera returns (individual plant silhouettes
    with visible stems and leaf mass, not flat walls).
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

    # Plant geometry ratios
    stem_ratio = 0.35          # stem is 35% of total height
    stem_radius = 0.015        # 15mm stem diameter (realistic for tomato/pepper)
    canopy_overhang = 0.1      # canopy 10cm wider than base segment

    rng = random.Random(42)

    for row_i, y in enumerate(r["y_positions"]):
        row_path = f"/World/CropRows/Row{row_i + 1}"
        UsdGeom.Xform.Define(stage, row_path)
        mat = materials[mat_keys[row_i % 2]]

        num_segments = int(r["length"] / stride)
        for seg_i in range(num_segments):
            dh = rng.uniform(-h_var, h_var)
            dw = rng.uniform(-w_var, w_var)
            total_h = r["height"] + dh
            base_w = r["width"] + dw

            stem_h = total_h * stem_ratio
            canopy_h = total_h * (1.0 - stem_ratio)
            canopy_w = base_w + canopy_overhang
            canopy_d = seg_len

            seg_x = row_start_x + seg_i * stride + seg_len / 2.0

            plant_xf = _create_plant(
                stage, f"{row_path}/Plant{seg_i}",
                stem_h=stem_h, canopy_h=canopy_h,
                canopy_w=canopy_w, canopy_d=canopy_d,
                stem_radius=stem_radius, material=mat)
            plant_xf.AddTranslateOp().Set(Gf.Vec3d(seg_x, y, 0.0))


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
