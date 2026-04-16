"""Visual enhancements for the AGV robot in Isaac Sim.

Adds detailed visual-only geometry over the URDF primitives to make
the robot look more realistic. Does NOT modify collision geometry,
TF frames, mass, or inertial properties.

Components:
- Chassis: multi-panel body with deck plate, side rails, bumpers,
  electronics enclosure, and wheel guards
- Wheels: hub + center cap inside the tire cylinder

The ZED 2i camera is left untouched (already looks good).
"""

import sys
import os

from pxr import UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

# Add parent scripts dir to path for world.primitives import
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from world.primitives import create_omnipbr_material


# ── Materials ────────────────────────────────────────────────────

def _create_materials(stage, robot_path):
    """Create OmniPBR materials for visual detail components."""
    MAT = f"{robot_path}/Materials"

    mats = {}

    # Chassis body — keep the existing brushed aluminum from import_robot_usd.py
    # (already bound to base_link)

    # Top deck plate — stainless steel
    mats["deck"] = create_omnipbr_material(
        stage, f"{MAT}/DeckPlate",
        metallic=0.7, roughness=0.3,
        diffuse_color=Gf.Vec3f(0.55, 0.55, 0.58))

    # Side structural rails — dark anodized aluminum
    mats["rail"] = create_omnipbr_material(
        stage, f"{MAT}/SideRail",
        metallic=0.5, roughness=0.4,
        diffuse_color=Gf.Vec3f(0.25, 0.25, 0.28))

    # Bumpers — rubber
    mats["bumper"] = create_omnipbr_material(
        stage, f"{MAT}/Bumper",
        metallic=0.0, roughness=0.9,
        diffuse_color=Gf.Vec3f(0.08, 0.08, 0.08))

    # Electronics enclosure — painted metal
    mats["elec"] = create_omnipbr_material(
        stage, f"{MAT}/ElecBox",
        metallic=0.3, roughness=0.6,
        diffuse_color=Gf.Vec3f(0.18, 0.18, 0.22))

    # Wheel guards — dark recessed panels
    mats["guard"] = create_omnipbr_material(
        stage, f"{MAT}/WheelGuard",
        metallic=0.2, roughness=0.7,
        diffuse_color=Gf.Vec3f(0.12, 0.12, 0.12))

    # Wheel hub — dark aluminum
    mats["hub"] = create_omnipbr_material(
        stage, f"{MAT}/WheelHub",
        metallic=0.6, roughness=0.3,
        diffuse_color=Gf.Vec3f(0.30, 0.30, 0.32))

    # Center cap — polished metal
    mats["cap"] = create_omnipbr_material(
        stage, f"{MAT}/CenterCap",
        metallic=0.8, roughness=0.15,
        diffuse_color=Gf.Vec3f(0.65, 0.65, 0.68))

    return mats


# ── Geometry helpers ─────────────────────────────────────────────

def _add_cube(stage, path, size, position, material=None):
    """Add a visual-only cube prim (no collision, no physics)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3f(size[0] / 2, size[1] / 2, size[2] / 2))
    if material:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    return cube


def _add_cylinder(stage, path, radius, height, position, material=None):
    """Add a visual-only cylinder prim (no collision, no physics)."""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    xf = UsdGeom.Xformable(cyl.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    if material:
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim())
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(material)
    return cyl


def _hide_urdf_visual(stage, link_path):
    """Hide the original URDF visual mesh under a link prim.

    After URDF import, visual geometry is under the link as a child Gprim.
    We make it invisible so our detail geometry shows instead.
    The collision geometry is separate and unaffected.
    """
    link_prim = stage.GetPrimAtPath(link_path)
    if not link_prim:
        return

    for child in link_prim.GetAllChildren():
        # Look for visual geometry prims (Mesh, Cube, Cylinder, Sphere)
        if child.IsA(UsdGeom.Gprim):
            # Skip if it has CollisionAPI (that's collision geometry, not visual)
            if child.HasAPI(UsdPhysics.CollisionAPI):
                continue
            # Check if it's under a "visuals" scope or is direct visual
            path_str = str(child.GetPath())
            if "collision" in path_str.lower():
                continue
            UsdGeom.Imageable(child).MakeInvisible()


# ── Chassis enhancement ──────────────────────────────────────────

def enhance_chassis_visual(stage, robot_path, mats):
    """Replace the plain chassis box with a multi-panel robot body.

    The URDF chassis is a 1.0x0.6x0.15m box at offset (0.2, 0, 0)
    relative to base_link. We hide it and add detailed geometry.

    All positions are relative to base_link origin.
    """
    base_link = f"{robot_path}/base_link"
    vis = f"{base_link}/VisualDetail"
    UsdGeom.Xform.Define(stage, vis)

    # Hide the original URDF box
    _hide_urdf_visual(stage, base_link)

    # Chassis offset (matches URDF visual origin)
    cx = 0.2

    # Main body — slightly smaller than collision box for panel gap effect
    _add_cube(stage, f"{vis}/MainBody",
              size=(0.96, 0.56, 0.12),
              position=(cx, 0, 0),
              material=mats.get("deck"))  # uses the existing chassis material via binding

    # Top deck plate — thin stainless steel sheet on top
    _add_cube(stage, f"{vis}/TopDeck",
              size=(0.88, 0.52, 0.006),
              position=(cx, 0, 0.063),
              material=mats["deck"])

    # Side structural rails (left and right)
    for side, y_sign in [("Left", 1), ("Right", -1)]:
        _add_cube(stage, f"{vis}/SideRail{side}",
                  size=(0.96, 0.02, 0.04),
                  position=(cx, y_sign * 0.29, -0.02),
                  material=mats["rail"])

    # Front bumper — rubber strip
    _add_cube(stage, f"{vis}/FrontBumper",
              size=(0.04, 0.52, 0.06),
              position=(cx + 0.50, 0, -0.02),
              material=mats["bumper"])

    # Rear bumper
    _add_cube(stage, f"{vis}/RearBumper",
              size=(0.04, 0.52, 0.06),
              position=(cx - 0.50, 0, -0.02),
              material=mats["bumper"])

    # Electronics enclosure on top
    _add_cube(stage, f"{vis}/ElecBox",
              size=(0.30, 0.38, 0.08),
              position=(cx - 0.05, 0, 0.10),
              material=mats["elec"])

    # Wheel guard panels (dark insets where wheels are)
    wheel_y = 0.3675  # half of track width
    for side, y_sign in [("Left", 1), ("Right", -1)]:
        _add_cube(stage, f"{vis}/WheelGuard{side}",
                  size=(0.16, 0.06, 0.14),
                  position=(cx, y_sign * 0.28, 0),
                  material=mats["guard"])

    # Bottom skid plate — thin dark plate underneath
    _add_cube(stage, f"{vis}/SkidPlate",
              size=(0.90, 0.50, 0.004),
              position=(cx, 0, -0.062),
              material=mats["guard"])

    print("  Chassis: multi-panel body (deck, rails, bumpers, elec box, wheel guards)")


# ── Wheel enhancement ────────────────────────────────────────────

def enhance_wheel_visual(stage, robot_path, wheel_name, mats):
    """Add hub and center cap inside the tire cylinder.

    The wheel joint has rpy="-pi/2 0 0" so in the wheel link's local
    frame, Z is the rotation axis and the cylinder extends along Z.
    Tire: r=0.0781m, h=0.05m (already exists from URDF import).
    """
    wheel_path = f"{robot_path}/{wheel_name}_wheel"
    vis = f"{wheel_path}/VisualDetail"
    UsdGeom.Xform.Define(stage, vis)

    # Hub — visible through the open ends of the tire cylinder
    _add_cylinder(stage, f"{vis}/Hub",
                  radius=0.045, height=0.04,
                  position=(0, 0, 0),
                  material=mats["hub"])

    # Center cap — outer face
    _add_cylinder(stage, f"{vis}/OuterCap",
                  radius=0.015, height=0.006,
                  position=(0, 0, 0.022),
                  material=mats["cap"])

    # Center cap — inner face
    _add_cylinder(stage, f"{vis}/InnerCap",
                  radius=0.015, height=0.006,
                  position=(0, 0, -0.022),
                  material=mats["cap"])


# ── Main entry point ─────────────────────────────────────────────

def enhance_robot_visuals(stage, robot_path):
    """Add visual detail geometry to the robot for RTX realism.

    Adds multi-panel chassis, wheel hubs, and center caps.
    Does NOT modify collision, mass, TF frames, or sensors.
    """
    print("Enhancing robot visuals...")

    mats = _create_materials(stage, robot_path)

    enhance_chassis_visual(stage, robot_path, mats)

    for wheel in ["left", "right"]:
        enhance_wheel_visual(stage, robot_path, wheel, mats)
        print(f"  {wheel.capitalize()} wheel: hub + center caps")

    print("  Visual enhancements complete (visual-only, no physics changes)")
