"""Physics material assignment for robot components.

Friction values calibrated to real-world measurements:
- Rubber on concrete/soil: μs=0.7, μd=0.6
- Nylon ball casters on hard floor: μs=0.03, μd=0.025
"""

from pxr import UsdPhysics, UsdShade, PhysxSchema


def _create_physics_material(stage, path, static_friction, dynamic_friction, restitution=0.0):
    """Create a PhysX material prim."""
    mat_prim = UsdShade.Material.Define(stage, path).GetPrim()
    phys_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    phys_mat.CreateStaticFrictionAttr(static_friction)
    phys_mat.CreateDynamicFrictionAttr(dynamic_friction)
    phys_mat.CreateRestitutionAttr(restitution)
    return mat_prim


def setup_physics_materials(stage, robot_path):
    """Apply calibrated friction materials to wheels and casters."""
    print("Setting up physics materials...")

    _create_physics_material(
        stage, f"{robot_path}/Materials/WheelMaterial",
        static_friction=0.7, dynamic_friction=0.6)

    _create_physics_material(
        stage, f"{robot_path}/Materials/CasterMaterial",
        static_friction=0.03, dynamic_friction=0.025)

    for wheel in ["left_wheel", "right_wheel"]:
        prim = stage.GetPrimAtPath(f"{robot_path}/{wheel}")
        if prim:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)

    for caster in ["front_caster", "rear_caster"]:
        prim = stage.GetPrimAtPath(f"{robot_path}/{caster}")
        if prim:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)

    print("  Wheels: μs=0.7/μd=0.6 (rubber on soil)")
    print("  Casters: μs=0.03/μd=0.025 (nylon on floor)")
