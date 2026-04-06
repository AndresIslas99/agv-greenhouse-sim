"""Physics material assignment for robot components."""

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
    """Apply friction materials to wheels and casters."""
    print("Setting up physics materials...")

    _create_physics_material(
        stage, f"{robot_path}/Materials/WheelMaterial",
        static_friction=1.0, dynamic_friction=1.0)

    _create_physics_material(
        stage, f"{robot_path}/Materials/CasterMaterial",
        static_friction=0.001, dynamic_friction=0.001)

    # Bind materials to collision geometry
    for wheel in ["left_wheel", "right_wheel"]:
        prim = stage.GetPrimAtPath(f"{robot_path}/{wheel}")
        if prim:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)

    for caster in ["front_caster", "rear_caster"]:
        prim = stage.GetPrimAtPath(f"{robot_path}/{caster}")
        if prim:
            PhysxSchema.PhysxMaterialAPI.Apply(prim)

    print("  Wheels: friction=1.0, Casters: friction=0.001")
