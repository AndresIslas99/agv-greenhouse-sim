"""URDF processing and import into Isaac Sim."""

import subprocess

from omni.isaac.urdf import _urdf as urdf_importer

from .config import MAIN_XACRO, TEMP_URDF


def process_xacro():
    """Run xacro to produce a plain URDF file."""
    print(f"Processing xacro: {MAIN_XACRO}")
    result = subprocess.run(
        ["xacro", MAIN_XACRO],
        capture_output=True, text=True, timeout=30)

    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr}")

    with open(TEMP_URDF, "w") as f:
        f.write(result.stdout)

    print(f"URDF written to: {TEMP_URDF}")
    return TEMP_URDF


def import_urdf(urdf_path, robot_path="/Robot"):
    """Import URDF into the current USD stage."""
    print("Importing URDF into Isaac Sim...")

    import_config = urdf_importer.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.import_inertia_tensor = True
    import_config.default_drive_type = 1  # Velocity drive
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3
    import_config.create_physics_scene = False

    result = urdf_importer.import_robot(urdf_path, import_config, robot_path)
    if not result:
        raise RuntimeError("URDF import failed")

    print(f"URDF imported as {robot_path}")
    return robot_path
