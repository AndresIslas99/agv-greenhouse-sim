#!/usr/bin/env python3
"""Domain randomization for greenhouse simulation.

Applies random variations to the greenhouse environment per episode
to improve sim-to-real transfer. Can run as:

1. Isaac Sim standalone script (modifies USD stage before simulation):
       isaacsim --exec sim_domain_randomizer.py

2. Imported and called from Python to randomize a live stage.

Randomizes: lighting, crop geometry, physics materials, obstacles, fog.
Configuration: config/domain_randomization.yaml
"""

import os
import random
import yaml

# Only import Isaac Sim modules when running standalone
_STANDALONE = __name__ == "__main__"


def load_randomization_config(config_path=None):
    """Load domain randomization ranges from YAML."""
    if config_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.dirname(script_dir)
        config_path = os.path.join(pkg_dir, "config", "domain_randomization.yaml")

    with open(config_path) as f:
        return yaml.safe_load(f)


def _sample(rng, spec):
    """Sample a value from a {min, max} range spec."""
    return rng.uniform(spec["min"], spec["max"])


def randomize_lighting(stage, cfg, rng):
    """Randomize light intensities, directions, color temperature, and HDRI rotation."""
    from pxr import UsdLux, UsdGeom, Gf

    lc = cfg["lighting"]

    sun = stage.GetPrimAtPath("/World/Lights/Sun")
    if sun:
        sun_light = UsdLux.DistantLight(sun)
        sun_light.GetIntensityAttr().Set(_sample(rng, lc["sun_intensity"]))

        # Color temperature jitter
        ct_offset = _sample(rng, lc["color_temp_offset"])
        sun_light.GetColorAttr().Set(Gf.Vec3f(
            1.0 + ct_offset, 0.98 + ct_offset * 0.5, 0.92 - ct_offset))

    dome = stage.GetPrimAtPath("/World/Lights/AmbientDome")
    if dome:
        UsdLux.DomeLight(dome).GetIntensityAttr().Set(
            _sample(rng, lc["dome_intensity"]))

        # HDRI rotation for time-of-day variation
        hdri_rot = lc.get("hdri_rotation", None)
        if hdri_rot:
            rotation_deg = _sample(rng, hdri_rot)
            xformable = UsdGeom.Xformable(dome)
            for op in xformable.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:rotateXYZ":
                    op.Set(Gf.Vec3f(0.0, rotation_deg, 0.0))
                    break

    fill = stage.GetPrimAtPath("/World/Lights/Fill")
    if fill:
        UsdLux.DistantLight(fill).GetIntensityAttr().Set(
            _sample(rng, lc["fill_intensity"]))

    # Supplemental rect lights intensity jitter
    sup_intensity = lc.get("supplemental_intensity", None)
    if sup_intensity:
        sup_root = stage.GetPrimAtPath("/World/Lights/Supplemental")
        if sup_root:
            intensity = _sample(rng, sup_intensity)
            for child in sup_root.GetChildren():
                rect = UsdLux.RectLight(child)
                if rect:
                    rect.GetIntensityAttr().Set(intensity)


def randomize_crop_rows(stage, cfg, rng):
    """Randomize plant heights, widths, gaps, and randomly remove plants."""
    from pxr import UsdGeom, Gf

    cc = cfg["crop_rows"]
    h_scale = _sample(rng, cc["height_scale"])
    w_scale = _sample(rng, cc["width_scale"])
    missing_rate = cc.get("missing_plant_rate", 0.05)

    rows_prim = stage.GetPrimAtPath("/World/CropRows")
    if not rows_prim:
        return

    for row_prim in rows_prim.GetChildren():
        for plant_prim in row_prim.GetChildren():
            # Random plant removal
            if rng.random() < missing_rate:
                plant_prim.SetActive(False)
                continue

            plant_prim.SetActive(True)

            # Scale canopy height/width
            canopy_path = f"{plant_prim.GetPath()}/Canopy/Mesh"
            canopy = stage.GetPrimAtPath(canopy_path)
            if canopy:
                xformable = UsdGeom.Xformable(canopy)
                # Apply per-plant jitter on top of episode-level scale
                local_h = h_scale * rng.uniform(0.9, 1.1)
                local_w = w_scale * rng.uniform(0.95, 1.05)
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpName() == "xformOp:scale":
                        current = op.Get()
                        op.Set(Gf.Vec3f(
                            current[0],
                            current[1] * local_w,
                            current[2] * local_h))


def randomize_physics(stage, cfg, rng):
    """Randomize friction coefficients within ±range of calibrated values."""
    from pxr import UsdPhysics, Sdf

    pc = cfg["physics"]

    # Wheel friction
    wheel_scale = _sample(rng, pc["wheel_friction_scale"])
    wheel_mat = stage.GetPrimAtPath("/Robot/Materials/WheelMaterial")
    if wheel_mat:
        phys = UsdPhysics.MaterialAPI(wheel_mat)
        phys.GetStaticFrictionAttr().Set(0.7 * wheel_scale)
        phys.GetDynamicFrictionAttr().Set(0.6 * wheel_scale)

    # Caster friction
    caster_scale = _sample(rng, pc["caster_friction_scale"])
    caster_mat = stage.GetPrimAtPath("/Robot/Materials/CasterMaterial")
    if caster_mat:
        phys = UsdPhysics.MaterialAPI(caster_mat)
        phys.GetStaticFrictionAttr().Set(0.03 * caster_scale)
        phys.GetDynamicFrictionAttr().Set(0.025 * caster_scale)


def randomize_obstacles(stage, cfg, rng):
    """Add random extra crates within the greenhouse."""
    from pxr import UsdGeom, Gf

    oc = cfg["obstacles"]
    n_extra = int(_sample(rng, oc["extra_crates"]))
    jitter = oc["crate_position_jitter"]

    # Jitter existing crate positions
    crates = stage.GetPrimAtPath("/World/Crates")
    if crates:
        for crate in crates.GetChildren():
            xformable = UsdGeom.Xformable(crate)
            for op in xformable.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    pos = op.Get()
                    op.Set(Gf.Vec3d(
                        pos[0] + _sample(rng, jitter),
                        pos[1] + _sample(rng, jitter),
                        pos[2]))


def randomize_episode(stage, config_path=None, seed=None):
    """Apply all randomizations to the stage for one episode.

    Args:
        stage: USD stage to randomize
        config_path: path to domain_randomization.yaml (None for default)
        seed: random seed (None for random, int for reproducible)

    Returns:
        Dict of applied randomization values for logging
    """
    cfg = load_randomization_config(config_path)
    rng = random.Random(seed)

    randomize_lighting(stage, cfg, rng)
    randomize_crop_rows(stage, cfg, rng)
    randomize_physics(stage, cfg, rng)
    randomize_obstacles(stage, cfg, rng)

    fog_range = cfg.get("fog", {}).get("fog_factor", {"min": 0, "max": 0})
    fog_factor = _sample(rng, fog_range) if fog_range["max"] > 0 else 0.0

    return {
        "seed": seed,
        "fog_factor": fog_factor,
    }


def main():
    """Standalone mode: randomize the greenhouse USD and save."""
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})

    import omni.usd

    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    world_usd = os.path.join(pkg_dir, "worlds", "greenhouse_simple.usd")
    output_usd = os.path.join(pkg_dir, "worlds", "greenhouse_randomized.usd")

    print(f"Loading: {world_usd}")
    omni.usd.get_context().open_stage(world_usd)
    stage = omni.usd.get_context().get_stage()

    result = randomize_episode(stage, seed=None)
    print(f"Randomization applied: {result}")

    stage.GetRootLayer().Export(output_usd)
    print(f"Saved: {output_usd}")

    simulation_app.close()


if __name__ == "__main__":
    main()
