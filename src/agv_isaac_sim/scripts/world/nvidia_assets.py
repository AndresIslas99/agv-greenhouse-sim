"""NVIDIA Isaac Sim asset integration helpers.

Isaac Sim 4.5 serves assets from a cloud S3 bucket. This module provides
helpers for resolving asset URLs and adding USD references to NVIDIA assets.

Asset loading works via USD references — the asset USD is loaded at runtime
when the stage is opened, not at build time. This means:
- No local download needed for most assets
- Assets are cached by Isaac Sim after first load
- Internet connection required on first use (or use local cache)

Usage:
    from world.nvidia_assets import get_assets_root, add_asset_reference

    root = get_assets_root()
    add_asset_reference(stage, "/World/Props/Pallet0",
                        f"{root}/Isaac/Props/KLT_Bin/small_KLT.usd",
                        position=(3.0, -2.0, 0.0))
"""

import os

from pxr import UsdGeom, Gf

# S3 bucket root for Isaac Sim 4.5 assets
_S3_ASSETS_ROOT = ("https://omniverse-content-production.s3-us-west-2."
                    "amazonaws.com/Assets/Isaac/4.5")

# Common asset paths (relative to assets root)
# These are examples — actual paths should be discovered via the asset browser
# or by probing the S3 bucket. Update as you find working paths.
KNOWN_ASSETS = {
    # Containers / crates
    "small_klt": "/Isaac/Props/KLT_Bin/small_KLT.usd",
    "large_klt": "/Isaac/Props/KLT_Bin/large_KLT.usd",
    # Pallets
    "pallet": "/Isaac/Props/Pallet/pallet.usd",
    # Warehouse items (example paths — verify availability)
    "cardboard_box_a": "/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd",
}


def get_assets_root():
    """Get the NVIDIA Isaac Sim asset root URL.

    Tries the Isaac Sim SDK path resolver first (works inside Isaac Sim),
    then falls back to the known S3 URL.

    Returns:
        str: Asset root URL (no trailing slash)
    """
    # Try Isaac Sim native resolver
    try:
        from isaacsim.storage.native import get_assets_root_path
        root = get_assets_root_path()
        if root:
            return root.rstrip("/")
    except (ImportError, AttributeError):
        pass

    # Try environment variable override
    env_root = os.environ.get("ISAAC_ASSETS_ROOT", "")
    if env_root:
        return env_root.rstrip("/")

    # Local cache (if assets were downloaded offline)
    local_cache = os.path.join(os.path.expanduser("~"), ".isaac_sim", "assets")
    if os.path.isdir(local_cache):
        return local_cache

    return _S3_ASSETS_ROOT


def add_asset_reference(stage, prim_path, asset_url, position=(0, 0, 0),
                        rotation_deg=(0, 0, 0), scale=1.0, instanceable=False):
    """Add a USD reference to an external asset at the given transform.

    Args:
        stage: USD stage
        prim_path: where to create the prim (e.g. "/World/Props/Crate0")
        asset_url: full URL or path to the USD asset
        position: (x, y, z) translation
        rotation_deg: (rx, ry, rz) Euler rotation in degrees
        scale: uniform scale factor
        instanceable: if True, mark as instanceable for GPU memory sharing

    Returns:
        UsdGeom.Xform prim
    """
    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    if any(r != 0 for r in rotation_deg):
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_deg))

    if scale != 1.0:
        xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))

    prim = xform.GetPrim()
    prim.GetReferences().AddReference(asset_url)

    if instanceable:
        prim.SetInstanceable(True)

    return xform


def add_props(stage, cfg):
    """Add environmental props to the greenhouse scene.

    Adds pallets, extra containers, and other props to break visual
    uniformity and increase realism. Props are loaded as USD references
    from the NVIDIA asset library.

    Props configuration in world_config.yaml:
        props:
          enabled: true
          items:
            - {asset: "pallet", position: [2.0, -6.0, 0.0], yaw: 15.0}
            - {asset: "small_klt", position: [2.5, -5.5, 0.0], yaw: 0.0}
    """
    props_cfg = cfg.get("props", {})
    if not props_cfg.get("enabled", False):
        return

    root = get_assets_root()
    UsdGeom.Xform.Define(stage, "/World/Props")

    items = props_cfg.get("items", [])
    loaded = 0
    for i, item in enumerate(items):
        asset_key = item.get("asset", "")
        asset_path = KNOWN_ASSETS.get(asset_key, "")
        if not asset_path:
            # Try as direct path
            asset_path = asset_key

        if not asset_path:
            continue

        # KNOWN_ASSETS paths start with / (e.g. /Isaac/Props/...) — prepend the root
        if asset_path.startswith("http"):
            full_url = asset_path
        else:
            full_url = f"{root}{asset_path}"
        position = item.get("position", [0, 0, 0])
        yaw = item.get("yaw", 0.0)
        scale = item.get("scale", 1.0)

        add_asset_reference(
            stage, f"/World/Props/Prop{i}",
            full_url,
            position=tuple(position),
            rotation_deg=(0.0, 0.0, yaw),
            scale=scale,
            instanceable=item.get("instanceable", False),
        )
        loaded += 1

    if loaded > 0:
        print(f"  Props: {loaded} NVIDIA assets loaded")
