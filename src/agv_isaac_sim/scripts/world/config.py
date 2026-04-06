"""Load and resolve world configuration from YAML.

Path resolution strategy:
1. Walk up from __file__ looking for src/agv_sim_worlds/ (works in source tree)
2. Support AGV_SIM_SRC_DIR env var override for non-standard layouts
"""

import os
import yaml


def _find_src_dir():
    """Find the workspace src/ directory containing agv_sim_worlds."""
    current = os.path.abspath(__file__)
    for _ in range(10):
        current = os.path.dirname(current)
        candidate = os.path.join(current, "agv_sim_worlds", "models", "textures")
        if os.path.isdir(candidate):
            return current
        candidate = os.path.join(current, "src", "agv_sim_worlds", "models", "textures")
        if os.path.isdir(candidate):
            return os.path.join(current, "src")
    raise FileNotFoundError(
        "Cannot find agv_sim_worlds package. Set AGV_SIM_SRC_DIR env var.")


def _find_isaac_pkg_dir():
    """Find the agv_isaac_sim package directory."""
    current = os.path.abspath(__file__)
    for _ in range(10):
        current = os.path.dirname(current)
        pkg_xml = os.path.join(current, "package.xml")
        if os.path.exists(pkg_xml):
            with open(pkg_xml) as f:
                if "agv_isaac_sim" in f.read():
                    return current
    raise FileNotFoundError("Cannot find agv_isaac_sim package directory")


def load_world_config(config_path=None):
    """Load world_config.yaml and resolve texture paths."""
    if config_path is None:
        pkg_dir = _find_isaac_pkg_dir()
        config_path = os.path.join(pkg_dir, "config", "world_config.yaml")

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    src_dir = os.environ.get("AGV_SIM_SRC_DIR", None) or _find_src_dir()
    pkg_dir = _find_isaac_pkg_dir()

    cfg["_src_dir"] = src_dir
    cfg["_pkg_dir"] = pkg_dir
    cfg["_textures_dir"] = os.path.join(src_dir, cfg["textures"]["base_dir"])
    cfg["_tag_textures_dir"] = os.path.join(src_dir, cfg["textures"]["tag_dir"])
    cfg["_output_path"] = os.path.join(pkg_dir, "worlds", "greenhouse_simple.usd")

    return cfg


def texture_path(cfg, filename):
    """Resolve a texture filename to its absolute path."""
    if not filename:
        return ""
    return os.path.join(cfg["_textures_dir"], filename)


def tag_texture_path(cfg, tag_id):
    """Resolve an AprilTag texture path by tag ID."""
    return os.path.join(cfg["_tag_textures_dir"], f"tag36h11_id{tag_id}.png")
