"""Load and resolve world configuration from YAML."""

import os
import yaml


def load_world_config(config_path=None):
    """Load world_config.yaml and resolve texture paths."""
    if config_path is None:
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        pkg_dir = os.path.dirname(script_dir)
        config_path = os.path.join(pkg_dir, "config", "world_config.yaml")

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    # Resolve texture base directories to absolute paths
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    cfg["_textures_dir"] = os.path.join(src_dir, cfg["textures"]["base_dir"])
    cfg["_tag_textures_dir"] = os.path.join(src_dir, cfg["textures"]["tag_dir"])
    cfg["_output_path"] = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "..", "worlds", "greenhouse_simple.usd")

    return cfg


def texture_path(cfg, filename):
    """Resolve a texture filename to its absolute path."""
    return os.path.join(cfg["_textures_dir"], filename)


def tag_texture_path(cfg, tag_id):
    """Resolve an AprilTag texture path by tag ID."""
    return os.path.join(cfg["_tag_textures_dir"], f"tag36h11_id{tag_id}.png")
