# agv_sim_worlds

Gazebo SDF worlds, environment models, and PBR textures.

## Key Files

- `worlds/greenhouse_simple.sdf` — Main world (6 crop rows, 5 aisles, 6 AprilTag models)
- `worlds/nav_test.sdf` — Simplified navigation test world
- `models/apriltag_{0-5}/` — Visual AprilTag models with tag36h11 textures
- `models/crate/` — Obstacle crate model
- `models/greenhouse_row/` — Heating pipe rail model
- `textures/` and `materials/` — Shared PBR assets

## Rules

- All models go under `models/` with `model.config` + `model.sdf`
- Visual surfaces MUST have PBR textures (ZED depth needs texture contrast)
- AprilTag visual models use IDs 0-5 only (textures exist for these)
- Additional tags (6+) are registry-only for the fake_marker_detector
