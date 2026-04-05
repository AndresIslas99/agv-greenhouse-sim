# agv_sim_apriltags

AprilTag marker registry and fake proximity-based detector.

## Key Files

- `config/markers_registry.yaml` — Tag positions (map frame), family, detection range
- `config/apriltag_ros_params.yaml` — apriltag_ros node parameters
- `scripts/fake_marker_detector.py` — Proximity-based marker detection (no camera needed)
- `launch/fake_markers.launch.py` — Launch fake detector
- `launch/apriltag_detection.launch.py` — Launch real apriltag_ros

## Rules

- Visual tag models (with textures) exist for IDs 0-5 only
- Tags 6+ are registry-only — used by fake_marker_detector but have no SDF model
- All tag poses use map frame with quaternion orientation
- fake_marker_detector uses TF to compute proximity, publishes PoseWithCovarianceStamped
