"""Static AprilTag placements in the greenhouse.

Mirrors APRILTAG_PLACEMENTS from
src/agv_isaac_sim/scripts/build_greenhouse_usd.py — kept here as a copy on
purpose so the validation package does not import Isaac Sim modules.

Tuple format: (id, x, y, z, rx_deg, ry_deg, rz_deg).
World frame: +X east, +Y north, +Z up.
The tag's normal is its local +Y axis after the rpy rotation; for a tag with
rx=0, ry=0, rz=θ, the normal points along +Y rotated by θ around Z, which in
the conventions used here means "facing in the direction of the rotation".

For the visibility test we precompute the outward normal as a unit vector in
the XY plane.
"""
import math


APRILTAG_PLACEMENTS = [
    # Wall-mounted tags (vertical)
    (0, -16.88, 0.0, 0.145, 0, 0, 0),          # West wall center, facing +X
    (1, 27.88, 0.0, 0.145, 0, 0, 180),          # East wall center, facing -X
    (5, -16.88, -5.0, 0.145, 0, 0, 0),          # West wall south, facing +X
    (14, 15.0, 7.38, 0.145, 0, 0, -90),         # North wall, facing -Y
    (15, 15.0, -7.38, 0.145, 0, 0, 90),         # South wall, facing +Y
    # East wall per-aisle (vertical, facing -X)
    (16, 27.88, -4.4, 0.145, 0, 0, 180),
    (17, 27.88, -2.2, 0.145, 0, 0, 180),
    (18, 27.88, 0.0, 0.145, 0, 0, 180),
    (19, 27.88, 2.2, 0.145, 0, 0, 180),
    (20, 27.88, 4.4, 0.145, 0, 0, 180),
    # West wall per-aisle, rear section (vertical, facing +X)
    (21, -16.88, -4.4, 0.145, 0, 0, 0),
    (22, -16.88, -2.2, 0.145, 0, 0, 0),
    (23, -16.88, 0.0, 0.145, 0, 0, 0),
    (24, -16.88, 2.2, 0.145, 0, 0, 0),
    (25, -16.88, 4.4, 0.145, 0, 0, 0),
    # Front floor aisles (horizontal, face UP +Z)
    (2, 7.0, -4.4, 0.002, 0, -90, 0),
    (3, 7.0, -2.2, 0.002, 0, -90, 0),
    (4, 7.0, 0.0, 0.002, 0, -90, 0),
    (12, 7.0, 2.2, 0.002, 0, -90, 0),
    (13, 7.0, 4.4, 0.002, 0, -90, 0),
    # Rear floor aisles (horizontal, face UP +Z)
    (33, 4.0, -4.4, 0.002, 0, -90, 0),
    (34, 4.0, -2.2, 0.002, 0, -90, 0),
    (35, 4.0, 0.0, 0.002, 0, -90, 0),
    (36, 4.0, 2.2, 0.002, 0, -90, 0),
    (37, 4.0, 4.4, 0.002, 0, -90, 0),
    # Front row starts (vertical, facing -X toward approaching robot)
    (6, 7.4, -5.5, 0.145, 0, 0, 180),
    (7, 7.4, -3.3, 0.145, 0, 0, 180),
    (8, 7.4, -1.1, 0.145, 0, 0, 180),
    (9, 7.4, 1.1, 0.145, 0, 0, 180),
    (10, 7.4, 3.3, 0.145, 0, 0, 180),
    (11, 7.4, 5.5, 0.145, 0, 0, 180),
    # Rear row starts (vertical, facing +X toward approaching robot)
    (26, 3.6, -5.5, 0.145, 0, 0, 0),
    (27, 3.6, -3.3, 0.145, 0, 0, 0),
    (28, 3.6, -1.1, 0.145, 0, 0, 0),
    (29, 3.6, 1.1, 0.145, 0, 0, 0),
    (30, 3.6, 3.3, 0.145, 0, 0, 0),
    (31, 3.6, 5.5, 0.145, 0, 0, 0),
]


def tag_normal_xy(rx_deg: float, ry_deg: float, rz_deg: float):
    """Outward normal of a tag in the XY plane (z component ignored).

    For ceiling/floor tags (ry = ±90) the normal is vertical and the function
    returns (0, 0) — the visibility test treats those as 'always not visible
    from the front camera' which matches reality (the ZED looks +X horizontal).
    """
    if abs(abs(ry_deg) - 90.0) < 1e-3:
        return (0.0, 0.0)
    rad = math.radians(rz_deg)
    # Tag faces along its local +X after rz rotation about Z (matches the
    # placements above where rz=0 → faces +X, rz=180 → faces -X).
    return (math.cos(rad), math.sin(rad))


def is_floor_tag(rx_deg: float, ry_deg: float, rz_deg: float) -> bool:
    """True if the tag is mounted horizontal (face up). Not visible by the
    forward-looking camera unless the robot is directly above it."""
    return abs(abs(ry_deg) - 90.0) < 1e-3
