#!/usr/bin/env python3
"""Generate an occupancy grid map from the known greenhouse geometry.

Produces a .pgm + .yaml pair compatible with nav2_map_server.
Run from repo root: python3 scripts/generate_map.py

The greenhouse layout (from greenhouse_simple.sdf):
  - Outer walls: X=[0,30], Y=[-7.5, 7.5]
  - 6 crop rows at Y = -5.5, -3.3, -1.1, 1.1, 3.3, 5.5 (20m long, 1m wide, starting at X=6)
  - Heating rails in aisles (thin, 51mm diameter — not obstacles for the robot)
  - Crates at various positions
  - Robot spawns at (2.0, 0.0)
"""

import struct
import os

# Map parameters
RESOLUTION = 0.05  # meters/pixel (matches nav2_params.yaml)
ORIGIN_X = -2.0    # 2m margin west of walls
ORIGIN_Y = -9.5    # 2m margin south of walls
MAP_WIDTH_M = 36.0   # covers X=[-2, 34]
MAP_HEIGHT_M = 19.0  # covers Y=[-9.5, 9.5]

WIDTH = int(MAP_WIDTH_M / RESOLUTION)   # 720 pixels
HEIGHT = int(MAP_HEIGHT_M / RESOLUTION)  # 380 pixels

# Pixel values: 254=free, 0=occupied, 205=unknown
FREE = 254
OCCUPIED = 0

# Wall thickness in meters
WALL_T = 0.2


def world_to_pixel(wx, wy):
    px = int((wx - ORIGIN_X) / RESOLUTION)
    py = int((wy - ORIGIN_Y) / RESOLUTION)
    return px, py


def fill_rect(grid, x1, y1, x2, y2, value=OCCUPIED):
    """Fill a rectangle from world coordinates (x1,y1) to (x2,y2)."""
    px1, py1 = world_to_pixel(min(x1, x2), min(y1, y2))
    px2, py2 = world_to_pixel(max(x1, x2), max(y1, y2))
    px1 = max(0, px1)
    py1 = max(0, py1)
    px2 = min(WIDTH - 1, px2)
    py2 = min(HEIGHT - 1, py2)
    for y in range(py1, py2 + 1):
        for x in range(px1, px2 + 1):
            grid[y][x] = value


def main():
    # Initialize grid as free space
    grid = [[FREE] * WIDTH for _ in range(HEIGHT)]

    # Outer walls (30m x 15m greenhouse, walls at boundary)
    # North wall: Y=7.5, X=[0,30]
    fill_rect(grid, 0, 7.5 - WALL_T / 2, 30, 7.5 + WALL_T / 2)
    # South wall: Y=-7.5
    fill_rect(grid, 0, -7.5 - WALL_T / 2, 30, -7.5 + WALL_T / 2)
    # West wall: X=0
    fill_rect(grid, -WALL_T / 2, -7.5, WALL_T / 2, 7.5)
    # East wall: X=30
    fill_rect(grid, 30 - WALL_T / 2, -7.5, 30 + WALL_T / 2, 7.5)

    # 6 crop rows (20m long, 1m wide, centered at X=16, starting at X=6)
    crop_ys = [-5.5, -3.3, -1.1, 1.1, 3.3, 5.5]
    for cy in crop_ys:
        fill_rect(grid, 6.0, cy - 0.5, 26.0, cy + 0.5)

    # Crates (obstacles)
    # Crate 1: (3.5, -2.0), 0.5 x 0.4
    fill_rect(grid, 3.5 - 0.25, -2.0 - 0.2, 3.5 + 0.25, -2.0 + 0.2)
    # Crate 2: (4.0, 3.5), 0.5 x 0.4
    fill_rect(grid, 4.0 - 0.25, 3.5 - 0.2, 4.0 + 0.25, 3.5 + 0.2)
    # Crate 3: (25.0, 0.0), 0.5 x 0.4
    fill_rect(grid, 25.0 - 0.25, 0.0 - 0.2, 25.0 + 0.25, 0.0 + 0.2)

    # Heating rails are 51mm diameter — too thin to be meaningful obstacles
    # for the robot (735mm track width). Omitting from map.

    # Write PGM (P5 binary format)
    output_dir = os.path.join(os.path.dirname(__file__),
                              '..', 'src', 'agv_sim_bringup', 'maps')
    os.makedirs(output_dir, exist_ok=True)

    pgm_path = os.path.join(output_dir, 'greenhouse_simple.pgm')
    yaml_path = os.path.join(output_dir, 'greenhouse_simple.yaml')

    with open(pgm_path, 'wb') as f:
        header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
        f.write(header.encode('ascii'))
        # PGM rows go top-to-bottom, but map Y increases upward
        # So we write rows in reverse order
        for row in reversed(grid):
            f.write(bytes(row))

    # Write YAML
    with open(yaml_path, 'w') as f:
        f.write(f"image: greenhouse_simple.pgm\n")
        f.write(f"resolution: {RESOLUTION}\n")
        f.write(f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n")

    print(f"Map generated: {pgm_path}")
    print(f"Map config:    {yaml_path}")
    print(f"Size: {WIDTH}x{HEIGHT} pixels, {MAP_WIDTH_M}x{MAP_HEIGHT_M}m at {RESOLUTION}m/px")
    print(f"Origin: ({ORIGIN_X}, {ORIGIN_Y})")


if __name__ == '__main__':
    main()
