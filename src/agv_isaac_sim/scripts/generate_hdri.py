#!/usr/bin/env python3
"""Generate a procedural greenhouse HDRI environment map.

Creates a latitude-longitude HDR image that simulates the light
environment inside a greenhouse:
- Bright warm zenith (sunlight diffused through polycarbonate roof)
- Dim green-tinted horizon (scattered light from vegetation)
- Subtle warm gradient from soil reflections below

Output: .hdr Radiance format, suitable for UsdLux.DomeLight textureFile.

Run: python3 scripts/generate_hdri.py
"""

import os
import struct
import math
import numpy as np


def generate_greenhouse_hdri(width=1024, height=512,
                             zenith_intensity=3.0,
                             horizon_intensity=0.8,
                             ground_intensity=0.3,
                             sun_glow_intensity=5.0,
                             sun_elevation_deg=60.0,
                             sun_azimuth_deg=200.0):
    """Generate a greenhouse interior HDRI as float32 RGB array.

    The HDRI simulates diffuse light through a translucent polycarbonate
    roof — no sharp sun disk, just a bright glow region overhead with
    warm color temperature and soft falloff toward the horizon.

    Args:
        width, height: HDRI resolution in pixels
        zenith_intensity: peak brightness at zenith (cd/m^2 approx)
        horizon_intensity: brightness at horizon
        ground_intensity: brightness below horizon (soil reflections)
        sun_glow_intensity: brightness of the diffuse sun glow region
        sun_elevation_deg: sun position elevation (degrees above horizon)
        sun_azimuth_deg: sun position azimuth (degrees from north)

    Returns:
        numpy array (height, width, 3) float32
    """
    # Create latitude-longitude coordinates
    v = np.linspace(0, 1, height)[:, None]  # 0=top(zenith), 1=bottom(nadir)
    u = np.linspace(0, 1, width)[None, :]

    # Convert to spherical coordinates
    theta = v * math.pi           # polar angle: 0=zenith, pi=nadir
    phi = u * 2 * math.pi         # azimuthal angle

    # Elevation angle: 0=horizon, pi/2=zenith, -pi/2=nadir
    elevation = math.pi / 2 - theta

    # --- Sky dome (above horizon) ---
    # Smooth gradient from zenith to horizon
    sky_mask = np.broadcast_to((elevation > 0).astype(np.float32), (height, width))
    sky_blend = np.clip(elevation / (math.pi / 2), 0, 1)  # 0 at horizon, 1 at zenith

    # Base sky color: warm daylight through polycarbonate
    # Broadcast to full (height, width) shape via np.broadcast_to then copy
    intensity_blend = horizon_intensity + sky_blend * (zenith_intensity - horizon_intensity)
    sky_r = np.broadcast_to((0.95 + sky_blend * 0.05) * intensity_blend, (height, width)).copy()
    sky_g = np.broadcast_to((0.93 + sky_blend * 0.04) * intensity_blend, (height, width)).copy()
    sky_b = np.broadcast_to((0.88 + sky_blend * 0.02) * intensity_blend, (height, width)).copy()

    # --- Sun glow (diffuse, no sharp disk) ---
    sun_elev_rad = math.radians(sun_elevation_deg)
    sun_az_rad = math.radians(sun_azimuth_deg)

    # Direction vector of sun
    sun_x = math.cos(sun_elev_rad) * math.sin(sun_az_rad)
    sun_y = math.cos(sun_elev_rad) * math.cos(sun_az_rad)
    sun_z = math.sin(sun_elev_rad)

    # Direction vectors of each pixel
    px = np.cos(elevation) * np.sin(phi)
    py = np.cos(elevation) * np.cos(phi)
    pz = np.sin(elevation)

    # Angle between each pixel and sun direction
    cos_angle = np.clip(px * sun_x + py * sun_y + pz * sun_z, -1, 1)

    # Wide gaussian glow (sigma ~30 degrees) to simulate polycarbonate diffusion
    sigma = math.radians(30)
    sun_glow = np.exp(-np.arccos(cos_angle) ** 2 / (2 * sigma ** 2))
    sun_glow *= sun_glow_intensity

    # Warm sun glow color
    sky_r += sun_glow * 1.0
    sky_g += sun_glow * 0.95
    sky_b += sun_glow * 0.85

    # --- Ground hemisphere (below horizon) ---
    ground_mask = np.broadcast_to((elevation <= 0).astype(np.float32), (height, width))
    ground_blend = np.clip(-elevation / (math.pi / 2), 0, 1)

    # Warm brown-green from soil and vegetation reflections
    ground_r = np.broadcast_to(ground_intensity * (0.35 + ground_blend * 0.1), (height, width))
    ground_g = np.broadcast_to(ground_intensity * (0.30 + ground_blend * 0.05), (height, width))
    ground_b = np.broadcast_to(ground_intensity * (0.20 + ground_blend * 0.02), (height, width))

    # --- Combine ---
    r = sky_r * sky_mask + ground_r * ground_mask
    g = sky_g * sky_mask + ground_g * ground_mask
    b = sky_b * sky_mask + ground_b * ground_mask

    hdri = np.stack([r, g, b], axis=-1).astype(np.float32)
    return hdri


def write_hdr(filepath, image):
    """Write a float32 RGB image as Radiance .hdr format.

    Implements the RGBE (Radiance HDR) format:
    - Header with FORMAT=32-bit_rle_rgbe
    - Scanlines encoded as RGBE (shared exponent)
    """
    height, width, _ = image.shape

    with open(filepath, 'wb') as f:
        # Header
        f.write(b'#?RADIANCE\n')
        f.write(b'FORMAT=32-bit_rle_rgbe\n')
        f.write(b'\n')
        f.write(f'-Y {height} +X {width}\n'.encode())

        # Convert float RGB to RGBE
        for y in range(height):
            scanline = image[y]
            rgbe = np.zeros((width, 4), dtype=np.uint8)

            for x in range(width):
                r, g, b = float(scanline[x, 0]), float(scanline[x, 1]), float(scanline[x, 2])
                v = max(r, g, b)
                if v < 1e-32:
                    rgbe[x] = [0, 0, 0, 0]
                else:
                    # frexp: v = mantissa * 2^exponent, mantissa in [0.5, 1.0)
                    mantissa, exponent = math.frexp(v)
                    scale = mantissa * 256.0 / v
                    rgbe[x, 0] = min(255, int(r * scale))
                    rgbe[x, 1] = min(255, int(g * scale))
                    rgbe[x, 2] = min(255, int(b * scale))
                    rgbe[x, 3] = exponent + 128

            f.write(rgbe.tobytes())


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(
        script_dir, '..', 'src', 'agv_isaac_sim', 'textures')
    os.makedirs(output_dir, exist_ok=True)

    output_path = os.path.join(output_dir, 'greenhouse_sky.hdr')

    print("Generating greenhouse HDRI environment map...")
    hdri = generate_greenhouse_hdri(
        width=1024, height=512,
        zenith_intensity=3.0,
        horizon_intensity=0.8,
        ground_intensity=0.3,
        sun_glow_intensity=5.0,
        sun_elevation_deg=60.0,
        sun_azimuth_deg=200.0,
    )

    write_hdr(output_path, hdri)
    print(f"  Output: {output_path}")
    print(f"  Size:   {hdri.shape[1]}x{hdri.shape[0]} (lat-long)")
    print(f"  Range:  [{hdri.min():.2f}, {hdri.max():.2f}]")


if __name__ == '__main__':
    main()
