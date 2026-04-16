#!/usr/bin/env python3
"""Generate procedural textures for the greenhouse simulation.

Creates albedo PNG files and companion normal maps that give surfaces
visual detail for stereo depth camera matching (ZED 2i) and RTX
rendering with OmniPBR materials.

Run: python3 scripts/generate_textures.py
"""

import os
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from scipy import ndimage

OUTPUT_DIR = os.path.join(os.path.dirname(__file__),
                          '..', 'src', 'agv_isaac_sim', 'assets', 'greenhouse_textures')


def noise_layer(w, h, scale=1):
    """Generate a layer of random noise at given scale."""
    small = np.random.rand(max(1, h // scale), max(1, w // scale))
    img = Image.fromarray((small * 255).astype(np.uint8), mode='L')
    return np.array(img.resize((w, h), Image.BILINEAR)) / 255.0


def fractal_noise(w, h, octaves=4):
    """Multi-octave fractal noise for natural-looking textures."""
    result = np.zeros((h, w))
    amplitude = 1.0
    for i in range(octaves):
        scale = 2 ** i
        result += noise_layer(w, h, max(1, 64 // scale)) * amplitude
        amplitude *= 0.5
    result /= result.max()
    return result


def ground_soil(size=1024):
    """Brown soil with gravel variation."""
    noise = fractal_noise(size, size, octaves=5)

    r = (0.28 + noise * 0.15) * 255
    g = (0.20 + noise * 0.10) * 255
    b = (0.12 + noise * 0.08) * 255

    # Add darker spots (gravel/stones)
    spots = noise_layer(size, size, 4)
    dark = spots < 0.3
    r[dark] *= 0.6
    g[dark] *= 0.6
    b[dark] *= 0.6

    img = np.stack([r, g, b], axis=-1).clip(0, 255).astype(np.uint8)
    return Image.fromarray(img)


def wall_polycarbonate(size=512):
    """Translucent white polycarbonate panels with seam lines."""
    noise = fractal_noise(size, size, octaves=3)

    base = 0.85 + noise * 0.10
    r = base * 255
    g = base * 255
    b = (base - 0.02) * 255

    img = np.stack([r, g, b], axis=-1).clip(0, 255).astype(np.uint8)
    pil_img = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_img)

    # Horizontal panel seams every ~panel_h pixels
    panel_h = size // 4
    for y in range(0, size, panel_h):
        draw.line([(0, y), (size, y)], fill=(180, 180, 175), width=2)

    # Vertical seams
    panel_w = size // 3
    for x in range(0, size, panel_w):
        draw.line([(x, 0), (x, size)], fill=(180, 180, 175), width=2)

    # Slight blur for realism
    return pil_img.filter(ImageFilter.GaussianBlur(radius=0.5))


def crop_leaves(size=512, alt=False):
    """Dense green leaves pattern."""
    noise = fractal_noise(size, size, octaves=5)
    detail = noise_layer(size, size, 2)

    if alt:
        base_g = 0.30
        var_g = 0.18
    else:
        base_g = 0.35
        var_g = 0.20

    r = (0.05 + noise * 0.08 + detail * 0.04) * 255
    g = (base_g + noise * var_g) * 255
    b = (0.03 + noise * 0.05) * 255

    # Simulate leaf shadows (darker patches)
    shadow = noise_layer(size, size, 8)
    dark = shadow < 0.25
    r[dark] *= 0.5
    g[dark] *= 0.6
    b[dark] *= 0.5

    # Bright spots (light through canopy)
    bright = shadow > 0.85
    g[bright] = np.minimum(g[bright] * 1.4, 255)

    img = np.stack([r, g, b], axis=-1).clip(0, 255).astype(np.uint8)
    return Image.fromarray(img)


def crate_wood(size=256):
    """Wood plank grain texture."""
    # Horizontal grain lines
    y_coords = np.tile(np.arange(size).reshape(-1, 1), (1, size))
    grain = np.sin(y_coords * 0.3 + noise_layer(size, size, 16) * 8) * 0.5 + 0.5
    noise = fractal_noise(size, size, octaves=3)

    r = (0.45 + grain * 0.12 + noise * 0.08) * 255
    g = (0.30 + grain * 0.08 + noise * 0.05) * 255
    b = (0.15 + grain * 0.04 + noise * 0.03) * 255

    # Plank divisions
    img = np.stack([r, g, b], axis=-1).clip(0, 255).astype(np.uint8)
    pil_img = Image.fromarray(img)
    draw = ImageDraw.Draw(pil_img)
    plank_h = size // 5
    for y in range(0, size, plank_h):
        draw.line([(0, y), (size, y)], fill=(80, 55, 30), width=2)

    return pil_img


def rail_steel(w=256, h=128):
    """Brushed galvanized steel texture."""
    # Horizontal brush marks
    x_coords = np.tile(np.arange(w).reshape(1, -1), (h, 1))
    brush = np.sin(x_coords * 0.8 + noise_layer(w, h, 32) * 4) * 0.5 + 0.5
    noise = fractal_noise(w, h, octaves=2)

    base = 0.60 + brush * 0.08 + noise * 0.06
    r = base * 255
    g = base * 255
    b = (base + 0.02) * 255

    # Oxidation spots
    spots = noise_layer(w, h, 4)
    oxide = spots < 0.15
    r[oxide] = np.minimum(r[oxide] * 1.15, 255)
    g[oxide] *= 0.85
    b[oxide] *= 0.75

    img = np.stack([r, g, b], axis=-1).clip(0, 255).astype(np.uint8)
    return Image.fromarray(img)


def generate_normal_map(albedo_img, strength=2.0):
    """Derive a tangent-space normal map from an albedo image using Sobel filter.

    Convention: R=+X, G=+Y, B=+Z (OpenGL tangent-space).
    Neutral normal (flat surface) = (128, 128, 255).
    """
    grey = np.array(albedo_img.convert('L'), dtype=np.float32) / 255.0
    dx = ndimage.sobel(grey, axis=1) * strength
    dy = ndimage.sobel(grey, axis=0) * strength

    # Normal vector: (-dx, -dy, 1) then normalize
    dz = np.ones_like(dx)
    length = np.sqrt(dx * dx + dy * dy + dz * dz)
    nx = -dx / length
    ny = -dy / length
    nz = dz / length

    # Map [-1,1] to [0,255]
    r = ((nx + 1.0) * 0.5 * 255).clip(0, 255).astype(np.uint8)
    g = ((ny + 1.0) * 0.5 * 255).clip(0, 255).astype(np.uint8)
    b = ((nz + 1.0) * 0.5 * 255).clip(0, 255).astype(np.uint8)

    return Image.fromarray(np.stack([r, g, b], axis=-1))


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Higher resolution textures for OmniPBR + RTX rendering
    textures = {
        'ground_soil.png': ground_soil(2048),
        'wall_polycarbonate.png': wall_polycarbonate(1024),
        'crop_leaves.png': crop_leaves(1024, alt=False),
        'crop_leaves_alt.png': crop_leaves(1024, alt=True),
        'crate_wood.png': crate_wood(512),
        'rail_steel.png': rail_steel(512, 256),
    }

    # Textures that benefit from normal maps (surfaces with visible depth)
    normal_map_targets = [
        'ground_soil.png',
        'crop_leaves.png',
        'crop_leaves_alt.png',
        'crate_wood.png',
        'rail_steel.png',
    ]

    saved = 0
    for name, img in textures.items():
        path = os.path.join(OUTPUT_DIR, name)
        img.save(path)
        print(f'  {name}: {img.size[0]}x{img.size[1]}')
        saved += 1

        if name in normal_map_targets:
            base, ext = os.path.splitext(name)
            normal_name = f'{base}_normal{ext}'
            strength = 3.0 if 'soil' in name else 2.0
            normal_img = generate_normal_map(img, strength=strength)
            normal_path = os.path.join(OUTPUT_DIR, normal_name)
            normal_img.save(normal_path)
            print(f'  {normal_name}: {normal_img.size[0]}x{normal_img.size[1]} (normal map)')
            saved += 1

    print(f'\n{saved} textures saved to {OUTPUT_DIR}')


if __name__ == '__main__':
    main()
