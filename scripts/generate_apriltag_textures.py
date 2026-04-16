#!/usr/bin/env python3
"""Generate / upscale tag36h11 AprilTag textures to 1024x1024 PNG.

The tag36h11 family uses a 10x10 grid:
  - Outer 1-cell border: white
  - Inner 1-cell border: black
  - 6x6 data payload: black/white per the tag's code word

For IDs 0-5: generates from hardcoded code table (canonical source).
For IDs 6+: upscales existing 512x512 textures from the Gazebo
model directories using nearest-neighbor (preserves sharp B/W edges).

The greenhouse simulation uses IDs 0-37 (36 placed tags).

Run: python3 scripts/generate_apriltag_textures.py
"""

import os
import struct
import zlib

# tag36h11 code words for IDs 0-5 (36-bit values, MSB-first in 6x6 grid).
# These are the canonical bit patterns from the apriltag library.
TAG36H11_CODES = {
    0: 0xD5D628584,
    1: 0xD97F18B49,
    2: 0xDD280910E,
    3: 0xE0D0F96D3,
    4: 0xE479E9C98,
    5: 0xE822DA25D,
}


def code_to_grid(code: int) -> list:
    """Convert 36-bit code to 6x6 grid (row-major, MSB = top-left)."""
    grid = []
    for row in range(6):
        row_bits = []
        for col in range(6):
            bit_index = 35 - (row * 6 + col)
            row_bits.append((code >> bit_index) & 1)
        grid.append(row_bits)
    return grid


def build_10x10(data_grid: list) -> list:
    """Build the full 10x10 tag grid with borders."""
    full = [[0] * 10 for _ in range(10)]

    # White outer border
    for i in range(10):
        full[0][i] = 1
        full[9][i] = 1
        full[i][0] = 1
        full[i][9] = 1

    # 6x6 data payload
    for r in range(6):
        for c in range(6):
            full[r + 2][c + 2] = data_grid[r][c]

    return full


def create_png(pixels: list, width: int, height: int) -> bytes:
    """Create a minimal PNG file from a list of RGB pixel rows."""

    def make_chunk(chunk_type: bytes, data: bytes) -> bytes:
        chunk = chunk_type + data
        return struct.pack('>I', len(data)) + chunk + struct.pack('>I', zlib.crc32(chunk) & 0xFFFFFFFF)

    signature = b'\x89PNG\r\n\x1a\n'
    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    ihdr = make_chunk(b'IHDR', ihdr_data)

    raw = b''
    for row in pixels:
        raw += b'\x00'
        for r, g, b in row:
            raw += struct.pack('BBB', r, g, b)
    compressed = zlib.compress(raw)
    idat = make_chunk(b'IDAT', compressed)
    iend = make_chunk(b'IEND', b'')

    return signature + ihdr + idat + iend


def generate_tag_from_code(tag_id: int, size: int = 1024) -> bytes:
    """Generate a PNG for a tag ID from the code table."""
    code = TAG36H11_CODES[tag_id]
    data_grid = code_to_grid(code)
    full_grid = build_10x10(data_grid)

    cell_size = size // 10
    pixels = []
    for row in range(size):
        pixel_row = []
        for col in range(size):
            grid_r = min(row // cell_size, 9)
            grid_c = min(col // cell_size, 9)
            val = 255 if full_grid[grid_r][grid_c] else 0
            pixel_row.append((val, val, val))
        pixels.append(pixel_row)

    return create_png(pixels, size, size)


def upscale_existing_texture(src_path: str, target_size: int = 1024) -> bytes:
    """Upscale an existing tag texture to target_size using nearest-neighbor.

    Nearest-neighbor preserves the sharp B/W cell boundaries critical
    for AprilTag detection. Bilinear/bicubic would blur the edges.
    """
    from PIL import Image
    img = Image.open(src_path).convert('RGB')
    img_upscaled = img.resize((target_size, target_size), Image.NEAREST)

    # Convert to our PNG format
    pixels = []
    for y in range(target_size):
        row = []
        for x in range(target_size):
            r, g, b = img_upscaled.getpixel((x, y))
            row.append((r, g, b))
        pixels.append(row)

    return create_png(pixels, target_size, target_size)


def main():
    import re
    script_dir = os.path.dirname(os.path.abspath(__file__))
    target_size = 1024

    # Canonical target: agv_isaac_sim/assets/tag_textures/ (consumed by the
    # Isaac USD builder via build_greenhouse_usd.py).
    tag_dir = os.path.join(
        script_dir, '..', 'src', 'agv_isaac_sim', 'assets', 'tag_textures')
    os.makedirs(tag_dir, exist_ok=True)

    # Discover tag IDs from files already present in the assets dir.
    tag_ids = set()
    for entry in os.listdir(tag_dir):
        m = re.match(r'tag36h11_id(\d+)\.png$', entry)
        if m:
            tag_ids.add(int(m.group(1)))
    tag_ids = sorted(tag_ids)

    if not tag_ids:
        # Bootstrap: generate at least IDs 0-5 from code table
        tag_ids = sorted(TAG36H11_CODES.keys())

    print(f"Regenerating {len(tag_ids)} AprilTag textures at {target_size}x{target_size}...")

    generated = 0
    upscaled = 0
    for tag_id in tag_ids:
        filename = f'tag36h11_id{tag_id}.png'
        out_path = os.path.join(tag_dir, filename)

        if tag_id in TAG36H11_CODES:
            png_data = generate_tag_from_code(tag_id, size=target_size)
            with open(out_path, 'wb') as f:
                f.write(png_data)
            generated += 1
            method = "generated"
        else:
            # Upscale from existing smaller texture in the same dir
            if os.path.exists(out_path):
                png_data = upscale_existing_texture(out_path, target_size=target_size)
                with open(out_path, 'wb') as f:
                    f.write(png_data)
                upscaled += 1
                method = "upscaled"
            else:
                print(f'  Tag {tag_id:2d}: SKIPPED (no source texture)')
                continue

        print(f'  Tag {tag_id:2d}: {filename} ({method})')

    print(f'\n{generated + upscaled} tags total '
          f'({generated} from code table, {upscaled} upscaled)')


if __name__ == '__main__':
    main()
