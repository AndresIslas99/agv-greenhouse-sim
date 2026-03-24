#!/usr/bin/env python3
"""Generate tag36h11 AprilTag textures (IDs 0-5) as 512×512 PNG images.

The tag36h11 family uses a 10×10 grid:
  - Outer 1-cell border: white
  - Inner 1-cell border: black
  - 6×6 data payload: black/white per the tag's code word

Bit patterns sourced from the tag36h11 code table.
"""

import os
import struct
import zlib

# tag36h11 code words for IDs 0-5 (36-bit values, MSB-first in 6×6 grid)
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
    """Convert 36-bit code to 6×6 grid (row-major, MSB = top-left)."""
    grid = []
    for row in range(6):
        row_bits = []
        for col in range(6):
            bit_index = 35 - (row * 6 + col)
            row_bits.append((code >> bit_index) & 1)
        grid.append(row_bits)
    return grid


def build_10x10(data_grid: list) -> list:
    """Build the full 10×10 tag grid with borders.

    Layout:
      Row 0, Row 9: white border
      Col 0, Col 9: white border
      Row 1, Row 8: black border (inner)
      Col 1, Col 8: black border (inner)
      Rows 2-7, Cols 2-7: 6×6 data
    """
    full = [[0] * 10 for _ in range(10)]

    # White outer border
    for i in range(10):
        full[0][i] = 1  # top row
        full[9][i] = 1  # bottom row
        full[i][0] = 1  # left col
        full[i][9] = 1  # right col

    # Black inner border (rows 1,8 and cols 1,8) — already 0

    # 6×6 data payload
    for r in range(6):
        for c in range(6):
            full[r + 2][c + 2] = data_grid[r][c]

    return full


def create_png(pixels: list, width: int, height: int) -> bytes:
    """Create a minimal PNG file from a list of RGB pixel rows."""

    def make_chunk(chunk_type: bytes, data: bytes) -> bytes:
        chunk = chunk_type + data
        return struct.pack('>I', len(data)) + chunk + struct.pack('>I', zlib.crc32(chunk) & 0xFFFFFFFF)

    # PNG signature
    signature = b'\x89PNG\r\n\x1a\n'

    # IHDR
    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    ihdr = make_chunk(b'IHDR', ihdr_data)

    # IDAT — raw pixel data with filter byte 0 per row
    raw = b''
    for row in pixels:
        raw += b'\x00'  # filter: none
        for r, g, b in row:
            raw += struct.pack('BBB', r, g, b)
    compressed = zlib.compress(raw)
    idat = make_chunk(b'IDAT', compressed)

    # IEND
    iend = make_chunk(b'IEND', b'')

    return signature + ihdr + idat + iend


def generate_tag_png(tag_id: int, size: int = 512) -> bytes:
    """Generate a 512×512 PNG for the given tag ID."""
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


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(
        script_dir, '..', 'src', 'agv_sim_worlds', 'materials', 'textures')
    os.makedirs(output_dir, exist_ok=True)

    for tag_id in range(6):
        png_data = generate_tag_png(tag_id)
        filename = f'tag36h11_id{tag_id}.png'
        filepath = os.path.join(output_dir, filename)
        with open(filepath, 'wb') as f:
            f.write(png_data)
        print(f'Generated {filepath}')


if __name__ == '__main__':
    main()
