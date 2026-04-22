#!/usr/bin/env python3
"""Generate a print-ready US Letter PDF of all greenhouse AprilTags.

The textures under assets/tag_textures/ are 1024 × 1024 px and contain
the tag36h11 pattern at 80 % of the image (8×8 detectable cells) plus
a 10 % white margin on each side. apriltag_ros configures `tag_size`
as the BLACK SQUARE outer edge, so we crop the white margin before
drawing — the printed 200 mm IS the detectable black square.

Each page:
  - 200 mm × 200 mm BLACK detectable face (crop = no scaling artifacts)
  - Cut marks at the black corners (so cutting on the marks gives a
    flush-to-black tag; add a manual white border if mounting on plate)
  - Header with mount type + assignment
  - 200 mm scale ruler to verify printer scale

Usage:
  python3 scripts/generate_apriltag_print_pdf.py [--per-tag] [--ids 1,2,3]
"""
import os
import sys
import argparse
import io

from PIL import Image
import numpy as np
from reportlab.lib.pagesizes import LETTER
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX_DIR = os.path.join(REPO, 'src/agv_isaac_sim/assets/tag_textures')
OUT_DIR = os.path.join(REPO, 'docs/printables')

# Mirror of APRILTAG_PLACEMENTS in build_greenhouse_usd.py.
# Each row: (id, x, y, z, rx, ry, rz, mount, assignment, descr)
#   mount      : 'PARED' (vertical wall-mounted) | 'SUELO' (face-up floor)
#   assignment : Pasillo N / Arbusto N / Sección — what the operator
#                should write on the printed sheet to know where it goes
#   descr      : longer English-style description for the small print
PLACEMENTS = [
    # ── Walls — esquinas / centros (sin pasillo) ──────────────────
    (0,  -16.88, 0.0,  0.145, 0, 0,    0, 'PARED', 'Pared OESTE — centro',  'West wall, facing +X'),
    (1,   27.88, 0.0,  0.145, 0, 0,  180, 'PARED', 'Pared ESTE — centro',   'East wall, facing −X'),
    (5,  -16.88, -5.0, 0.145, 0, 0,    0, 'PARED', 'Pared OESTE — sur',     'West wall south, facing +X'),
    (14,  15.0,  7.38, 0.145, 0, 0,  -90, 'PARED', 'Pared NORTE',           'North wall, facing −Y'),
    (15,  15.0, -7.38, 0.145, 0, 0,   90, 'PARED', 'Pared SUR',             'South wall, facing +Y'),

    # ── Wall — Pared ESTE, una por pasillo (entrada al pasillo desde el este) ──
    (16,  27.88, -4.4, 0.145, 0, 0,  180, 'PARED', 'Pared ESTE — Pasillo 1', 'East wall aisle 1, facing −X'),
    (17,  27.88, -2.2, 0.145, 0, 0,  180, 'PARED', 'Pared ESTE — Pasillo 2', 'East wall aisle 2, facing −X'),
    (18,  27.88, 0.0,  0.145, 0, 0,  180, 'PARED', 'Pared ESTE — Pasillo 3', 'East wall aisle 3, facing −X'),
    (19,  27.88, 2.2,  0.145, 0, 0,  180, 'PARED', 'Pared ESTE — Pasillo 4', 'East wall aisle 4, facing −X'),
    (20,  27.88, 4.4,  0.145, 0, 0,  180, 'PARED', 'Pared ESTE — Pasillo 5', 'East wall aisle 5, facing −X'),

    # ── Wall — Pared OESTE, una por pasillo (entrada desde el oeste) ──
    (21, -16.88, -4.4, 0.145, 0, 0,    0, 'PARED', 'Pared OESTE — Pasillo 1', 'West wall aisle 1, facing +X'),
    (22, -16.88, -2.2, 0.145, 0, 0,    0, 'PARED', 'Pared OESTE — Pasillo 2', 'West wall aisle 2, facing +X'),
    (23, -16.88, 0.0,  0.145, 0, 0,    0, 'PARED', 'Pared OESTE — Pasillo 3', 'West wall aisle 3, facing +X'),
    (24, -16.88, 2.2,  0.145, 0, 0,    0, 'PARED', 'Pared OESTE — Pasillo 4', 'West wall aisle 4, facing +X'),
    (25, -16.88, 4.4,  0.145, 0, 0,    0, 'PARED', 'Pared OESTE — Pasillo 5', 'West wall aisle 5, facing +X'),

    # ── Floor — entrada DELANTERA (front), pegado al inicio del pasillo ──
    (2,   7.33, -4.4,  0.002, 0, -90,  0, 'SUELO', 'Suelo entrada delantera — Pasillo 1', 'Front floor entry aisle 1'),
    (3,   7.33, -2.2,  0.002, 0, -90,  0, 'SUELO', 'Suelo entrada delantera — Pasillo 2', 'Front floor entry aisle 2'),
    (4,   7.33, 0.0,   0.002, 0, -90,  0, 'SUELO', 'Suelo entrada delantera — Pasillo 3', 'Front floor entry aisle 3'),
    (12,  7.33, 2.2,   0.002, 0, -90,  0, 'SUELO', 'Suelo entrada delantera — Pasillo 4', 'Front floor entry aisle 4'),
    (13,  7.33, 4.4,   0.002, 0, -90,  0, 'SUELO', 'Suelo entrada delantera — Pasillo 5', 'Front floor entry aisle 5'),

    # ── Floor — entrada TRASERA (rear), del otro lado del gap ──
    (33,  3.67, -4.4,  0.002, 0, -90,  0, 'SUELO', 'Suelo entrada trasera — Pasillo 1', 'Rear floor entry aisle 1'),
    (34,  3.67, -2.2,  0.002, 0, -90,  0, 'SUELO', 'Suelo entrada trasera — Pasillo 2', 'Rear floor entry aisle 2'),
    (35,  3.67, 0.0,   0.002, 0, -90,  0, 'SUELO', 'Suelo entrada trasera — Pasillo 3', 'Rear floor entry aisle 3'),
    (36,  3.67, 2.2,   0.002, 0, -90,  0, 'SUELO', 'Suelo entrada trasera — Pasillo 4', 'Rear floor entry aisle 4'),
    (37,  3.67, 4.4,   0.002, 0, -90,  0, 'SUELO', 'Suelo entrada trasera — Pasillo 5', 'Rear floor entry aisle 5'),

    # ── Wall — frente de cada arbusto, sección DELANTERA ──
    (6,    7.4, -5.5,  0.145, 0, 0,  180, 'PARED', 'Frente delantero — Arbusto 1', 'Front row 1 start, facing −X'),
    (7,    7.4, -3.3,  0.145, 0, 0,  180, 'PARED', 'Frente delantero — Arbusto 2', 'Front row 2 start, facing −X'),
    (8,    7.4, -1.1,  0.145, 0, 0,  180, 'PARED', 'Frente delantero — Arbusto 3', 'Front row 3 start, facing −X'),
    (9,    7.4, 1.1,   0.145, 0, 0,  180, 'PARED', 'Frente delantero — Arbusto 4', 'Front row 4 start, facing −X'),
    (10,   7.4, 3.3,   0.145, 0, 0,  180, 'PARED', 'Frente delantero — Arbusto 5', 'Front row 5 start, facing −X'),
    (11,   7.4, 5.5,   0.145, 0, 0,  180, 'PARED', 'Frente delantero — Arbusto 6', 'Front row 6 start, facing −X'),

    # ── Wall — frente de cada arbusto, sección TRASERA ──
    (26,   3.6, -5.5,  0.145, 0, 0,    0, 'PARED', 'Frente trasero — Arbusto 1', 'Rear row 1 start, facing +X'),
    (27,   3.6, -3.3,  0.145, 0, 0,    0, 'PARED', 'Frente trasero — Arbusto 2', 'Rear row 2 start, facing +X'),
    (28,   3.6, -1.1,  0.145, 0, 0,    0, 'PARED', 'Frente trasero — Arbusto 3', 'Rear row 3 start, facing +X'),
    (29,   3.6, 1.1,   0.145, 0, 0,    0, 'PARED', 'Frente trasero — Arbusto 4', 'Rear row 4 start, facing +X'),
    (30,   3.6, 3.3,   0.145, 0, 0,    0, 'PARED', 'Frente trasero — Arbusto 5', 'Rear row 5 start, facing +X'),
    (31,   3.6, 5.5,   0.145, 0, 0,    0, 'PARED', 'Frente trasero — Arbusto 6', 'Rear row 6 start, facing +X'),
]

TAG_FACE_MM = 200.0  # matches simulator's 0.20 m face
PAGE_W, PAGE_H = LETTER  # in points (1/72 in). reportlab handles units.


def _load_cropped_tag(img_path: str) -> ImageReader:
    """Load the PNG and crop to its black bounding box. Returns a
    reportlab ImageReader of the cropped tag (so the printed 200 mm
    is exactly the BLACK SQUARE outer edge — what apriltag_ros's
    `tag_size` parameter expects)."""
    img = Image.open(img_path).convert('RGB')
    arr = np.array(img.convert('L'))
    h, w = arr.shape
    row_has_black = (arr < 128).any(axis=1)
    col_has_black = (arr < 128).any(axis=0)
    top    = int(np.argmax(row_has_black))
    bottom = int(h - 1 - np.argmax(row_has_black[::-1]))
    left   = int(np.argmax(col_has_black))
    right  = int(w - 1 - np.argmax(col_has_black[::-1]))
    cropped = img.crop((left, top, right + 1, bottom + 1))
    buf = io.BytesIO()
    cropped.save(buf, format='PNG')
    buf.seek(0)
    return ImageReader(buf)


def _facing_string(rx: float, ry: float, rz: float) -> str:
    """Human-readable face direction. Floor tags have ry=-90° (face up).
    Wall tags use rz to point along ±X / ±Y."""
    if abs(ry) > 45:                           # floor (face up)
        return 'arriba (+Z, hacia el techo)'
    if rz == 0:    return 'este (+X)'
    if rz == 180:  return 'oeste (−X)'
    if rz == 90:   return 'sur (+Y)'
    if rz == -90:  return 'norte (−Y)'
    return f'rotación libre Rz={rz}°'


def draw_tag_page(c: canvas.Canvas, tag_id: int, x: float, y: float, z: float,
                  rx: float, ry: float, rz: float,
                  mount: str, assignment: str, descr: str):
    """Render one US-Letter page with tag image + label + scale ruler."""
    img_path = os.path.join(TEX_DIR, f'tag36h11_id{tag_id}.png')
    if not os.path.exists(img_path):
        c.drawCentredString(PAGE_W / 2, PAGE_H / 2,
                            f'MISSING TEXTURE: {img_path}')
        c.showPage()
        return

    # Layout: tag centred horizontally, ~32 mm from top to leave space
    # for the prominent header (mount type + assignment) above it.
    tag_w = TAG_FACE_MM * mm
    tag_h = TAG_FACE_MM * mm
    tag_x = (PAGE_W - tag_w) / 2.0
    tag_y = PAGE_H - 32 * mm - tag_h

    # Tag image — CROPPED to the black bounding box first, so the
    # 200 mm dimension below IS the detectable black edge. The PNG
    # ships with a 10% white border that would otherwise shrink the
    # printed black to ~160 mm.
    img = _load_cropped_tag(img_path)
    c.drawImage(img, tag_x, tag_y, width=tag_w, height=tag_h,
                preserveAspectRatio=True, mask='auto')

    # Cut-line marks at the four corners of the 200 mm face.
    c.setLineWidth(0.4)
    c.setDash(2, 2)
    mark_len = 6 * mm
    for cx, cy in [(tag_x, tag_y), (tag_x + tag_w, tag_y),
                   (tag_x, tag_y + tag_h), (tag_x + tag_w, tag_y + tag_h)]:
        c.line(cx - mark_len, cy, cx + mark_len, cy)
        c.line(cx, cy - mark_len, cx, cy + mark_len)
    c.setDash()  # reset

    # ── Top header — the OPERATOR's at-a-glance info ──
    # Line 1: mount type (PARED / SUELO) + tag ID
    c.setFont('Helvetica-Bold', 20)
    c.drawCentredString(PAGE_W / 2, PAGE_H - 13 * mm,
                        f'{mount}    ·    ID {tag_id}')
    # Line 2: assignment (where it goes)
    c.setFont('Helvetica-Bold', 14)
    c.drawCentredString(PAGE_W / 2, PAGE_H - 21 * mm, assignment)
    # Line 3 (small): family
    c.setFont('Helvetica', 8)
    c.drawCentredString(PAGE_W / 2, PAGE_H - 26 * mm, 'tag36h11 family')

    # ── Label block (below the tag) ──
    label_y = tag_y - 11 * mm
    c.setFont('Helvetica-Bold', 11)
    c.drawCentredString(PAGE_W / 2, label_y, descr)

    label_y -= 7 * mm
    c.setFont('Helvetica', 10)
    c.drawCentredString(PAGE_W / 2, label_y,
        f'Pose mundo:  x={x:+.2f} m   y={y:+.2f} m   z={z:.3f} m')
    label_y -= 5 * mm
    # Show orientation in plain language plus raw degrees
    facing = _facing_string(rx, ry, rz)
    c.drawCentredString(PAGE_W / 2, label_y,
        f'Cara apunta a:  {facing}   (Rx={rx}°  Ry={ry}°  Rz={rz}°)')
    label_y -= 5 * mm
    c.drawCentredString(PAGE_W / 2, label_y,
        f'Negro detectable: {TAG_FACE_MM:.0f} × {TAG_FACE_MM:.0f} mm   '
        f'(cortar afuera de las marcas; agregar borde blanco si se monta en plato)')

    # Scale ruler (below the label block) — 200 mm long with 50 mm ticks
    ruler_y = label_y - 12 * mm
    ruler_x0 = (PAGE_W - 200 * mm) / 2.0
    ruler_x1 = ruler_x0 + 200 * mm
    c.setLineWidth(0.7)
    c.line(ruler_x0, ruler_y, ruler_x1, ruler_y)
    c.setFont('Helvetica', 8)
    for mm_pos in (0, 50, 100, 150, 200):
        x_tick = ruler_x0 + mm_pos * mm
        c.line(x_tick, ruler_y, x_tick, ruler_y + 3 * mm)
        c.drawCentredString(x_tick, ruler_y - 4 * mm, f'{mm_pos} mm')
    c.drawCentredString(PAGE_W / 2, ruler_y - 9 * mm,
        'Imprimir al 100 %. La barra debe medir 200 mm — y el cuadro NEGRO también.')

    # Bottom-left footprint
    c.setFont('Helvetica-Oblique', 7)
    c.drawString(8 * mm, 6 * mm,
        f'AGV Opalina — AprilTag tag36h11 — face 0.20 m / borde sugerido 0.25 m. '
        f'Mount: {mount}.')

    c.showPage()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--per-tag', action='store_true',
                    help='Output one PDF per tag id instead of a combined book.')
    ap.add_argument('--ids', type=str, default=None,
                    help='Comma-separated subset (e.g. 0,4,18). Default: all.')
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)

    if args.ids:
        wanted = set(int(s) for s in args.ids.split(','))
        rows = [r for r in PLACEMENTS if r[0] in wanted]
    else:
        rows = list(PLACEMENTS)
    rows.sort(key=lambda r: r[0])  # ascending by id for usability

    if args.per_tag:
        produced = []
        for row in rows:
            tag_id = row[0]
            out = os.path.join(OUT_DIR, f'apriltag_id{tag_id:02d}.pdf')
            c = canvas.Canvas(out, pagesize=LETTER)
            draw_tag_page(c, *row)
            c.save()
            produced.append(out)
        print(f'Generated {len(produced)} per-tag PDFs in {OUT_DIR}/')
        for p in produced:
            print(f'  {p}')
    else:
        out = os.path.join(OUT_DIR, 'apriltags_greenhouse_book.pdf')
        c = canvas.Canvas(out, pagesize=LETTER)
        for row in rows:
            draw_tag_page(c, *row)
        c.save()
        print(f'Generated combined book ({len(rows)} pages):')
        print(f'  {out}')


if __name__ == '__main__':
    main()
