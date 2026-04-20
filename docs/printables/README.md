# Greenhouse AprilTag printables

Print-ready PDFs of the 37 AprilTags (`tag36h11` family) used in the
Opalina greenhouse, sized to match the simulator spec.

## Files

| File | What it is |
|---|---|
| `apriltags_greenhouse_book.pdf` | All 37 tags in one PDF (one per page). Print this for the full set. |
| `apriltags_greenhouse_index.pdf` | 1–2 page reference index grouped by mount type (PARED / SUELO) and assignment (Pasillo N / Arbusto N). Hand to whoever is mounting the tags. |
| `apriltag_id00.pdf` … `apriltag_id37.pdf` | One PDF per tag id, for partial reprints. IDs 0-31 + 33-37 (no id 32). |

## What's on each page

```
PARED  ·  ID 23                       ← mount type (PARED / SUELO) + ID
Pared OESTE — Pasillo 3               ← where this tag goes
tag36h11 family

[ 200 × 200 mm AprilTag image ]
[ cut-marks at the 4 black corners ]

West wall aisle 3, facing +X
Pose mundo:  x=−16.88 m   y=+0.00 m   z=0.145 m
Cara apunta a:  este (+X)   (Rx=0°  Ry=0°  Rz=0°)
Negro detectable: 200 × 200 mm

[ 200 mm scale ruler ]
"Imprimir al 100 %. La barra debe medir 200 mm — y el cuadro NEGRO también."
```

## Print settings (critical)

- Paper: **US Letter** (8.5 × 11 in)
- Scale: **100 %** (NOT "Fit to page", NOT "Shrink to fit")
- Orientation: portrait
- Margins: default

Verify with a ruler after printing:
1. **Black square**: edge-to-edge = 200 mm = 20 cm  ← what `apriltag_ros` uses as `tag_size`
2. **Bottom bar**: 200 mm
3. **Cut marks** at the 4 corners coincide with the black corners

If any of those differs from 200 mm, your printer is scaling — fix the print dialog and reprint.

## Tag groups

| Group | Mount | Tags | Where |
|---|---|---|---|
| Suelo entrada delantera | Floor (face up) | 2, 3, 4, 12, 13 | x=7.0 m, one per aisle |
| Suelo entrada trasera | Floor (face up) | 33, 34, 35, 36, 37 | x=4.0 m, one per aisle |
| Frente delantero arbustos | Wall | 6, 7, 8, 9, 10, 11 | x=7.4 m, one per row, facing −X |
| Frente trasero arbustos | Wall | 26, 27, 28, 29, 30, 31 | x=3.6 m, one per row, facing +X |
| Pared OESTE pasillos | Wall | 21, 22, 23, 24, 25 | x=−16.88 m, one per aisle, facing +X |
| Pared ESTE pasillos | Wall | 16, 17, 18, 19, 20 | x=27.88 m, one per aisle, facing −X |
| Pared esquinas / centros | Wall | 0, 1, 5, 14, 15 | corners of the greenhouse |

## Re-generating

```bash
cd /home/andres/agv-sim
python3 scripts/generate_apriltag_print_pdf.py             # combined book
python3 scripts/generate_apriltag_print_pdf.py --per-tag   # one PDF per id
python3 scripts/generate_apriltag_print_pdf.py --ids 4,18  # subset
```

The placement table inside the script mirrors `APRILTAG_PLACEMENTS` in
`src/agv_isaac_sim/scripts/build_greenhouse_usd.py`. If you change tag
positions there, also update the script's `PLACEMENTS` list and re-run.
