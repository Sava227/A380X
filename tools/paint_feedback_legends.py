#!/usr/bin/env python3
"""Paint the cockpit-feedback lamp legends (OVRD blue, FIRE red) into the installed FCU
label atlases, cockpit_3d/fcu_labels.png and fcu_labels_LIT.png, on cells no drawn geometry samples.

The cells and texts are tools/cockpit_feedback_contract.py EXTRA_LEGEND_CELLS; tools/build_fcu_panel.py
draws the same cells when it regenerates the atlas. Only those cells change (checked); re-running is a
no-op. Needs Pillow. Run the DDS converter afterwards when the aircraft uses DDS textures.

    python3 tools/paint_feedback_legends.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" [--dry-run]
"""
from __future__ import annotations

import argparse
from pathlib import Path

from build_cockpit_controls import FCU_LABEL_COLUMNS, FCU_LABEL_ROWS
from build_fcu_panel import feedback_legend_cell
from cockpit_feedback_contract import EXTRA_LEGEND_CELLS
from ovhd_cockpit_lamps import ATLASES, atlas_usage, cell_box

CONTROLS_OBJ = "objects/interior/cockpit-controls/a380x_cockpit_controls.obj"
OUR_MARKER = "# A380X FEEDBACK V1"


def paint(path: Path, controls_text: str):
    """(image, changed cells); raises if a cell to paint is sampled by geometry other than our lamps."""
    from PIL import Image
    image = Image.open(path).convert("RGBA")
    original = image.copy()
    used = atlas_usage(controls_text.split("\n" + OUR_MARKER)[0], exclude_marker="#@@none@@")
    changed = []
    for key, (_text, _rgba, cell) in sorted(EXTRA_LEGEND_CELLS.items(), key=lambda kv: kv[1][2]):
        box = cell_box(cell, *image.size)
        fresh = feedback_legend_cell(key, (box[2] - box[0], box[3] - box[1]))
        if image.crop(box).tobytes() == fresh.tobytes():
            continue
        if cell in used:
            raise ValueError(f"{path.name}: atlas cell {cell} is sampled by other geometry")
        image.paste(fresh, box[:2])
        changed.append(cell)
    for cell in range(FCU_LABEL_COLUMNS * FCU_LABEL_ROWS):
        box = cell_box(cell, *image.size)
        if cell not in changed and image.crop(box).tobytes() != original.crop(box).tobytes():
            raise ValueError(f"{path.name}: touched foreign cell {cell}")
    return image, changed


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    controls = (args.aircraft / CONTROLS_OBJ).read_bytes().decode("latin-1")
    for relative in ATLASES:
        path = args.aircraft / relative
        image, changed = paint(path, controls)
        if changed and not args.dry_run:
            image.save(path)
        print(f"{relative}: {'would paint' if args.dry_run else 'painted'} cells {changed}" if changed
              else f"{relative}: up to date")


if __name__ == "__main__":
    main()
