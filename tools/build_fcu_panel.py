#!/usr/bin/env python3
"""Build the A380 panel texture, native displays, flyPad, and FCU atlas.

The generated files are consumed by the special X-Plane cockpit object.  They
do not replace the optional FlyWithLua windows.  The aircraft-local native
plugin draws the FBW-derived captain/first-officer PFD and ND pairs and EWD into the
opaque display beds; no stock X-Plane EFIS instrument is placed in those beds.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

from PIL import Image, ImageDraw, ImageFont

from avionics_panel_layout import (
    EFB_PANEL_RECT,
    EFIS_BARO_PANEL_RECTS,
    EFIS_UPPER_PANEL_RECTS,
    EFIS_LOWER_PANEL_RECTS,
    EWD_PANEL_RECT,
    FCU_PANEL_RECT,
    FCU_SCREEN_HEIGHT,
    ND_PANEL_RECTS,
    PANEL_SIZE,
    PFD_PANEL_RECTS,
)
from build_cockpit_controls import (FCU_LABEL_COLUMNS, FCU_LABEL_ROWS, FCU_LABELS,
                                    FCU_ANNUNCIATORS, FCU_ANNUNCIATOR_FIRST_CELL,
                                    GLARE_ANNUNCIATORS, GLARE_ANNUNCIATOR_FIRST_CELL,
                                    ADIRS_ANNUNCIATORS, ADIRS_ANNUNCIATOR_FIRST_CELL,
                                    ELEC_LEGEND_CELLS)


LABEL_COLOR = (220, 226, 235, 255)
# FCU daylight-label cells now holding overhead lamp legends (no geometry samples the labels).
from ovhd_lamps_contract import NEW_LEGEND_CELLS as _OVHD_LEGEND_CELLS  # noqa: E402
OVHD_CELLS = frozenset(_OVHD_LEGEND_CELLS.values())


def font(size: int) -> ImageFont.ImageFont:
    """Use Pillow's bundled font so builds do not depend on a workstation font."""
    return ImageFont.load_default(size=size)


def centered_text(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    value: str,
    fill: tuple[int, int, int, int],
    size: int,
) -> None:
    face = font(size)
    left, top, right, bottom = box
    bounds = draw.textbbox((0, 0), value, font=face, stroke_width=1)
    width, height = bounds[2] - bounds[0], bounds[3] - bounds[1]
    draw.text(
        (left + (right - left - width) / 2, top + (bottom - top - height) / 2 - bounds[1]),
        value,
        font=face,
        fill=fill,
        stroke_width=1,
        stroke_fill=(20, 25, 34, 220),
    )


def gear_arrow(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int],
               color: tuple[int, int, int, int]) -> None:
    left, top, right, bottom = box
    middle = (left + right) / 2
    width = min(right - left, bottom - top) * 0.38
    points = [
        (middle - width * .27, top + 5), (middle + width * .27, top + 5),
        (middle + width * .27, bottom - width * .75),
        (middle + width * .63, bottom - width * .75),
        (middle, bottom - 5),
        (middle - width * .63, bottom - width * .75),
        (middle - width * .27, bottom - width * .75),
    ]
    draw.polygon(points, fill=color)


def gear_triangle(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int],
                  color: tuple[int, int, int, int]) -> None:
    left, top, right, bottom = box
    middle=(left+right)/2
    points=[(middle-28,top+13),(middle+28,top+13),(middle,bottom-9)]
    draw.line([*points,points[0]],fill=color,width=5,joint='curve')


def elec_legends(draw, cell_width: int, cell_height: int) -> None:
    """ELEC overhead ALTN / DISC / AVAIL / ON cells (FAULT and OFF reuse the ADIRS cells)."""
    from elec_panel_contract import LEGEND_COLORS, LEGEND_TEXT
    if max(ELEC_LEGEND_CELLS.values()) >= FCU_LABEL_COLUMNS * FCU_LABEL_ROWS:
        raise ValueError('Annunciator atlas capacity exceeded')
    for legend, index in ELEC_LEGEND_CELLS.items():
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box = (column * cell_width + 2, row * cell_height + 2,
               (column + 1) * cell_width - 2, (row + 1) * cell_height - 2)
        centered_text(draw, box, LEGEND_TEXT[legend], LEGEND_COLORS[legend], 34)


def ovhd_legend_cell(legend: str, size: tuple[int, int]) -> Image.Image:
    """One overhead lamp legend cell (tools/ovhd_lamps_contract.py LEGENDS), ELEC style."""
    from ovhd_lamps_contract import LEGENDS
    text, rgba, _cell = LEGENDS[legend]
    image = Image.new('RGBA', size, (0, 0, 0, 0))
    centered_text(ImageDraw.Draw(image), (2, 2, size[0] - 2, size[1] - 2), text, rgba, 34)
    return image


def ovhd_legends(atlas: Image.Image, cell_width: int, cell_height: int) -> None:
    """Overhead lamp legend cells on the unused FCU daylight-label cells (tools/ovhd_cockpit_lamps.py)."""
    from ovhd_lamps_contract import NEW_LEGEND_CELLS
    for legend, index in NEW_LEGEND_CELLS.items():
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        atlas.paste(ovhd_legend_cell(legend, (cell_width, cell_height)), (column * cell_width, row * cell_height))


def feedback_legend_cell(key: str, size: tuple[int, int]) -> Image.Image:
    """Cockpit-feedback lamp legend cell (tools/cockpit_feedback_contract.py EXTRA_LEGEND_CELLS), ELEC style."""
    from cockpit_feedback_contract import EXTRA_LEGEND_CELLS
    text, rgba, _cell = EXTRA_LEGEND_CELLS[key]
    image = Image.new('RGBA', size, (0, 0, 0, 0))
    centered_text(ImageDraw.Draw(image), (2, 2, size[0] - 2, size[1] - 2), text, rgba, 34)
    return image


def feedback_legends(atlas: Image.Image, cell_width: int, cell_height: int) -> None:
    """Cockpit-feedback lamp legends on unused atlas cells (tools/paint_feedback_legends.py)."""
    from cockpit_feedback_contract import EXTRA_LEGEND_CELLS
    for key, (_text, _rgba, index) in EXTRA_LEGEND_CELLS.items():
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        atlas.paste(feedback_legend_cell(key, (cell_width, cell_height)), (column * cell_width, row * cell_height))


def build_label_atlas(output: Path) -> None:
    size = (1024, 512)
    atlas = Image.new('RGBA', size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(atlas)
    cell_width = size[0] // FCU_LABEL_COLUMNS
    cell_height = size[1] // FCU_LABEL_ROWS
    for index, (label, *_placement) in enumerate(FCU_LABELS):
        if index in OVHD_CELLS:
            continue
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box = (
            column * cell_width + 3,
            row * cell_height + 3,
            (column + 1) * cell_width - 3,
            (row + 1) * cell_height - 3,
        )
        text_size = 38 if len(label) <= 5 else 28
        centered_text(draw, box, label, LABEL_COLOR, text_size)
    for index, (_label, _node, _dataref) in enumerate(FCU_ANNUNCIATORS, start=FCU_ANNUNCIATOR_FIRST_CELL):
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        # These cells cover generated thin bars fitted to the upper button
        # fronts. The original SEQ2 text decals remain white below the bars.
        x0, y0 = column * cell_width, row * cell_height
        draw.rectangle((x0 + 3, y0 + 3, x0 + cell_width - 4,
                        y0 + cell_height - 4), fill=(48, 255, 78, 255))
    for index, (label, node, _dataref, color) in enumerate(GLARE_ANNUNCIATORS,
                                                             start=GLARE_ANNUNCIATOR_FIRST_CELL):
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box=(column * cell_width + 2, row * cell_height + 2,
             (column + 1) * cell_width - 2, (row + 1) * cell_height - 2)
        if node == 'GEAR_ARROW': gear_arrow(draw,box,color)
        elif node.endswith('_GREEN') or node == 'PUSH_AUTOBKR_LDGGEAR_2_SEQ2':
            gear_triangle(draw,box,color)
        else: centered_text(draw,box,label,color,34)
    if ADIRS_ANNUNCIATOR_FIRST_CELL + len(ADIRS_ANNUNCIATORS) > FCU_LABEL_COLUMNS * FCU_LABEL_ROWS:
        raise ValueError('Annunciator atlas capacity exceeded')
    for index, (label, _node, _dataref, color) in enumerate(ADIRS_ANNUNCIATORS,
                                                         start=ADIRS_ANNUNCIATOR_FIRST_CELL):
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box = (column * cell_width + 2, row * cell_height + 2,
               (column + 1) * cell_width - 2, (row + 1) * cell_height - 2)
        centered_text(draw, box, label, color, 34)
    elec_legends(draw, cell_width, cell_height)
    ovhd_legends(atlas, cell_width, cell_height)
    feedback_legends(atlas, cell_width, cell_height)
    atlas.save(output / 'fcu_labels.png')

    lit = Image.new('RGBA', size, (0, 0, 0, 0))
    lit_draw = ImageDraw.Draw(lit)
    for index, (label, *_placement) in enumerate(FCU_LABELS):
        if index in OVHD_CELLS:
            continue
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box = (
            column * cell_width + 3,
            row * cell_height + 3,
            (column + 1) * cell_width - 3,
            (row + 1) * cell_height - 3,
        )
        text_size = 38 if len(label) <= 5 else 28
        centered_text(lit_draw, box, label, (235, 242, 255, 255), text_size)
    for index, (_label, _node, _dataref) in enumerate(FCU_ANNUNCIATORS, start=FCU_ANNUNCIATOR_FIRST_CELL):
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        x0, y0 = column * cell_width, row * cell_height
        lit_draw.rectangle((x0 + 3, y0 + 3, x0 + cell_width - 4,
                            y0 + cell_height - 4), fill=(48, 255, 78, 255))
    for index, (label, node, _dataref, color) in enumerate(GLARE_ANNUNCIATORS,
                                                             start=GLARE_ANNUNCIATOR_FIRST_CELL):
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box=(column * cell_width + 2, row * cell_height + 2,
             (column + 1) * cell_width - 2, (row + 1) * cell_height - 2)
        if node == 'GEAR_ARROW': gear_arrow(lit_draw,box,color)
        elif node.endswith('_GREEN') or node == 'PUSH_AUTOBKR_LDGGEAR_2_SEQ2':
            gear_triangle(lit_draw,box,color)
        else: centered_text(lit_draw,box,label,color,34)
    for index, (label, _node, _dataref, color) in enumerate(ADIRS_ANNUNCIATORS,
                                                         start=ADIRS_ANNUNCIATOR_FIRST_CELL):
        column, row = index % FCU_LABEL_COLUMNS, index // FCU_LABEL_COLUMNS
        box = (column * cell_width + 2, row * cell_height + 2,
               (column + 1) * cell_width - 2, (row + 1) * cell_height - 2)
        centered_text(lit_draw, box, label, color, 34)
    elec_legends(lit_draw, cell_width, cell_height)
    ovhd_legends(lit, cell_width, cell_height)
    feedback_legends(lit, cell_width, cell_height)
    lit.save(output / 'fcu_labels_LIT.png')


def build_panel_background(output: Path) -> None:
    panels = output / '-PANELS-'
    panels.mkdir(parents=True, exist_ok=True)
    panel = Image.new('RGBA', PANEL_SIZE, (0, 0, 0, 0))
    draw = ImageDraw.Draw(panel)
    # The plugin owns all FCU pixels, including unit legends, managed dots,
    # dashes, Mach decimals and signed V/S/FPA. Keep the unpowered bed dark.
    left, bottom, right, top = FCU_PANEL_RECT
    draw.rectangle((left, PANEL_SIZE[1] - top, right - 1,
                    PANEL_SIZE[1] - bottom - 1), fill=(1, 3, 6, 255))
    for left, bottom, right, top in (*EFIS_BARO_PANEL_RECTS,
            *(rect for side in EFIS_UPPER_PANEL_RECTS for rect in side),
            *(rect for side in EFIS_LOWER_PANEL_RECTS for rect in side)):
        draw.rectangle((left, PANEL_SIZE[1] - top, right - 1, PANEL_SIZE[1] - bottom - 1),
                       fill=(1, 3, 6, 255))
    panel_font = font(11)

    # Panel coordinates have their origin at bottom left; Pillow's origin is
    # top left. Give each live display an opaque black render bed while leaving
    # every other physical display transparent/dark through its original material.
    for display, rects in (('PFD', PFD_PANEL_RECTS), ('ND', ND_PANEL_RECTS),
                           ('EWD', (EWD_PANEL_RECT,))):
        for index, (left, bottom, right, top) in enumerate(rects, start=1):
            image_top = PANEL_SIZE[1] - top
            image_bottom = PANEL_SIZE[1] - bottom
            draw.rectangle((left, image_top, right - 1, image_bottom - 1), fill=(1, 3, 6, 255))
            draw.rectangle((left + 1, image_top + 1, right - 2, image_bottom - 2),
                           outline=(24, 48, 64, 255), width=2)
            draw.text((left + 8, image_top + 7), f'{display} {index}',
                      font=panel_font, fill=(90, 150, 175, 255))

    left, bottom, right, top = EFB_PANEL_RECT
    image_top = PANEL_SIZE[1] - top
    image_bottom = PANEL_SIZE[1] - bottom
    draw.rounded_rectangle((left, image_top, right - 1, image_bottom - 1), radius=14,
                           fill=(4, 10, 18, 255), outline=(38, 73, 92, 255), width=3)
    draw.text((left + 18, image_top + 14), 'A380X FLYPAD - NATIVE PLUGIN',
              font=font(16), fill=(94, 196, 220, 255))
    panel.save(panels / 'Panel.png')


def build_panel_definition(output: Path) -> None:
    # No stock gen_LED instruments may overwrite the custom FCU framebuffer.
    definitions = (
        '# A380X FBW AVIONICS PLUGIN OWNS PFD, ND, EWD, FCU AND EFB REGIONS\n'
        '# FCU managed dashes/dots, Mach, TRK/FPA and annunciations are drawn natively.\n'
        'GROUP A380 NATIVE AVIONICS\n'
        '  LOCKED\n'
        '  HIDE\n'
        'END_GROUP\n'
    )
    (output / 'panel_3d.txt').write_text(definitions, encoding='ascii')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Delete assets from the retired stock X-Plane FD adapter so a focused
    # refresh cannot carry them forward into a custom-avionics package.
    shutil.rmtree(args.output / 'generic' / 'pfd', ignore_errors=True)
    shutil.rmtree(args.output / 'generic' / 'fcu', ignore_errors=True)
    build_label_atlas(args.output)
    build_panel_background(args.output)
    build_panel_definition(args.output)
    report = {
        'output': str(args.output),
        'panel_size': PANEL_SIZE,
        'fcu_screen_height': FCU_SCREEN_HEIGHT,
        'pfd_regions': PFD_PANEL_RECTS,
        'nd_regions': ND_PANEL_RECTS,
        'ewd_region': EWD_PANEL_RECT,
        'efb_region': EFB_PANEL_RECT,
        'efis_baro_regions': EFIS_BARO_PANEL_RECTS,
        'efis_upper_regions': EFIS_UPPER_PANEL_RECTS,
        'efis_lower_regions': EFIS_LOWER_PANEL_RECTS,
        'labels': len(FCU_LABELS),
        'fcu_region': FCU_PANEL_RECT,
        'display_renderer': 'aircraft-native FBW-derived avionics plugin',
    }
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
