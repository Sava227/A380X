#!/usr/bin/env python3
"""Hide the PFD/ND panel-atlas copies in an existing a380x_cockpit_controls.obj while the cockpit
devices are active (the same change build_cockpit_controls.py now generates).

X-Plane draws these ATTR_cockpit_region quads over the a380x.xp12.pfd.* / nd.* device quads, so
the 320x416 PFD and 180x234 ND atlas copies were what the pilot saw (in-sim marker test: both
showed the copy; the ND looked blurry). Each of the four quads is wrapped in
ANIM_hide 0.5 1.5 a380x/xp12/xpkit/<pfd|nd>_devices_active, so a copy reappears only when its
devices are unavailable. Quads are found by their UVs (PFD_PANEL_RECTS / ND_PANEL_RECTS), never
by line number; exactly four must match. Idempotent (marker below); writes in place.

    python3 tools/hide_atlas_screens.py <aircraft>/objects/interior/cockpit-controls/a380x_cockpit_controls.obj
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from avionics_panel_layout import ND_PANEL_RECTS, PANEL_SIZE, PFD_PANEL_RECTS  # noqa: E402
from build_cockpit_controls import ATLAS_SCREEN_HIDE  # noqa: E402

MARKER = '# A380X ATLAS SCREENS HIDDEN WHILE DEVICES ACTIVE V1'


def patch(text: str) -> tuple[str, int]:
    if MARKER in text:
        return text, 0
    lines = text.splitlines()
    uv = [tuple(map(float, l.split()[7:9])) for l in lines if l.startswith('VT')]
    idx = [int(v) for l in lines if l.split()[:1] in (['IDX'], ['IDX10']) for v in l.split()[1:]]
    rects = [('pfd', r) for r in PFD_PANEL_RECTS] + [('nd', r) for r in ND_PANEL_RECTS]
    w, h = PANEL_SIZE
    out, region, found = [], False, {}
    for n, line in enumerate(lines):
        f = line.split()
        if f[:2] == ['ATTR_cockpit_region', '0']:
            region = True
        elif f[:1] in (['ATTR_no_cockpit'], ['ATTR_cockpit_device']):
            region = False
        if region and f[:1] == ['TRIS'] and int(f[2]) == 6:
            us = [uv[i][0] for i in idx[int(f[1]):int(f[1]) + 6]]
            vs = [uv[i][1] for i in idx[int(f[1]):int(f[1]) + 6]]
            for kind, (left, bottom, right, top) in rects:
                if (abs(min(us) - left / w) < 1e-4 and abs(max(us) - right / w) < 1e-4
                        and abs(min(vs) - bottom / h) < 1e-4 and abs(max(vs) - top / h) < 1e-4):
                    found[n] = kind
        out.append(line)
    if len(found) != 4 or sorted(found.values()) != ['nd', 'nd', 'pfd', 'pfd']:
        raise SystemExit(f'expected the 2 PFD + 2 ND atlas quads, found {sorted(found.values())}')
    result = []
    for n, line in enumerate(out):
        if n in found:
            result += [MARKER, 'ANIM_begin', f'ANIM_hide\t0.5\t1.5\t{ATLAS_SCREEN_HIDE[found[n]]}', line, 'ANIM_end']
        else:
            result.append(line)
    return '\n'.join(result) + '\n', 4


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('obj', type=Path)
    path = ap.parse_args().obj
    text, changed = patch(path.read_text(encoding='utf-8'))
    if changed:
        path.write_text(text, encoding='utf-8')
    print(f'{path.name}: {changed} atlas display quads wrapped' if changed else f'{path.name}: already patched')


if __name__ == '__main__':
    main()
