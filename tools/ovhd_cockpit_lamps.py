#!/usr/bin/env python3
"""Bind every authored overhead pushbutton legend (outside ELEC and ADIRS) to the native
overhead lamp module (native/avionics/a380x_ovhd_lamps.h / .inc, docs/OVERHEAD_LAMPS.md).

Patches the INSTALLED cockpit objects and atlas in place (nothing is regenerated):

* a380x_cockpit_controls.obj - after MARKER, one lit legend quad per lamp of
  tools/ovhd_lamps_contract.py on its authored FBW <node>_SEQ1/_SEQ2 face (lifted 0.4 mm,
  ATTR_light_level by a380x/xp12/ovhd/lamp/<id>_upper|_lower, hidden when dark, the same
  proven construction as the ELEC/ADIRS/FCU lamps). A quad follows its pushbutton exactly as
  the installed legend decal does: the decal's transform in decal_a380_pushbutton_decal.obj is
  evaluated at every key of its latched stock dataref (APU MASTER/START, bleeds, packs,
  anti-ice, probes, feed pumps) and replayed as an ANIM_trans table. Then draw-disabled click
  targets: CREW OXY SUPPLY (sim/oxy/crew_valve_toggle) and the ANN LT switch
  (a380x/xp12/ovhd/ann_lt_up / _down, up/down cursor).
* panel_a380_cockpit_ovhd_knobs_4k.obj - the ANN LT lever rotates about its authored FBW
  hinge by a380x/xp12/ovhd/ann_lt_switch (0 TEST, 0.5 BRT, 1 DIM).
* cockpit_3d/fcu_labels.png / fcu_labels_LIT.png - the new legend cells 0-13 (previously the
  unused FCU daylight-label cells: no drawn geometry samples them, checked); every other pixel
  is preserved (checked). tools/build_fcu_panel.py draws the same cells.

Vertex/index/UV payloads of the panel object stay byte-identical (checked). Every file is
preflighted before the first write and backed up by content hash. Re-running is a no-op.

    python3 tools/ovhd_cockpit_lamps.py --source <A380_COCKPIT_LOD00.gltf> \\
        --aircraft output/A380X-XP12 --backup <dir> --report <json> [--dry-run]
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'blender'))
from obj8_preview import parse_obj8  # noqa: E402
from build_cockpit_controls import FCU_LABEL_COLUMNS, FCU_LABEL_ROWS, adirs_lamp_face  # noqa: E402
from elec_cockpit_panel import CREW_RIGHT, LEGEND_TOP, LAMP_HIDE, LAMP_NO_BLEND, Source, cell_uv, oriented_box  # noqa: E402
from ovhd_lamps_contract import (ANN_LT_NODE, CLICKS, LEGENDS, NEW_LEGEND_CELLS, PREFIX,  # noqa: E402
                                 lamps)
from patch_center_right_animations import source_targets, wrap_geometry  # noqa: E402
from patch_efis_animations import vertex_payload  # noqa: E402
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

MARKER = '# A380X OVHD LAMPS V1'
LAMP_COMMENT = '# OVHD LAMP '
ANN_MARKER = '# OVHD ANN LT MOTION '
CONTROLS_OBJ = 'objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
DECAL_OBJ = 'objects/interior/cockpit-panels/decal_a380_pushbutton_decal.obj'
KNOBS_OBJ = 'objects/interior/cockpit-panels/panel_a380_cockpit_ovhd_knobs_4k.obj'
ATLASES = ('cockpit_3d/fcu_labels.png', 'cockpit_3d/fcu_labels_LIT.png')
LIFT = 0.0004                 # m, legend quad above the authored face (as ELEC / ADIRS)
ANN_DATAREF = PREFIX + 'ann_lt_switch'
ANN_CLICK = ('ATTR_manip_command_switch_up_down\tup_down\t' + PREFIX + 'ann_lt_up\t' + PREFIX +
             'ann_lt_down\tANN LT - TEST / BRT / DIM')
# Latched stock pushbutton travel the legacy export animated the overhead caps/legends with.
MOTION_REFS = frozenset((
    'sim/cockpit2/electrical/APU_starter_switch', 'sim/cockpit2/bleedair/actuators/apu_bleed',
    'sim/cockpit2/bleedair/actuators/pack_left', 'sim/cockpit2/bleedair/actuators/pack_right',
    'sim/cockpit2/ice/ice_surface_hot_bleed_air_on', 'sim/cockpit2/ice/ice_window_heat_on',
    *(f'sim/cockpit2/bleedair/actuators/engine_bleed_sov[{i}]' for i in range(4)),
    *(f'sim/cockpit2/ice/ice_inlet_heat_on_per_engine[{i}]' for i in range(4)),
    *(f'sim/cockpit2/engine/actuators/fuel_pump_on[{i}]' for i in range(4)),
))


# ------------------------------------------------------------------ geometry
def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def face(source: Source, node: str):
    """Authored legend face: corners BL, BR, TR, TL (as read by the crew) ON the face, normal.

    One FBW face (PUSH_OVHD_AIRCOND_ENG2BLEED_SEQ1) covers the whole cap (16 vertices, 14 x 14 mm)
    instead of its upper half; a SEQ1 / SEQ2 face taller than 0.75 x its width is cut to its
    upper / lower half, the size of every other legend face (14 x 7 mm)."""
    corners, normal = adirs_lamp_face(source.local_points(node), source.world(source.names[node]))
    corners = [tuple(p - LIFT * n for p, n in zip(c, normal)) for c in corners]
    if math.dist(corners[0], corners[3]) > 0.75 * math.dist(corners[0], corners[1]):
        bl, br, tr, tl = corners
        ml, mr = (tuple((a + b) / 2 for a, b in zip(bl, tl)), tuple((a + b) / 2 for a, b in zip(br, tr)))
        corners = [ml, mr, tr, tl] if node.endswith('_SEQ1') else [bl, br, mr, ml]
    return corners, normal


def lamp_quad(corners, normal, legend, offset=(0.0, 0.0, 0.0)):
    """Lifted quad with the legend cell's UVs, upright for the crew."""
    right, up = _sub(corners[1], corners[0]), _sub(corners[3], corners[0])
    u0, v0, u1, v1 = cell_uv(LEGENDS[legend][2])
    if _dot(right, CREW_RIGHT) < 0:
        u0, u1 = u1, u0
    if _dot(up, LEGEND_TOP) < 0:
        v0, v1 = v1, v0
    points = [tuple(p + LIFT * n + o for p, n, o in zip(c, normal, offset)) for c in corners]
    return [(*p, *normal, u, v) for p, (u, v) in zip(points, ((u0, v0), (u1, v0), (u1, v1), (u0, v1)))]


class DecalMotion:
    """Where the installed legend decal of a face is, at every key of its stock dataref."""

    def __init__(self, text: str):
        self.lines = text.splitlines()
        self.keys = collections.defaultdict(set)
        current = None
        for line in self.lines:
            words = line.split()
            if words[:1] == ['ANIM_trans_begin']:
                current = words[1]
            elif words[:1] == ['ANIM_trans_key'] and current:
                self.keys[current].add(float(words[1]))
            elif words[:1] == ['ANIM_trans_end']:
                current = None
        self._cache = {}

    def evaluate(self, values):
        key = tuple(sorted(values.items()))
        if key not in self._cache:
            obj = parse_obj8(self.lines, values=dict(values), source=DECAL_OBJ)
            positions = obj.positions
            triangles = []
            for draw in obj.draws:
                m = draw.matrix
                for start in range(draw.start, draw.start + draw.count, 3):
                    tri = []
                    for k in range(3):
                        i = obj.indices[start + k] * 3
                        p = positions[i:i + 3]
                        tri.append(tuple(m[r * 4] * p[0] + m[r * 4 + 1] * p[1] + m[r * 4 + 2] * p[2] + m[r * 4 + 3]
                                         for r in range(3)))
                    triangles.append((start, draw.transform_datarefs, tri))
            self._cache[key] = triangles
        return self._cache[key]

    def match(self, corners, normal):
        """Decal triangles lying on this face at rest (all datarefs 0)."""
        origin, right, up = corners[0], _sub(corners[1], corners[0]), _sub(corners[3], corners[0])
        width, height = math.sqrt(_dot(right, right)), math.sqrt(_dot(up, up))
        found = []
        for start, refs, tri in self.evaluate({}):
            centre = tuple(sum(p[a] for p in tri) / 3 for a in range(3))
            n = _cross(_sub(tri[1], tri[0]), _sub(tri[2], tri[0]))
            length = math.sqrt(_dot(n, n))
            if length < 1e-14 or abs(_dot(n, normal)) / length < 0.95:
                continue
            rel = _sub(centre, origin)
            if abs(_dot(rel, normal)) > 0.003:
                continue
            x, y = _dot(rel, right) / width, _dot(rel, up) / height
            if -0.0005 <= x <= width + 0.0005 and -0.0005 <= y <= height + 0.0005:
                found.append((start, refs))
        return found

    def offsets(self, corners, normal):
        """(dataref or None, [(key value, world offset)]) of the decal relative to the glTF face."""
        found = self.match(corners, normal)
        if not found:
            return None, [(0.0, (0.0, 0.0, 0.0))]
        refs = {ref for _start, chain in found for ref in chain}
        moving = sorted(refs & MOTION_REFS)
        if refs - MOTION_REFS or len(moving) > 1:
            raise ValueError(f'unexpected legend decal animation {sorted(refs)}')
        starts = {start for start, _refs in found}
        rest = [sum(c[a] for c in corners) / 4 for a in range(3)]
        dataref = moving[0] if moving else None
        values = sorted(self.keys[dataref]) if dataref else [0.0]
        result = []
        for value in values:
            points = [p for start, _refs, tri in self.evaluate({dataref: value} if dataref else {})
                      if start in starts for p in tri]
            centre = [(min(p[a] for p in points) + max(p[a] for p in points)) / 2 for a in range(3)]
            result.append((value, tuple(c - r for c, r in zip(centre, rest))))
        return dataref, result


def cap_keys(knobs_text: str, near, dataref: str):
    """Key table [(value, local delta)] of the installed cap block nearest `near` driven by `dataref`."""
    best, pivot, lines = None, None, knobs_text.split('\n')
    for i, line in enumerate(lines):
        words = line.split()
        if words[:1] == ['ANIM_trans'] and len(words) == 7:
            pivot = tuple(map(float, words[1:4]))
        elif words == ['ANIM_trans_begin', dataref] and pivot is not None:
            keys = []
            for row in lines[i + 1:]:
                fields = row.split()
                if fields[:1] != ['ANIM_trans_key']:
                    break
                keys.append((float(fields[1]), tuple(map(float, fields[2:5]))))
            distance = math.dist(pivot, near)
            if distance < 0.005 and (best is None or distance < best[0]):
                best = (distance, keys)
    return best[1] if best else None


def apu_master_motion(source: Source, knobs_text: str, start_keyed):
    """The legacy export moves the APU MASTER SW cap (latched, APU_starter_switch 0..1) but not its
    legend decal; its lamps follow the cap. Direction: the APU START legend travel (same panel,
    same local -Y press axis), magnitude: the MASTER cap key table."""
    ref = 'sim/cockpit2/electrical/APU_starter_switch'
    from build_cockpit_controls import transform_point
    origin = transform_point(source.world(source.names['PUSH_OVHD_APU_MASTERSW']), (0, 0, 0))
    keys = cap_keys(knobs_text, origin, ref)
    start = dict(start_keyed)
    if not keys or 2.0 not in start:
        raise ValueError('APU MASTER SW cap or APU START legend travel not found')
    travel = math.sqrt(_dot(start[2.0], start[2.0])) - math.sqrt(_dot(start[0.0], start[0.0]))
    direction = tuple(v / math.sqrt(_dot(start[2.0], start[2.0])) for v in start[2.0])
    base = keys[0][1]
    keyed = [(value, tuple(d * math.dist(local, base) for d in direction)) for value, local in keys]
    if not 0.0005 < travel < 0.003 or keyed[0][0] != 0.0:
        raise ValueError('Unexpected APU cap travel')
    return ref, keyed


def controls_block(source_path: Path, decal_text: str, knobs_text: str = ''):
    """(vertices, indices, draw commands, report) appended after MARKER."""
    source = Source(source_path)
    motion = DecalMotion(decal_text)
    vertices, indices = [], []
    commands = ['ATTR_no_cockpit', 'ATTR_manip_none', 'ATTR_draw_enable', 'ATTR_poly_os\t2']
    report = []
    start_corners, start_normal = face(source, 'PUSH_OVHD_APU_START_SEQ1')
    master = apu_master_motion(source, knobs_text, motion.offsets(start_corners, start_normal)[1]) \
        if knobs_text else None
    for dataref, pid, sequence, legend, node in lamps():
        corners, normal = face(source, node)
        ref, keyed = motion.offsets(corners, normal)
        if pid == 'apu_master' and master:
            if ref is not None:
                raise ValueError('APU MASTER SW legend decal is animated now; drop the cap override')
            ref, keyed = master
        static = keyed[0][1] if ref is None else (0.0, 0.0, 0.0)
        start, base = len(indices), len(vertices)
        vertices.extend(lamp_quad(corners, normal, legend, static))
        indices.extend((base, base + 2, base + 1, base, base + 3, base + 2))
        commands += [f'{LAMP_COMMENT}{dataref.removeprefix(PREFIX)}', LAMP_NO_BLEND, f'ATTR_light_level\t0\t1\t{dataref}',
                     'ANIM_begin', f'ANIM_hide\t{LAMP_HIDE[0]}\t{LAMP_HIDE[1]}\t{dataref}']
        if ref:
            commands += [f'ANIM_trans_begin\t{ref}',
                         *(f'ANIM_trans_key\t{obj8_fmt(value)}\t' + '\t'.join(f'{obj8_fmt(v)}' for v in delta)
                           for value, delta in keyed), 'ANIM_trans_end']
        commands += [f'TRIS\t{start}\t6', 'ANIM_end', 'ATTR_light_level_reset', 'ATTR_blend']
        report.append(dict(dataref=dataref, legend=legend, node=node, cell=LEGENDS[legend][2],
                           motion=ref, keys=[[v, [round(x * 1000, 4) for x in d]] for v, d in keyed]))
    commands += ['ATTR_poly_os\t0', 'ATTR_draw_disable']
    clicks = []
    for pid, command, label in CLICKS:
        node = next(r[4] for r in lamps() if r[1] == pid).rsplit('_SEQ', 1)[0]
        box_vertices, box_indices = oriented_box(source, node)
        start, base = len(indices), len(vertices)
        vertices.extend(box_vertices)
        indices.extend(base + i for i in box_indices)
        commands += [f'ATTR_manip_command\tbutton\t{command}\t{label}', f'TRIS\t{start}\t{len(box_indices)}',
                     'ATTR_manip_none']
        clicks.append(command)
    box_vertices, box_indices = oriented_box(source, ANN_LT_NODE, margin=0.002)
    start, base = len(indices), len(vertices)
    vertices.extend(box_vertices)
    indices.extend(base + i for i in box_indices)
    commands += [ANN_CLICK, f'TRIS\t{start}\t{len(box_indices)}', 'ATTR_manip_none', 'ATTR_draw_enable']
    clicks.append(PREFIX + 'ann_lt_up/down')
    return vertices, indices, commands, dict(lamps=report, click_targets=clicks)


def _append(text, vertices, indices, commands):
    lines = text.splitlines()
    row = next(i for i, line in enumerate(lines) if line.startswith('POINT_COUNTS'))
    count = list(map(int, lines[row].split()[1:]))
    if sum(line.startswith('VT\t') for line in lines) != count[0] or \
            sum(len(line.split()) - 1 for line in lines if re.match(r'^IDX(?:10)?\s', line)) != count[3]:
        raise ValueError('Existing cockpit controls geometry is inconsistent; refusing to rewrite it')
    at = max(i for i, line in enumerate(lines) if line.startswith('VT\t')) + 1
    lines[at:at] = ['VT\t' + '\t'.join(f'{obj8_fmt(v)}' for v in vertex) for vertex in vertices]
    at = max(i for i, line in enumerate(lines) if re.match(r'^IDX(?:10)?\s', line)) + 1
    lines[at:at] = ['IDX\t' + str(count[0] + i) for i in indices]
    lines[row] = 'POINT_COUNTS\t' + '\t'.join(map(str, (count[0] + len(vertices), count[1], count[2],
                                                        count[3] + len(indices))))
    shifted = []
    for line in commands:
        words = line.split()
        if words and words[0] == 'TRIS':
            line = f'TRIS\t{int(words[1]) + count[3]}\t{words[2]}'
        shifted.append(line)
    return '\n'.join([*lines, '', MARKER, *shifted, ''])


def augment_controls(text, source_path, decal_text, knobs_text=''):
    if MARKER in text:
        missing = [row[0] for row in lamps() if f'ATTR_light_level\t0\t1\t{row[0]}\n' not in text]
        if missing:
            raise ValueError(f'Incomplete previous overhead lamp patch: {missing[:5]}')
        return text, dict(appended=False)
    present = [row[0] for row in lamps() if row[0] in text]
    if present:
        raise ValueError(f'Overhead lamp datarefs already used without {MARKER}: {present[:5]}')
    vertices, indices, commands, report = controls_block(source_path, decal_text, knobs_text)
    return _append(text, vertices, indices, commands), dict(appended=True, **report)


# ------------------------------------------------------------------ atlas
def cell_box(cell, width, height):
    w, h = width // FCU_LABEL_COLUMNS, height // FCU_LABEL_ROWS
    column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
    return column * w, row * h, (column + 1) * w, (row + 1) * h


def legend_cell_image(legend, size):
    """A fresh cell: legend text centred in its colour on transparent black (ELEC style); the same
    function tools/build_fcu_panel.py uses when it regenerates the atlas."""
    from build_fcu_panel import ovhd_legend_cell
    return ovhd_legend_cell(legend, size)


def atlas_usage(controls_text: str, exclude_marker=MARKER):
    """Atlas cells sampled by drawn non-panel-texture triangles of the controls object."""
    head = controls_text.split('\n' + exclude_marker + '\n')[0]
    uvs, indices, used = [], [], set()
    draw, cockpit = True, False
    for line in head.split('\n'):
        if line.startswith('VT\t'):
            words = line.split('\t')
            uvs.append((float(words[7]), float(words[8])))
        elif line.startswith(('IDX\t', 'IDX10\t')):
            indices.extend(int(v) for v in line.split()[1:])
    for line in head.split('\n'):
        words = line.split()
        if not words:
            continue
        if words[0] == 'ATTR_draw_disable':
            draw = False
        elif words[0] == 'ATTR_draw_enable':
            draw = True
        elif words[0] in ('ATTR_cockpit', 'ATTR_cockpit_region', 'ATTR_cockpit_device', 'ATTR_cockpit_lit_only'):
            cockpit = True
        elif words[0] == 'ATTR_no_cockpit':
            cockpit = False
        elif words[0] == 'TRIS' and draw and not cockpit:
            start, count = int(words[1]), int(words[2])
            for t in range(start, start + count, 3):
                tri = [uvs[indices[t + k]] for k in range(3)]
                u = sum(p[0] for p in tri) / 3
                v = sum(p[1] for p in tri) / 3
                used.add(min(FCU_LABEL_ROWS - 1, int((1 - v) * FCU_LABEL_ROWS)) * FCU_LABEL_COLUMNS +
                         min(FCU_LABEL_COLUMNS - 1, int(u * FCU_LABEL_COLUMNS)))
    return used


def atlas_patch(path: Path, controls_text: str):
    """Return (image, changed cells): the new legend cells drawn, every other pixel unchanged."""
    from PIL import Image
    image = Image.open(path).convert('RGBA')
    original = image.copy()
    used = atlas_usage(controls_text)
    changed = []
    for legend, cell in sorted(NEW_LEGEND_CELLS.items(), key=lambda row: row[1]):
        box = cell_box(cell, *image.size)
        fresh = legend_cell_image(legend, (box[2] - box[0], box[3] - box[1]))
        if image.crop(box).tobytes() == fresh.tobytes():
            continue
        if cell in used:
            raise ValueError(f'{path.name}: atlas cell {cell} is sampled by existing geometry')
        image.paste(fresh, box[:2])
        changed.append(cell)
    for cell in range(FCU_LABEL_COLUMNS * FCU_LABEL_ROWS):
        if cell in changed:
            continue
        box = cell_box(cell, *image.size)
        if image.crop(box).tobytes() != original.crop(box).tobytes():
            raise ValueError(f'{path.name}: atlas patch touched foreign cell {cell}')
    return image, changed


# ------------------------------------------------------------------ ANN LT lever
def ann_lt_targets(source: Path):
    return source_targets(source, motions=((ANN_LT_NODE, ANN_LT_NODE, 'ovhd/ann_lt_switch', 1),))


def patch_knobs(text: str, targets):
    if ANN_DATAREF in text and ANN_MARKER not in text:
        raise ValueError('ANN LT lever already animated without the overhead lamp marker')
    changed, applied, _gear = wrap_geometry(text, Path(KNOBS_OBJ).name, targets, ANN_MARKER)
    if vertex_payload(text) != vertex_payload(changed):
        raise ValueError('ANN LT patch changed geometry, UV or texture payload')
    if ANN_MARKER not in changed:
        raise ValueError('ANN LT lever geometry not found in the overhead knobs object')
    return changed, applied


# ------------------------------------------------------------------ main
def plan(source: Path, aircraft: Path):
    staged, report = [], dict(controls=None, knobs=None, atlas=[])
    knobs_path = aircraft / KNOBS_OBJ
    knobs_original = knobs_path.read_text()
    path = aircraft / CONTROLS_OBJ
    original = path.read_text()
    decal = (aircraft / DECAL_OBJ).read_text()
    changed, summary = augment_controls(original, source, decal, knobs_original)
    staged.append((path, original, changed))
    report['controls'] = dict(path=str(path), **summary)
    path, original = knobs_path, knobs_original
    knobs, applied = patch_knobs(original, ann_lt_targets(source))
    staged.append((path, original, knobs))
    report['knobs'] = dict(path=str(path), ann_lt_lever=applied)
    atlas = []
    for relative in ATLASES:
        image, cells = atlas_patch(aircraft / relative, staged[0][1])   # the controls object as installed
        atlas.append((aircraft / relative, image, cells))
        report['atlas'].append(dict(path=relative, cells_written=cells))
    return staged, atlas, report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--aircraft', required=True, type=Path)
    parser.add_argument('--backup', type=Path)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--stage', type=Path, help='Write the patched files under this directory (aircraft-relative '
                        'paths, plus the decal object for previews) instead of the aircraft; implies no install')
    args = parser.parse_args()
    if not args.dry_run and args.backup is None and args.stage is None:
        parser.error('--backup is required unless --dry-run or --stage is used')
    staged, atlas, report = plan(args.source, args.aircraft)
    if args.stage:
        import shutil
        for path, _original, changed in staged:
            target = args.stage / path.relative_to(args.aircraft)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(changed)
        for path, image, _cells in atlas:
            target = args.stage / path.relative_to(args.aircraft)
            target.parent.mkdir(parents=True, exist_ok=True)
            image.save(target)
        (args.stage / DECAL_OBJ).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(args.aircraft / DECAL_OBJ, args.stage / DECAL_OBJ)
        args.dry_run = True
    files = []
    for path, original, changed in staged:
        digest = hashlib.sha256(original.encode()).hexdigest()
        if changed != original and not args.dry_run:
            backup = args.backup / (digest[:16] + '-' + path.name)
            backup.parent.mkdir(parents=True, exist_ok=True)
            if not backup.exists():
                backup.write_text(original)
            path.write_text(changed)
        files.append(dict(path=str(path), changed=changed != original, before_sha256=digest))
    for path, image, cells in atlas:
        if cells and not args.dry_run:
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            backup = args.backup / (digest[:16] + '-' + path.name)
            backup.parent.mkdir(parents=True, exist_ok=True)
            if not backup.exists():
                backup.write_bytes(path.read_bytes())
            image.save(path)
    report.update(dry_run=args.dry_run, files=files)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    moving = sum(1 for row in (report['controls'] or {}).get('lamps', []) if row['motion'])
    print(json.dumps(dict(files=[f['path'].rsplit('/', 1)[1] for f in files if f['changed']],
                          atlas=[a['cells_written'] for a in report['atlas']],
                          lamps=len((report['controls'] or {}).get('lamps', [])), moving_lamps=moving,
                          report=str(args.report))))


if __name__ == '__main__':
    main()
