#!/usr/bin/env python3
"""Bind the authored ELEC + EMER ELEC PWR overhead panel to the native ELEC panel module.

Patches the INSTALLED cockpit objects in place (nothing is regenerated):

* panel_a380_cockpit_ovhd_knobs_4k.obj / decal_a380_pushbutton_decal.obj /
  decal_a380x_cockpit_decal_emis.obj - momentary press travel for every ELEC pushbutton and
  its legend segments (authored FBW clips), guard covers rotating about their authored hinges.
  The legacy BAT 1/2 and GEN 1-4 blocks that followed the latched stock switches are
  retargeted to the momentary a380x/xp12/elec/pb/<id>_press_ratio travel.
* a380x_cockpit_controls.obj - the six legacy BAT/GEN click meshes are retargeted to the
  native commands (pointing-finger cursor); then, after MARKER, lit legend overlays (45 lamps,
  atlas cells fcu_labels*.png), draw-disabled click boxes for the other pushbuttons (guarded
  ones only while their guard is open) and grab-hand click/drag boxes for the six guards that
  rotate with the guard.
* cockpit_3d/fcu_labels.png / fcu_labels_LIT.png - the ALTN, DISC, AVAIL and ON legend cells
  (59-62, previously empty); every other pixel is preserved (checked).

Vertex/index/UV payloads of the panel objects stay byte-identical (checked). Every file is
preflighted before the first write and backed up by content hash. Re-running is a no-op.

    python3 tools/elec_cockpit_panel.py --source <A380_COCKPIT_LOD00.gltf> \
        --aircraft output/A380X-XP12 --backup <dir> --report <json> [--dry-run]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

from build_cockpit_controls import (
    ADIRS_ANNUNCIATOR_FIRST_CELL, ELEC_LEGEND_CELLS, FCU_LABEL_COLUMNS, FCU_LABEL_ROWS,
    accessor_values, adirs_lamp_face, load_buffer, local_matrix, matrix_multiply, transform_point,
)
from elec_panel_contract import COVERS, CONTROLS, PREFIX, lamps, press_command, pressable_ids
from patch_center_right_animations import animation_block, source_targets, wrap_geometry
from patch_efis_animations import vertex_payload
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

MARKER = '# A380X ELEC PANEL V1'
MOTION_MARKER = '# ELEC MOTION '
COVER_MARKER = '# ELEC COVER '
PANEL_FILES = ('panel_a380_cockpit_ovhd_knobs_4k.obj', 'decal_a380_pushbutton_decal.obj',
               'decal_a380x_cockpit_decal_emis.obj')
CONTROLS_OBJ = 'objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
ATLASES = ('cockpit_3d/fcu_labels.png', 'cockpit_3d/fcu_labels_LIT.png')
# The legacy export animated these buttons from the latched stock switch.
LEGACY_REFS = {'sim/cockpit2/electrical/battery_on[0]': 'bat1', 'sim/cockpit2/electrical/battery_on[1]': 'bat2',
               **{f'sim/cockpit2/electrical/generator_on[{i}]': f'gen{i + 1}' for i in range(4)}}
LEGACY_IDS = frozenset(LEGACY_REFS.values())
LEGACY_MANIPS = {'sim/electrical/battery_1_toggle': 'bat1', 'sim/electrical/battery_2_toggle': 'bat2',
                 **{f'sim/electrical/generator_{i}_toggle': f'gen{i}' for i in range(1, 5)}}
COVER_ACTIVE = 0.6            # a380x_elec_panel.h A380X_ELEC_COVER_ACTIVE
CLICK_MARGIN = 0.0005         # m, around the authored cap / guard
LAMP_NO_BLEND = 'ATTR_no_blend\t0.5'  # lamps write depth (blended geometry does not)
LAMP_HIDE = -1, 0.001         # dark lamps are not drawn (a dim 0.3 lamp stays visible)
CREW_RIGHT, LEGEND_TOP = (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0)  # overhead text reads with its top aft


def node_by_id():
    return {cid: row for row in CONTROLS for cid in (row[0],)}


def caption(cid):
    row = node_by_id()[cid]
    kind = row[3]
    if cid == 'ac_ess_feed':
        return 'AC ESS FEED - AUTO / ALTN (guarded)'
    if cid == 'pax_sys':
        return 'PAX SYS - ON / OFF'
    if kind == 'latch':
        return f'{row[1]} - AUTO / OFF'
    if kind == 'ext':
        return f'{row[1]} - press: ON when AVAIL, OFF'
    if cid.startswith('drive'):
        return f'{row[1]} - press to disconnect IDG (guarded, irreversible in flight)'
    return 'RAT MAN ON - deploy RAT (guarded)'


class Source:
    """Pinned FBW cockpit glTF: node world matrices and local positions."""

    def __init__(self, path: Path):
        self.path = path
        self.document = json.loads(path.read_text())
        self.buffers = [load_buffer(path, row['uri']) for row in self.document['buffers']]
        self.nodes = self.document['nodes']
        self.names = {node['name']: i for i, node in enumerate(self.nodes) if 'name' in node}
        self.parents = {child: i for i, node in enumerate(self.nodes) for child in node.get('children', [])}
        self._world = {}

    def world(self, index):
        if index not in self._world:
            local = local_matrix(self.nodes[index])
            self._world[index] = (matrix_multiply(self.world(self.parents[index]), local)
                                  if index in self.parents else local)
        return self._world[index]

    def local_points(self, name):
        node = self.nodes[self.names[name]]
        return [point for primitive in self.document['meshes'][node['mesh']]['primitives']
                for point in accessor_values(self.document, self.buffers, primitive['attributes']['POSITION'])]


def press_motions(source_path, ids):
    rows = tuple((node, node, f'elec/pb/{cid}_press_ratio', 1)
                 for cid, _caption, node, kind, *_rest in CONTROLS if cid in ids)
    return source_targets(source_path, motions=rows)


def cover_motions(source_path):
    return source_targets(source_path, motions=tuple((node, node, f'elec/cover/{cover}', 1)
                                                     for cover, node, _caption in COVERS),
                          include_child_points=True)


def _block_begin(lines, at):
    depth = 0
    for i in range(at, -1, -1):
        word = lines[i].strip()
        if word == 'ANIM_end':
            depth += 1
        elif word == 'ANIM_begin':
            if not depth:
                return i
            depth -= 1
    raise ValueError('Legacy animation outside ANIM_begin')


def retarget_legacy(text, origins):
    """Latched stock refs -> momentary travel, only in blocks pivoting on the ELEC buttons."""
    lines = text.split('\n')
    changed = []
    for i, line in enumerate(lines):
        words = line.split()
        if len(words) != 2 or words[0] != 'ANIM_trans_begin' or words[1] not in LEGACY_REFS:
            continue
        cid = LEGACY_REFS[words[1]]
        begin = _block_begin(lines, i)
        pivot = next((tuple(map(float, lines[k].split()[1:4])) for k in range(begin + 1, i)
                      if lines[k].split()[:1] == ['ANIM_trans']), None)
        if pivot is None or math.dist(pivot, origins[cid]) > 0.003:
            continue
        lines[i] = line[:len(line) - len(line.lstrip())] + f'ANIM_trans_begin\t{PREFIX}pb/{cid}_press_ratio'
        changed.append(cid)
    return '\n'.join(lines), changed


def patch_panel_object(text, filename, motions, covers, origins):
    changed, legacy = retarget_legacy(text, origins)
    meshes = []
    for marker, targets in ((MOTION_MARKER, motions), (COVER_MARKER, covers)):
        changed, applied, _gear = wrap_geometry(changed, filename, targets, marker)
        meshes.extend(applied)
    if vertex_payload(text) != vertex_payload(changed):
        raise ValueError(f'{filename}: ELEC patch changed geometry, UV or texture payload')
    return changed, dict(legacy_retargeted=legacy, meshes=meshes)


# ------------------------------------------------------------------ controls object
def legend_cell(legend):
    if legend == 'fault':
        return ADIRS_ANNUNCIATOR_FIRST_CELL
    if legend == 'off':
        return ADIRS_ANNUNCIATOR_FIRST_CELL + 1
    return ELEC_LEGEND_CELLS[legend]


def cell_uv(cell):
    column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
    return (column / FCU_LABEL_COLUMNS, 1 - (row + 1) / FCU_LABEL_ROWS,
            (column + 1) / FCU_LABEL_COLUMNS, 1 - row / FCU_LABEL_ROWS)


def lamp_quad(source, node, legend):
    """Corners BL, BR, TR, TL (as read by the crew) lifted 0.4 mm, with atlas UVs."""
    index = source.names[node]
    corners, normal = adirs_lamp_face(source.local_points(node), source.world(index))
    right = [b - a for a, b in zip(corners[0], corners[1])]
    up = [b - a for a, b in zip(corners[0], corners[3])]
    u0, v0, u1, v1 = cell_uv(legend_cell(legend))
    if sum(a * b for a, b in zip(right, CREW_RIGHT)) < 0:
        u0, u1 = u1, u0
    if sum(a * b for a, b in zip(up, LEGEND_TOP)) < 0:
        v0, v1 = v1, v0
    return [(*point, *normal, u, v) for point, (u, v) in zip(corners, ((u0, v0), (u1, v0), (u1, v1), (u0, v1)))]


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(v):
    length = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / length for x in v)


def oriented_box(source, node, margin=CLICK_MARGIN):
    """12-triangle box fitted to the node's local bounds; OBJ8 clockwise fronts, outward normals."""
    points = source.local_points(node)
    low = [min(p[a] for p in points) - margin for a in range(3)]
    high = [max(p[a] for p in points) + margin for a in range(3)]
    x, y, z = low
    X, Y, Z = high
    local = ((x, y, z), (X, y, z), (X, Y, z), (x, Y, z), (x, y, Z), (X, y, Z), (X, Y, Z), (x, Y, Z))
    matrix = source.world(source.names[node])
    world = [transform_point(matrix, p) for p in local]
    centre = tuple(sum(p[a] for p in world) / 8 for a in range(3))
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5))
    vertices, indices = [], []
    for face in faces:
        a, b, c = (world[k] for k in face[:3])
        normal = _unit(_cross(_sub(b, a), _sub(c, a)))
        face_centre = tuple(sum(world[k][axis] for k in face) / 4 for axis in range(3))
        order = (0, 2, 1, 0, 3, 2)
        if sum(n * d for n, d in zip(normal, _sub(face_centre, centre))) < 0:
            normal = tuple(-n for n in normal)
        else:  # mirrored node matrix: keep fronts clockwise when seen from outside
            order = (0, 1, 2, 0, 2, 3)
        base = len(vertices)
        vertices.extend((*world[k], *normal, 0.0, 0.0) for k in face)
        indices.extend(base + k for k in order)
    return vertices, indices


def controls_block(source_path: Path):
    """(vertices, indices, draw commands) appended after MARKER."""
    source = Source(source_path)
    motions = {t['name']: t for t in press_motions(source_path, set(pressable_ids()))}
    covers = {t['dataref'].rsplit('/', 1)[1]: t for t in cover_motions(source_path)}
    rows = {row[0]: row for row in CONTROLS}
    vertices, indices, commands = [], [], ['ATTR_no_cockpit', 'ATTR_manip_none', 'ATTR_draw_enable',
                                           'ATTR_poly_os\t2']
    lamp_report = []
    for dataref, cid, _sequence, legend, legend_node in lamps():
        start, base = len(indices), len(vertices)
        vertices.extend(lamp_quad(source, legend_node, legend))
        indices.extend((base, base + 2, base + 1, base, base + 3, base + 2))
        motion = motions.get(rows[cid][2])
        # Alpha-tested opaque: the later-drawn legend decal object must not paint over the lamp
        # (tools/patch_lamp_depth.py).
        commands.extend([f'# ELEC LAMP {dataref.removeprefix(PREFIX)}', LAMP_NO_BLEND,
                         f'ATTR_light_level\t0\t1\t{dataref}',
                         'ANIM_begin', f'ANIM_hide\t{LAMP_HIDE[0]}\t{LAMP_HIDE[1]}\t{dataref}'])
        commands.extend(animation_block(motion, start, 6, '# ELEC LAMP MOTION ') if motion else [f'TRIS\t{start}\t6'])
        commands.extend(['ANIM_end', 'ATTR_light_level_reset', 'ATTR_blend'])
        lamp_report.append(dict(dataref=dataref, legend=legend, node=legend_node, cell=legend_cell(legend)))
    commands.extend(['ATTR_poly_os\t0', 'ATTR_draw_disable'])
    clicks = []
    for cid in pressable_ids():
        if cid in LEGACY_IDS:
            continue  # the retargeted legacy click mesh of the authored button is used
        node, cover = rows[cid][2], rows[cid][6]
        box_vertices, box_indices = oriented_box(source, node)
        start, base = len(indices), len(vertices)
        vertices.extend(box_vertices)
        indices.extend(base + i for i in box_indices)
        line = f'ATTR_manip_command\tbutton\t{press_command(cid)}\t{caption(cid)}'
        if cover:
            commands.extend(['ANIM_begin', f'ANIM_hide\t-1\t{obj8_fmt(COVER_ACTIVE - 0.001)}\t{PREFIX}cover/{cover}',
                             line, f'TRIS\t{start}\t{len(box_indices)}', 'ATTR_manip_none', 'ANIM_end'])
        else:
            commands.extend([line, f'TRIS\t{start}\t{len(box_indices)}', 'ATTR_manip_none'])
        clicks.append(cid)
    for cover, node, label in COVERS:
        box_vertices, box_indices = oriented_box(source, node)
        start, base = len(indices), len(vertices)
        vertices.extend(box_vertices)
        indices.extend(base + i for i in box_indices)
        block = animation_block(covers[cover], start, len(box_indices), '# ELEC COVER CLICK ')
        manip = (f'ATTR_manip_command\thand\t{PREFIX}{cover}_cover_toggle\t'
                 f'{label} - click to open/close, or hold and drag down/up to move it')
        tris = block.index(next(row for row in block if row.startswith('TRIS')))
        commands.extend([*block[:tris], manip, block[tris], 'ATTR_manip_none', *block[tris + 1:]])
        clicks.append(cover + '_cover')
    commands.append('ATTR_draw_enable')
    return vertices, indices, commands, dict(lamps=lamp_report, click_targets=clicks)


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


def retarget_legacy_manips(text):
    """Legacy stock BAT/GEN click meshes -> native toggle commands with the button cursor."""
    changed = []
    out = []
    for line in text.split('\n'):
        words = line.split('\t')
        if len(words) >= 4 and words[0] == 'ATTR_manip_command' and words[2] in LEGACY_MANIPS:
            cid = LEGACY_MANIPS[words[2]]
            line = f'ATTR_manip_command\tbutton\t{press_command(cid)}\t{caption(cid)}'
            changed.append(cid)
        out.append(line)
    return '\n'.join(out), changed


def augment_controls(text, source_path):
    text, legacy = retarget_legacy_manips(text)
    if MARKER in text:
        return text, dict(legacy_manips=legacy, appended=False)
    present = [press_command(cid) for cid in pressable_ids() if cid not in LEGACY_IDS
               if '\t' + press_command(cid) + '\t' in text]
    if present:
        raise ValueError(f'ELEC click targets already exist without {MARKER}: {present}')
    missing = [press_command(cid) for cid in LEGACY_IDS if '\t' + press_command(cid) + '\t' not in text]
    if missing:
        raise ValueError(f'Legacy BAT/GEN click meshes not found: {missing}')
    vertices, indices, commands, report = controls_block(source_path)
    return _append(text, vertices, indices, commands), dict(legacy_manips=legacy, appended=True, **report)


# ------------------------------------------------------------------ atlas
def atlas_cells(path: Path, lit: bool):
    """Return the atlas with the ELEC legend cells drawn (other pixels unchanged)."""
    from PIL import Image, ImageDraw
    from build_fcu_panel import centered_text
    from elec_panel_contract import LEGEND_COLORS, LEGEND_TEXT
    image = Image.open(path).convert('RGBA')
    width, height = image.size
    cell_w, cell_h = width // FCU_LABEL_COLUMNS, height // FCU_LABEL_ROWS
    draw = ImageDraw.Draw(image)
    for legend, cell in ELEC_LEGEND_CELLS.items():
        column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
        box = (column * cell_w, row * cell_h, (column + 1) * cell_w, (row + 1) * cell_h)
        region = image.crop(box)
        if region.getextrema()[3][1] != 0:
            existing = region
            fresh = Image.new('RGBA', region.size, (0, 0, 0, 0))
            fresh_draw = ImageDraw.Draw(fresh)
            centered_text(fresh_draw, (2, 2, cell_w - 2, cell_h - 2), LEGEND_TEXT[legend], LEGEND_COLORS[legend], 34)
            if list(existing.getdata()) != list(fresh.getdata()):
                raise ValueError(f'{path.name}: atlas cell {cell} is already used')
            continue
        centered_text(draw, (box[0] + 2, box[1] + 2, box[2] - 2, box[3] - 2), LEGEND_TEXT[legend],
                      LEGEND_COLORS[legend], 34)
    return image


def atlas_patch(path: Path):
    from PIL import Image
    original = Image.open(path).convert('RGBA')
    patched = atlas_cells(path, path.name.endswith('_LIT.png'))
    changed_cells = set()
    for cell in range(FCU_LABEL_COLUMNS * FCU_LABEL_ROWS):
        column, row = cell % FCU_LABEL_COLUMNS, cell // FCU_LABEL_COLUMNS
        w, h = original.size[0] // FCU_LABEL_COLUMNS, original.size[1] // FCU_LABEL_ROWS
        box = (column * w, row * h, (column + 1) * w, (row + 1) * h)
        if list(original.crop(box).getdata()) != list(patched.crop(box).getdata()):
            changed_cells.add(cell)
    if changed_cells - set(ELEC_LEGEND_CELLS.values()):
        raise ValueError(f'{path.name}: atlas patch touched foreign cells {sorted(changed_cells)}')
    return patched, sorted(changed_cells)


# ------------------------------------------------------------------ main
def plan(source: Path, aircraft: Path):
    targets_press = press_motions(source, set(pressable_ids()) - LEGACY_IDS)
    targets_cover = cover_motions(source)
    src = Source(source)
    origins = {cid: transform_point(src.world(src.names[node]), (0, 0, 0))
               for cid, _c, node, *_rest in CONTROLS if cid in LEGACY_IDS}
    staged, report = [], dict(panel=[], controls=None, atlas=[])
    for name in PANEL_FILES:
        path = aircraft / 'objects/interior/cockpit-panels' / name
        original = path.read_text()
        changed, summary = patch_panel_object(original, name, targets_press, targets_cover, origins)
        staged.append((path, original, changed))
        report['panel'].append(dict(path=str(path), **summary))
    applied = {m['name'] for row in report['panel'] for m in row['meshes']}
    expected = {t['name'] for t in (*targets_press, *targets_cover)}
    first_run = bool(applied)
    if first_run and expected - applied:
        raise ValueError(f'ELEC meshes not found in the panel objects: {sorted(expected - applied)}')
    path = aircraft / CONTROLS_OBJ
    original = path.read_text()
    changed, summary = augment_controls(original, source)
    staged.append((path, original, changed))
    report['controls'] = dict(path=str(path), **summary)
    atlas = []
    for relative in ATLASES:
        image, cells = atlas_patch(aircraft / relative)
        atlas.append((aircraft / relative, image, cells))
        report['atlas'].append(dict(path=relative, cells_written=cells))
    report['press_targets'] = sorted({t['name'] for t in targets_press})
    report['cover_targets'] = [{k: v for k, v in t.items() if k != 'points'} for t in targets_cover]
    return staged, atlas, report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--aircraft', required=True, type=Path)
    parser.add_argument('--backup', type=Path)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not args.dry_run and args.backup is None:
        parser.error('--backup is required unless --dry-run is used')
    staged, atlas, report = plan(args.source, args.aircraft)
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
    print(json.dumps(dict(files=[f['path'].rsplit('/', 1)[1] for f in files if f['changed']],
                          atlas=[a['cells_written'] for a in report['atlas']],
                          lamps=len((report['controls'] or {}).get('lamps', [])),
                          report=str(args.report))))


if __name__ == '__main__':
    main()
