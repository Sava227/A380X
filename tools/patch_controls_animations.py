#!/usr/bin/env python3
"""Animate the remaining EFIS, ISIS 1/2 and ECAM-brightness controls in the INSTALLED cockpit OBJs.

Like patch_fcu_animations.py this only rewrites OBJ draw commands: vertex, index, UV, normal and
texture payloads stay byte-identical (checked). Motion comes from the authored FBW glTF clips
(pivot, axis, travel/angle and the exact mesh set including legend children), so press
direction is the authored button travel into the panel and knob axes are the authored shafts.

* EFIS keys that the legacy export already moved with the LATCHED state (CSTR, WPT, NDB, ARPT,
  VOR1, WX, TERR, TRAF) are retargeted to the momentary `<key>_pressed` travel datarefs.
* EFIS keys without motion (VOR.D, VOR2, TAXI, LS, VV), all ISIS 1/2 pushbuttons, the ISIS
  SET/SEL push (axial) and knob (endless rotation, ANIM_keyframe_loop 360) and the ECAM SD/EWD
  brightness knobs are wrapped in new ANIM blocks after `# CTRL ... MOTION` markers.
* The legacy EFIS baro knob blocks get the push (STD) / pull (QNH) shaft translation.

Idempotent: markers / retargeted names make a rerun a no-op. Every file is preflighted before
the first write; originals are backed up by content hash.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

from patch_center_right_animations import primitive_points, source_targets, wrap_geometry
from patch_efis_animations import vertex_payload
from build_cockpit_controls import load_buffer, local_matrix, matrix_multiply, transform_point
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

SIDES = (('CS', 'captain'), ('FO', 'first_officer'))
ISIS_UNITS = ((1, 'isis'), (2, 'isis2'))
# Legacy latched-state blocks -> momentary travel datarefs.
LEGACY_KEYS = {'cstr': 'cstr', 'wpt': 'wpt', 'ndb': 'ndb', 'arpt': 'arpt', 'vor1': 'vor1',
               'wx_requested': 'wx', 'terr': 'terr', 'traf': 'traf'}
NEW_EFIS_KEYS = (('VORD', 'vord'), ('ADF', 'vor2'), ('TAXI', 'taxi'), ('LS', 'ls'), ('VV', 'vv'))
ISIS_KEYS = (('MODE', 'mode'), ('LS_DIR', 'ls'), ('PLUS', 'plus'), ('MINUS', 'minus'), ('MENU', 'menu'))
BUTTON_MARKER = '# CTRL BUTTON MOTION '
AXIAL_MARKER = '# CTRL AXIAL MOTION '
ROTARY_MARKER = '# CTRL ROTARY MOTION '
BARO_MARKER = '# CTRL EFIS BARO AXIAL '
CREW_NORMAL = (0.0, 0.3328, -0.943)   # main-panel face normal toward the crew (19.4 deg back)
ISIS_DEGREES_PER_DETENT = 10          # FBW knob template COUNT = 36

BUTTON_MOTIONS = (
    *((f'PUSH_EFIS_{short}_{node}', f'PUSH_EFIS_{short}_{node}', f'efis/{side}/{key}_pressed', 1)
      for short, side in SIDES for node, key in NEW_EFIS_KEYS),
    *((f'PUSH_ISIS_{unit}_{node}', f'PUSH_ISIS_{unit}_{node}', f'{prefix}/{key}_pressed', 1)
      for unit, prefix in ISIS_UNITS for node, key in ISIS_KEYS),
)
AXIAL_MOTIONS = tuple((f'PUSH_ISIS_{unit}_SET_SEL', f'PUSH_ISIS_{unit}_SET_SEL', f'{prefix}/set_sel_axial', 1)
                      for unit, prefix in ISIS_UNITS)
ROTARY_MOTIONS = (
    *((f'KNOB_ISIS_{unit}_SET_SEL', f'KNOB_ISIS_{unit}_SET_SEL', f'{prefix}/set_sel_rotation', 360)
      for unit, prefix in ISIS_UNITS),
    ('KNOB_EFIS_SD', 'KNOB_EFIS_SD', 'ecam/sd/brightness', 1),
    ('KNOB_EFIS_EWD', 'KNOB_EFIS_EWD', 'ecam/sd/ewd_brightness', 1),
)
# Legacy-animated EFIS caps: the export left their SEQ2 legend decals static in world space.
LEGACY_LEGEND_MOTIONS = tuple(
    (f'PUSH_EFIS_{short}_{node}', f'PUSH_EFIS_{short}_{node}', f'efis/{side}/{key}_pressed', 1)
    for short, side in SIDES for node, key in (('CSTR', 'cstr'), ('WPT', 'wpt'), ('NDB', 'ndb'), ('ARPT', 'arpt'),
                                               ('VOR', 'vor1'), ('WX', 'wx'), ('TERR', 'terr'), ('TRAF', 'traf')))
# Legend decals that FBW did not parent under their button (ISIS 2 "-").
ORPHAN_LEGENDS = (('PUSH_ISIS_2_MINUS_SEQ1', 'PUSH_ISIS_2_MINUS'),)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def node_meshes(source, node_name):
    document = json.loads(source.read_text())
    buffers = [load_buffer(source, row['uri']) for row in document['buffers']]
    nodes = document['nodes']
    parents = {child: i for i, node in enumerate(nodes) for child in node.get('children', [])}
    index = next(i for i, node in enumerate(nodes) if node.get('name') == node_name)

    def world(i):
        local = local_matrix(nodes[i])
        return matrix_multiply(world(parents[i]), local) if i in parents else local

    rows = []
    for primitive in document['meshes'][nodes[index]['mesh']]['primitives']:
        points, count = primitive_points(document, buffers, primitive)
        points = [transform_point(world(index), p) for p in points]
        rows.append(dict(material=document['materials'][primitive['material']]['name'].lower(),
                         low=tuple(min(p[a] for p in points) for a in range(3)),
                         high=tuple(max(p[a] for p in points) for a in range(3)), count=count))
    return rows


def control_groups(source):
    buttons = source_targets(source, motions=BUTTON_MOTIONS)
    buttons += [target for target in source_targets(source, motions=LEGACY_LEGEND_MOTIONS)
                if '_SEQ' in target['name']]
    for legend, owner in ORPHAN_LEGENDS:
        motion = next(t for t in buttons if t['name'] == owner)
        for row in node_meshes(source, legend):
            low = tuple(v + d for v, d in zip(row['low'], (-.00004, -.00004, -.004)))
            high = tuple(v + d for v, d in zip(row['high'], (.00004, .0015, .00004)))
            buttons.append(dict(motion, name=legend, material=row['material'], low=low, high=high,
                                count=row['count'], points=None))
    # Knob shafts carry no decals: exact authored points keep the knob and its push cap apart.
    axial = source_targets(source, motions=AXIAL_MOTIONS, include_child_points=True)
    rotary = source_targets(source, motions=ROTARY_MOTIONS, include_child_points=True)
    for target in [*buttons, *axial]:
        if target['kind'] != 'translation' or target['values'] != [0, 1]:
            raise ValueError(f'{target["name"]}: expected an authored rest/pressed translation')
        travel = target['poses'][-1]
        if _dot(travel, CREW_NORMAL) >= 0:
            raise ValueError(f'{target["name"]}: authored travel does not go into the panel')
    for target in rotary:
        if target['kind'] != 'rotation':
            raise ValueError(f'{target["name"]}: expected an authored rotation')
        if 'set_sel_rotation' in target['dataref']:
            # Endless knob: dataref = degrees clockwise as seen from the seat. A positive angle
            # about an axis pointing toward the crew is counter-clockwise for the crew.
            sign = -1.0 if _dot(target['axis'], CREW_NORMAL) > 0 else 1.0
            target['values'], target['poses'] = [0, 360], [0.0, 360.0 * sign]
    return [(BUTTON_MARKER, buttons), (AXIAL_MARKER, axial), (ROTARY_MARKER, rotary)]


def baro_shaft(source):
    """World-space push travel of PUSH_KNOB_EFIS_<side>_BARO1 (parent of the baro knob)."""
    motions = (('PUSH_KNOB_EFIS_CS_BARO1', 'KNOB_EFIS_CS_BARO1', 'efis/captain/baro_knob_axial', 1),
               ('PUSH_EFIS_FO_BARO1', 'KNOB_EFIS_FO_BARO1', 'efis/first_officer/baro_knob_axial', 1))
    result = {}
    for target in source_targets(source, motions=motions):
        travel = target['poses'][-1]
        if target['kind'] != 'translation' or _dot(travel, CREW_NORMAL) >= 0:
            raise ValueError('EFIS baro shaft source must push into the panel')
        result[target['dataref'].split('/')[3]] = travel
    if set(result) != {'captain', 'first_officer'}:
        raise ValueError('Missing EFIS baro shaft source')
    return result


def _block_end(lines, begin):
    depth = 0
    for i in range(begin, len(lines)):
        word = lines[i].strip()
        if word == 'ANIM_begin':
            depth += 1
        elif word == 'ANIM_end':
            depth -= 1
            if not depth:
                return i
    raise ValueError('Unclosed ANIM block')


def _enclosing_begin(lines, at):
    depth = 0
    for i in range(at, -1, -1):
        word = lines[i].strip()
        if word == 'ANIM_end':
            depth += 1
        elif word == 'ANIM_begin':
            if not depth:
                return i
            depth -= 1
    raise ValueError('Animation outside ANIM_begin')


def retarget_legacy(text):
    """Latched EFIS key travel -> momentary travel, only in blocks that draw geometry."""
    lines = text.split('\n')
    changed = 0
    pattern = re.compile(r'^(\s*ANIM_trans_begin\s+a380x/xp12/efis/(captain|first_officer)/)(\w+)$')
    for i, line in enumerate(lines):
        match = pattern.match(line)
        if not match or match.group(3) not in LEGACY_KEYS:
            continue
        begin = _enclosing_begin(lines, i)
        if not any(lines[k].strip().startswith('TRIS') for k in range(begin, _block_end(lines, begin))):
            continue
        lines[i] = match.group(1) + LEGACY_KEYS[match.group(3)] + '_pressed'
        changed += 1
    return '\n'.join(lines), changed


def add_baro_shaft(text, shaft):
    lines = text.split('\n')
    added = 0
    for side, travel in shaft.items():
        ref = f'a380x/xp12/efis/{side}/baro_knob_axial'
        if ref in text:
            continue
        rotate = next((i for i, line in enumerate(lines)
                       if line.strip().startswith('ANIM_rotate_begin') and line.strip().endswith(f'efis/{side}/baro_preset_hpa')),
                      None)
        if rotate is None:
            continue
        begin = _enclosing_begin(lines, rotate)
        if any(not lines[k].strip().startswith(('ANIM_trans\t', 'ANIM_rotate\t')) for k in range(begin + 1, rotate)):
            raise ValueError('Unexpected legacy baro knob block layout')
        if not any(lines[k].strip().startswith('TRIS') for k in range(begin, _block_end(lines, begin))):
            continue
        if sum(1 for line in lines[:begin] if line.strip() == 'ANIM_begin') - \
                sum(1 for line in lines[:begin] if line.strip() == 'ANIM_end') != 0:
            raise ValueError('Legacy baro knob block is nested: world-space travel would be wrong')
        indent = lines[begin][:len(lines[begin]) - len(lines[begin].lstrip())]
        vec = lambda v: '\t'.join(f'{obj8_fmt(x)}' for x in v)
        block = [f'{indent}{BARO_MARKER}{side}', f'{indent}ANIM_trans_begin\t{ref}',
                 f'{indent}ANIM_trans_key\t-1\t{vec(-x for x in travel)}', f'{indent}ANIM_trans_key\t0\t0\t0\t0',
                 f'{indent}ANIM_trans_key\t1\t{vec(travel)}', f'{indent}ANIM_trans_end']
        lines[begin + 1:begin + 1] = block
        added += 1
    return '\n'.join(lines), added


def add_keyframe_loops(text):
    """Endless ISIS knobs: ANIM_keyframe_loop 360 as the last line of each set_sel_rotation key
    table (before ANIM_rotate_end, as Laminar's XPlane2Blender writes it). Also migrates a loop
    line that an earlier run of this tool placed after ANIM_rotate_end. Idempotent."""
    lines = text.split('\n')
    result, pending, i = [], False, 0
    while i < len(lines):
        line, word = lines[i], lines[i].strip()
        if word.startswith('ANIM_rotate_begin') and word.endswith('/set_sel_rotation'):
            pending = True
        elif pending and word.startswith('ANIM_keyframe_loop'):
            i += 1
            continue  # re-emitted below, in the canonical place
        elif pending and word == 'ANIM_rotate_end':
            result.append(line[:len(line) - len(line.lstrip())] + 'ANIM_keyframe_loop\t360')
            result.append(line)
            if i + 1 < len(lines) and lines[i + 1].strip().startswith('ANIM_keyframe_loop'):
                i += 1  # drop the legacy post-_end loop line
            pending = False
            i += 1
            continue
        result.append(line)
        i += 1
    return '\n'.join(result)


def patch_object(original, filename, groups, shaft):
    changed, legacy = retarget_legacy(original)
    changed, baro = add_baro_shaft(changed, shaft)
    meshes = []
    for marker, targets in groups:
        before = changed
        changed, applied, _ = wrap_geometry(changed, filename, targets, marker)
        meshes.extend(dict(row, group=marker.strip('# ')) for row in applied)
        if marker == ROTARY_MARKER:
            changed = add_keyframe_loops(changed)
    if vertex_payload(original) != vertex_payload(changed):
        raise ValueError(f'{filename}: control patch changed geometry, UV or texture payload')
    return changed, dict(legacy_retargeted=legacy, baro_shafts=baro, meshes=meshes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--backup', type=Path)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if not args.dry_run and args.backup is None:
        parser.error('--backup is required unless --dry-run is used')
    groups = control_groups(args.source)
    shaft = baro_shaft(args.source)
    staged = []
    for path in sorted(args.directory.glob('*.obj')):
        if path.name.startswith('._'):
            continue
        original = path.read_text()
        changed, summary = patch_object(original, path.name, groups, shaft)
        if changed != original:
            staged.append((path, original, changed, summary))
    expected = {t['name'] for _, targets in groups for t in targets}
    applied = {m['name'] for *_, summary in staged for m in summary['meshes']}
    first_run = any(summary['meshes'] for *_, summary in staged)
    if first_run and expected - applied:
        raise ValueError(f'Control meshes not found in the cockpit objects: {sorted(expected - applied)}')
    rows = []
    for path, original, changed, summary in staged:
        digest = hashlib.sha256(original.encode()).hexdigest()
        backup = args.backup / (digest[:16] + '-' + path.name) if args.backup else None
        if not args.dry_run:
            backup.parent.mkdir(parents=True, exist_ok=True)
            if not backup.exists():
                backup.write_text(original)
            path.write_text(changed)
        rows.append(dict(path=str(path), before_sha256=digest, backup=str(backup) if backup else None, **summary))
    report = dict(dry_run=args.dry_run, files=rows, already_applied=not first_run,
                  missing=sorted(expected - applied) if first_run else [],
                  targets=[{k: v for k, v in t.items() if k != 'points'} for _, targets in groups for t in targets],
                  baro_shaft=shaft)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(files=len(rows), already_applied=not first_run, meshes=sum(len(r['meshes']) for r in rows),
                          legacy=sum(r['legacy_retargeted'] for r in rows),
                          baro=sum(r['baro_shafts'] for r in rows), missing=report['missing'],
                          report=str(args.report))))


if __name__ == '__main__':
    main()
