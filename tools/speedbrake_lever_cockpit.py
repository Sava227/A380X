#!/usr/bin/env python3
"""Stage the animated A380 SPEED BRAKE lever (push down, 1/2 notch, ARM) and its slot brushes.

Pinned FBW sources (docs/SPEED_BRAKE.md):

* Cockpit glTF A380_COCKPIT_LOD00.gltf, clip ``lever_speed_brake`` (20 keys at 24 fps): LEVER_SPEEDBRAKE is
  13.7 mm up along the lever at key 0 (ARM, the node's rest pose) and down at keys 1..19; it holds the RET
  angle (13.06 deg about +X) through key 9 and rotates to FULL (-11.62 deg) at key 19, the 1/2 position at the
  angular midpoint, key 14. FBW behaviour (pedestal.xml, A32NX_Interior_Handling.xml
  FBW_HANDLING_Lever_Spoilers_Template): O:Position 0 ARM, 1 RET, 2 1/2, 3 FULL.
* The same clip moves a rig of 38 joints (SPD_LVR_CTR_BONE, SPD_LVR_LH/RH_BONE1..17, SPD_LVR_LH/RH_SIDE,
  NEUTRAL_BONE_1): the joints of skin 7, used by the two skinned 140-triangle meshes Plane.083 (right) and
  Plane.084 (left), the brush strips that line the lever slot. Their JOINTS_0 palette is one-based (as the
  audited FBW wing skins) and every vertex has a single joint (weight 1). The inner-edge joints move 2-5 mm
  outward/up as the lever stem passes, the joints at the RET position close behind it.

How they ended up in the X-Plane export: OBJ8 has no skinning. The export wrote LEVER_SPEEDBRAKE correctly
(node-local vertices, translation + rotation keys on X-Plane's speedbrake_ratio over the whole clip), wrote
the 26 brush joints as empty ANIM blocks (no geometry, left alone) and the two skinned meshes as static
triangles in their mesh space, which is SPEED_BRAKE_PANEL's frame (the rig parent SPD_LVR cancels that
node's translation): 280 triangles drawn 32 m from the cockpit near the object origin, none in the slot.

Staged changes (the input aircraft is never written):

* cockpit_a380_cockpit_pedestal_misc01_4k.obj: the LEVER_SPEEDBRAKE block is re-bound to
  a380x/xp12/speedbrake/lever_anim (handle 0 RET .. 1 FULL, the clip's keys 9..19) and lift_anim (+1 ARM =
  the clip's pull-up, 0 normal, -1 pushed down 5 mm, in the lever's own frame). The brush triangles are put
  into the slot (static SPEED_BRAKE_PANEL translation) and each one follows the mean of its three vertices'
  joint translations, keyed on lever_anim (a rigid approximation of the skin: at most 3.3 mm between
  neighbouring triangles, at the stem). VT/IDX/UV/texture payload, every other line (the thrust and flaps
  patches included) and the drawn index ranges stay identical (checked). Markers ``MOTION_MARKER``,
  ``BRUSH_MARKER``.
* a380x_cockpit_controls.obj: after ``MARKER``, one draw-disabled box around the handle, animated with the
  lever, carrying one X-Plane click-hold-drag ATTR_manip_drag_rotate about the lever pivot on
  a380x/xp12/speedbrake/lever_input: -0.25 (beyond RET toward ARM) .. 1 (FULL), keyframes at the clip's
  angles. The plugin (a380x_speedbrake.inc) models holding = pushing down, the 1/2 notch and the pull-up
  into ARM at RET. X-Plane's lift detents are not used: their lift is always away from the pivot (OBJ8
  specification), the opposite of the push-down this lever needs.

    python3 tools/speedbrake_lever_cockpit.py --aircraft output/A380X-XP12 --source <A380_COCKPIT_LOD00.gltf> \\
        --stage build/speedbrake-geometry-<timestamp>
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

from avionics_provider import SPEEDBRAKE_DATAREFS
from build_cockpit_controls import accessor_values, local_matrix, matrix_identity, matrix_multiply, transform_point
from cockpit_lever_geometry import Source, oriented_box, tris_coverage, vertex_payload
from isis_cockpit_device import _append
from patch_center_right_animations import transform_vector
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

PREFIX = 'a380x/xp12/speedbrake/'
MARKER = '# A380X SPEED BRAKE LEVER V1'
MOTION_MARKER = MARKER + ' MOTION '
BRUSH_MARKER = MARKER + ' BRUSH '
CLICK_MARKER = MARKER + ' CLICK '
CONTROLS = 'objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
PEDESTAL = 'objects/interior/cockpit/cockpit_a380_cockpit_pedestal_misc01_4k.obj'
MATERIAL = 'A380_COCKPIT_PEDESTAL_MISC01_4k'
CLIP = 'lever_speed_brake'
LEVER = 'LEVER_SPEEDBRAKE'
BRUSHES = ('Plane.083', 'Plane.084')   # right and left slot brush strips (skin 7)
PANEL = 'SPEED_BRAKE_PANEL'            # frame of the brush mesh positions
NATIVE_HANDLE = 'sim/cockpit2/controls/speedbrake_ratio'
RET_KEY, FULL_KEY = 9, 19              # clip keys of RET and FULL (1/2 = key 14)
PUSH_M = 0.005                         # push-down travel (not authored by FBW; see docs/SPEED_BRAKE.md)
ARM_ZONE = -0.25                       # lever_input range beyond RET toward ARM (a380x_speedbrake.h)
CLICK_MARGIN = 0.002                   # m around the handle
TOLERANCE = 1e-5                       # m, exported vertices against the glTF
TOOLTIP = ('SPEED BRAKE lever - hold (push down) and drag: RET / 1/2 / FULL; at RET drag forward '
           '(pull up) to ARM the ground spoilers')


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _num(value: float) -> str:
    return f'{obj8_fmt(0.0 if abs(value) < 1e-12 else value)}'


def _vec(values) -> str:
    return '\t'.join(_num(v) for v in values)


def _rotate_x(vector, degrees):
    a = math.radians(degrees)
    x, y, z = vector
    return (x, y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))


def _quat_x_angle(q) -> float:
    """Angle (deg) of a rotation about +X; raises if the quaternion has another axis."""
    x, y, z, w = q
    if abs(y) > 1e-6 or abs(z) > 1e-6:
        raise ValueError('lever_speed_brake rotation is not about X')
    return math.degrees(2.0 * math.atan2(x, w))


def _channels(source: Source, name: str) -> dict[str, list]:
    document = source.document
    animation = next(a for a in document['animations'] if a['name'] == CLIP)
    index = source.names[name]
    rows = {}
    for channel in animation['channels']:
        if channel['target']['node'] != index:
            continue
        sampler = animation['samplers'][channel['sampler']]
        times = [row[0] for row in accessor_values(document, source.buffers, sampler['input'])]
        values = accessor_values(document, source.buffers, sampler['output'])
        rows[channel['target']['path']] = (times, values)
    return rows


def lever_motion(source: Source) -> dict:
    """LEVER_SPEEDBRAKE: rest (ARM) / normal translation, RET..FULL angles at the eleven clip keys 9..19."""
    rows = _channels(source, LEVER)
    times, translations = rows['translation']
    rotation_times, quaternions = rows['rotation']
    if len(times) != FULL_KEY + 1 or rotation_times != times:
        raise ValueError(f'{CLIP}: expected {FULL_KEY + 1} keys on {LEVER}')
    rest = source.nodes[source.names[LEVER]]
    up, down = tuple(translations[0]), tuple(translations[1])
    if max(abs(a - b) for a, b in zip(up, rest['translation'])) > 1e-6:
        raise ValueError(f'{CLIP}: key 0 is not the rest (ARM) pose')
    if any(max(abs(a - b) for a, b in zip(t, down)) > 1e-6 for t in translations[1:]):
        raise ValueError(f'{CLIP}: the lever does not stay down from key 1')
    angles = [_quat_x_angle(q) for q in quaternions]
    rest_angle = _quat_x_angle(rest['rotation'])
    if any(abs(a - rest_angle) > 1e-4 for a in angles[:RET_KEY + 1]):
        raise ValueError(f'{CLIP}: the lever rotates before key {RET_KEY}')
    sweep = angles[RET_KEY:]
    if any(b >= a for a, b in zip(sweep, sweep[1:])):
        raise ValueError(f'{CLIP}: RET..FULL rotation is not monotonic')
    lift_world = tuple(a - b for a, b in zip(up, down))          # down -> up, at the RET angle
    lift_local = _rotate_x(lift_world, -rest_angle)              # in the lever's own frame
    length = math.sqrt(sum(v * v for v in lift_local))
    push_local = tuple(-PUSH_M / length * v for v in lift_local)
    return dict(node=LEVER, times=times, normal=down, arm=up, rest_angle=rest_angle,
                handle=[(k - RET_KEY) / (FULL_KEY - RET_KEY) for k in range(RET_KEY, FULL_KEY + 1)],
                angles=sweep, half_angle=angles[(RET_KEY + FULL_KEY) // 2], lift_world=lift_world,
                lift_local=lift_local, push_local=push_local, lift_length_m=length, push_m=PUSH_M)


def _lever_angle(motion: dict, handle: float) -> float:
    """Absolute angle (deg about +X) at a handle value, linear between keys, extrapolated below RET."""
    h, a = motion['handle'], motion['angles']
    if handle <= h[0]:
        return a[0] + (a[1] - a[0]) * (handle - h[0]) / (h[1] - h[0])
    for k in range(1, len(h)):
        if handle <= h[k]:
            return a[k - 1] + (a[k] - a[k - 1]) * (handle - h[k - 1]) / (h[k] - h[k - 1])
    return a[-1]


def brush_rig(source: Source) -> dict:
    """Per brush vertex (mesh = panel frame): joint, and the motion of each joint per clip key.

    Skin displacement of a joint at key k = translation of (joint world at k) x (its inverse bind matrix), a
    pure translation for this rig (checked). The mesh positions are the RET/ARM pose (each joint's vertices
    sit at its rest position, the strips already parted around the stem at joints 3-5), while the inverse
    bind matrices of joints 3-5 describe a closed brush; the motion used is therefore the displacement
    relative to RET, D(k) - D(RET): the brush is drawn as modelled at RET and moves as the clip moves it."""
    document = source.document
    panel = source.world(source.names[PANEL])
    offset = transform_point(panel, (0.0, 0.0, 0.0))
    if any(abs(panel[r][c] - (1.0 if r == c else 0.0)) > 1e-9 for r in range(3) for c in range(3)):
        raise ValueError(f'{PANEL} is not a pure translation')
    deltas, binds, strips = {}, {}, []
    for name in BRUSHES:
        node = document['nodes'][source.names[name]]
        skin = document['skins'][node['skin']]
        joints = [document['nodes'][j]['name'] for j in skin['joints']]
        ibm = [[[m[c * 4 + r] for c in range(4)] for r in range(4)]
               for m in accessor_values(document, source.buffers, skin['inverseBindMatrices'])]
        primitive = document['meshes'][node['mesh']]['primitives'][0]
        if document['materials'][primitive['material']]['name'].lower() != MATERIAL.lower():
            raise ValueError(f'{name}: unexpected material')
        positions = accessor_values(document, source.buffers, primitive['attributes']['POSITION'])
        palette = accessor_values(document, source.buffers, primitive['attributes']['JOINTS_0'])
        weights = accessor_values(document, source.buffers, primitive['attributes']['WEIGHTS_0'])
        indices = [int(r[0]) for r in accessor_values(document, source.buffers, primitive['indices'])]
        packed = primitive.get('extras', {}).get('ASOBO_primitive', {})
        start, base = packed.get('StartIndex', 0), packed.get('BaseVertexIndex', 0)
        count = packed.get('PrimitiveCount', len(indices) // 3) * 3
        used = sorted({base + i for i in indices[start:start + count]})
        vertices = []
        for v in used:
            lanes = [(int(j), w) for j, w in zip(palette[v], weights[v]) if w > 0.0]
            if not lanes or abs(sum(w for _, w in lanes) - 1.0) > 1e-3:
                raise ValueError(f'{name}: vertex {v} has no normalised joint weights')
            # One-based ASOBO palette (index 0 is never used; the highest index is the joint count).
            if any(j < 1 or j > len(joints) for j, _ in lanes):
                raise ValueError(f'{name}: joint palette is not one-based')
            vertices.append(dict(position=tuple(positions[v]), joints=[(joints[j - 1], w) for j, w in lanes]))
        strips.append(dict(name=name, vertices=vertices, triangles=count // 3))
        for joint in {j for row in vertices for j, _ in row['joints']}:
            if joint in deltas:
                continue
            index = source.names[joint]
            inverse_bind = ibm[joints.index(joint)]
            rows = _channels(source, joint)
            rest = dict(source.nodes[index])
            keys = []
            for k in range(FULL_KEY + 1):
                pose = dict(rest)
                for path in ('translation', 'rotation', 'scale'):
                    if path in rows:
                        values = rows[path][1]
                        if len(values) not in (2, FULL_KEY + 1):
                            raise ValueError(f'{joint}: unexpected key count')
                        pose[path] = values[k] if len(values) == FULL_KEY + 1 else values[0]
                parent = source.world(source.parents[index]) if index in source.parents else matrix_identity()
                skin_matrix = matrix_multiply(matrix_multiply(parent, local_matrix(pose)), inverse_bind)
                if any(abs(skin_matrix[r][c] - (1.0 if r == c else 0.0)) > 1e-4 for r in range(3) for c in range(3)):
                    raise ValueError(f'{joint}: the brush skin rotates (not a pure translation)')
                # Float noise of the export (< 1 micron) is no motion.
                keys.append(tuple(skin_matrix[r][3] for r in range(3)))
            deltas[joint] = [tuple(0.0 if abs(a - b) < 1e-6 else a - b for a, b in zip(key, keys[RET_KEY]))
                             for key in keys]
            binds[joint] = transform_point(source.world(index), (0.0, 0.0, 0.0))
    for joint, rows in deltas.items():
        if any(max(abs(a - b) for a, b in zip(rows[k], rows[RET_KEY])) > 1e-6 for k in range(RET_KEY)):
            raise ValueError(f'{joint}: the brush moves between ARM and RET')
    # The mesh frame: each joint's vertices lie at its rest position once the panel offset is applied.
    check = []
    for strip in strips:
        groups = defaultdict(list)
        for row in strip['vertices']:
            if len(row['joints']) == 1:
                groups[row['joints'][0][0]].append(row['position'])
        for joint, points in groups.items():
            if not joint.startswith('SPD_LVR_') or 'BONE' not in joint or 'CTR' in joint:
                continue
            centre = [sum(p[a] for p in points) / len(points) + offset[a] for a in range(3)]
            check.append(max(abs(a - b) for a, b in zip(centre, binds[joint])))
    if not check or max(check) > 0.003:
        raise ValueError(f'Brush vertices are not at their joints after the {PANEL} offset: {max(check or [1])}')
    return dict(offset=offset, deltas=deltas, strips=strips, joint_distance_max_m=max(check))


def motions(source: Source) -> dict:
    return dict(lever=lever_motion(source), brush=brush_rig(source))


# ---------------------------------------------------------------- pedestal OBJ
def _block_end(lines: list[str], begin: int) -> int:
    depth = 0
    for k in range(begin, len(lines)):
        word = lines[k].split()[:1]
        depth += word == ['ANIM_begin']
        depth -= word == ['ANIM_end']
        if depth == 0:
            return k
    raise ValueError('Unclosed animation block')


def _lever_block(lines: list[str]) -> tuple[int, int]:
    hits = []
    for k, line in enumerate(lines):
        if line.split()[:1] == ['ANIM_trans_begin'] and line.split()[-1] == NATIVE_HANDLE and k and \
                lines[k - 1].strip() == 'ANIM_begin':
            end = _block_end(lines, k - 1)
            if any(lines[n].split()[:1] == ['TRIS'] for n in range(k, end)):
                hits.append((k - 1, end))
    if len(hits) != 1:
        raise ValueError(f'{PEDESTAL}: expected one exported speed brake lever block with geometry, found {len(hits)}')
    return hits[0]


def _lever_lines(motion: dict, indent: str, tris: str) -> list[str]:
    pivot = motion['normal']
    rows = [MOTION_MARKER + LEVER, 'ANIM_begin', f'ANIM_trans\t{_vec(pivot)}\t{_vec(pivot)}',
            f'ANIM_rotate_begin\t1\t0\t0\t{PREFIX}lever_anim']
    rows += [f'ANIM_rotate_key\t{_num(h)}\t{_num(a)}' for h, a in zip(motion['handle'], motion['angles'])]
    rows += ['ANIM_rotate_end', f'ANIM_trans_begin\t{PREFIX}lift_anim',
             f'ANIM_trans_key\t-1\t{_vec(motion["push_local"])}', 'ANIM_trans_key\t0\t0\t0\t0',
             f'ANIM_trans_key\t1\t{_vec(motion["lift_local"])}', 'ANIM_trans_end', tris.strip(), 'ANIM_end']
    return [indent + row for row in rows]


def _grid(points):
    grid = defaultdict(list)
    for k, p in enumerate(points):
        grid[tuple(round(v / TOLERANCE) for v in p)].append(k)
    return grid


def _find(grid, points, p):
    key = tuple(round(v / TOLERANCE) for v in p)
    return [k for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
            for k in grid.get((key[0] + dx, key[1] + dy, key[2] + dz), ())
            if max(abs(a - b) for a, b in zip(points[k], p)) <= TOLERANCE]


def _brush_motion(rig: dict, joints_of_triangle) -> tuple:
    """Per-key translation (handle keys 0..10 = clip keys 9..19) of a triangle: the mean of its vertices."""
    keys = []
    for k in range(RET_KEY, FULL_KEY + 1):
        total = [0.0, 0.0, 0.0]
        for joints in joints_of_triangle:
            for joint, weight in joints:
                d = rig['deltas'][joint][k]
                total = [t + weight * c / 3.0 for t, c in zip(total, d)]
        keys.append(tuple(round(v, 9) for v in total))
    return tuple(keys)


def rebind_pedestal(text: str, source: Source, motion: dict) -> tuple[str, dict]:
    """Re-bind the lever block and put the brush strips into the slot, animated (idempotent)."""
    if MOTION_MARKER in text:
        if text.count(MOTION_MARKER + LEVER) != 1 or BRUSH_MARKER not in text or PREFIX + 'lift_anim' not in text:
            raise ValueError(f'{PEDESTAL}: incomplete previous speed brake patch')
        return text, {}
    if PREFIX in text or BRUSH_MARKER in text:
        raise ValueError(f'{PEDESTAL}: speed brake datarefs already present without the marker')
    ending = '\r\n' if '\r\n' in text else '\n'
    lines = text.split(ending)
    lever, rig = motion['lever'], motion['brush']
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith('VT\t')]
    indices = [int(v) for line in lines if line.startswith(('IDX\t', 'IDX10\t')) for v in line.split()[1:]]
    # --- the lever block: keys = the clip, geometry = the lever's own (node-local) vertices
    begin, end = _lever_block(lines)
    body = [lines[n].split() for n in range(begin + 1, end)]
    trans = [row for row in body if row[:1] == ['ANIM_trans_key']]
    rots = [row for row in body if row[:1] == ['ANIM_rotate_key']]
    if len(trans) != FULL_KEY + 1 or len(rots) != FULL_KEY + 1:
        raise ValueError('Exported lever keys are not the 20 clip keys')
    for k, row in enumerate(trans):
        if max(abs(float(a) - b) for a, b in zip(row[2:5], lever['arm'] if k == 0 else lever['normal'])) > 1e-4:
            raise ValueError('Exported lever translation keys disagree with the glTF clip')
    clip_angles = [lever['rest_angle']] * RET_KEY + lever['angles']
    if max(abs(float(row[2]) - a) for row, a in zip(rots, clip_angles)) > 1e-3:
        raise ValueError('Exported lever rotation keys disagree with the glTF clip')
    draws = [n for n in range(begin, end) if lines[n].split()[:1] == ['TRIS']]
    if len(draws) != 1:
        raise ValueError('The exported lever block draws more than the lever')
    start, count = map(int, lines[draws[0]].split()[1:3])
    local = source.local_points(LEVER)
    grid = _grid(local)
    if any(not _find(grid, local, vertices[i]) for i in indices[start:start + count]):
        raise ValueError('Exported lever vertices are not the glTF lever (node-local)')
    indent = lines[begin][:len(lines[begin]) - len(lines[begin].lstrip())]
    lever_rows = _lever_lines(lever, indent, lines[draws[0]])
    # --- the brushes: OBJ vertex -> (strip, joints) by position (mesh space = the exported positions)
    owners = {}
    for s, strip in enumerate(rig['strips']):
        points = [row['position'] for row in strip['vertices']]
        g = _grid(points)
        for i in set(indices):
            hits = _find(g, points, vertices[i])
            if not hits:
                continue
            joints = {tuple(strip['vertices'][h]['joints']) for h in hits}
            if len(joints) != 1 or i in owners:
                raise ValueError(f'Ambiguous brush vertex {i}')
            owners[i] = (s, list(joints)[0])
    groups, rows = {}, []
    out, brush_triangles, k, depth = [], 0, 0, 0
    while k < len(lines):
        if k == begin:   # balanced: the depth after it is the depth before it
            out.extend(lever_rows)
            k = end + 1
            continue
        line = lines[k]
        words = line.split()
        depth += (words[:1] == ['ANIM_begin']) - (words[:1] == ['ANIM_end'])
        if words[:1] == ['TRIS']:
            first, n = map(int, words[1:3])
            tri = [tuple(indices[p:p + 3]) for p in range(first, first + n, 3)]
            inside = [all(i in owners for i in t) for t in tri]
            if any(inside):
                if not all(inside):
                    raise ValueError(f'TRIS {first} {n} mixes brush and other geometry')
                if depth:
                    raise ValueError(f'Brush TRIS {first} {n} is already animated')
                ind = line[:len(line) - len(line.lstrip())]
                runs = []
                for p, t in zip(range(first, first + n, 3), tri):
                    strip = owners[t[0]][0]
                    if any(owners[i][0] != strip for i in t):
                        raise ValueError('A brush triangle spans both strips')
                    key = (strip, _brush_motion(rig, [owners[i][1] for i in t]))
                    if runs and runs[-1][0] == key:
                        runs[-1][2] += 3
                    else:
                        runs.append([key, p, 3])
                for key, p, c in runs:
                    gid = groups.setdefault(key, len(groups))
                    moving = any(any(abs(v) > 1e-9 for v in d) for d in key[1])
                    block = [f'{BRUSH_MARKER}{rig["strips"][key[0]]["name"]} {gid}', 'ANIM_begin',
                             f'ANIM_trans\t{_vec(rig["offset"])}\t{_vec(rig["offset"])}']
                    if moving:
                        block.append(f'ANIM_trans_begin\t{PREFIX}lever_anim')
                        block += [f'ANIM_trans_key\t{_num((m) / (FULL_KEY - RET_KEY))}\t{_vec(d)}'
                                  for m, d in enumerate(key[1])]
                        block.append('ANIM_trans_end')
                    block += [f'TRIS\t{p}\t{c}', 'ANIM_end']
                    out.extend(ind + row for row in block)
                    brush_triangles += c // 3
                k += 1
                continue
        out.append(line)
        k += 1
    expected = sum(strip['triangles'] for strip in rig['strips'])
    if brush_triangles != expected:
        raise ValueError(f'Found {brush_triangles} brush triangles, expected {expected}')
    result = ending.join(out)
    if vertex_payload(text) != vertex_payload(result):
        raise ValueError('visible VT/IDX/UV/texture payload changed')
    if tris_coverage(text) != tris_coverage(result):
        raise ValueError('drawn index ranges changed')
    keep = ('TRIS', 'ANIM_', MARKER)
    if [l for l in lines if not l.lstrip().startswith(keep)] != [l for l in out if not l.lstrip().startswith(keep)]:
        raise ValueError('lines other than the lever block and the brush TRIS changed')
    if lines[:begin] != out[:begin]:
        raise ValueError('lines before the lever block changed')
    moving_groups = sum(any(any(abs(v) > 1e-9 for v in d) for d in key[1]) for key in groups)
    gaps = []
    for key in groups:
        gaps.append(max(math.sqrt(sum(v * v for v in d)) for d in key[1]))
    report = dict(lever=dict(tris=[start, count], pivot=list(lever['normal']), rest_angle_deg=lever['rest_angle'],
                             half_angle_deg=lever['half_angle'], full_angle_deg=lever['angles'][-1],
                             lift_local=list(lever['lift_local']), push_local=list(lever['push_local'])),
                  brushes=dict(triangles=brush_triangles, groups=len(groups), moving_groups=moving_groups,
                               blocks=sum(1 for row in out if row.lstrip().startswith(BRUSH_MARKER)),
                               offset=list(rig['offset']), max_translation_m=max(gaps),
                               joint_distance_max_m=rig['joint_distance_max_m']))
    return result, report


# ---------------------------------------------------------------- controls OBJ
def manipulator(lever: dict) -> list[str]:
    """Click-hold-drag about the lever pivot: lever_input -0.25..1, angles relative to RET about -X."""
    def angle(h):
        return lever['rest_angle'] - _lever_angle(lever, h)
    pivot = lever['normal']
    rows = ['\t'.join(['ATTR_manip_drag_rotate', 'hand', _vec(pivot), _vec((-1.0, 0.0, 0.0)), _num(angle(ARM_ZONE)),
                       _num(angle(1.0)), '0', _num(ARM_ZONE), '1', '0', '0', PREFIX + 'lever_input', 'none', TOOLTIP])]
    rows += [f'ATTR_manip_keyframe\t{_num(h)}\t{_num(angle(h))}' for h in lever['handle'][:-1]]
    return rows


def click_geometry(source: Source, lever: dict):
    index = source.names[LEVER]
    matrix = [list(row) for row in source.world(index)]
    for axis in range(3):   # the rest pose is ARM: the box is built at the normal (RET) height
        matrix[axis][3] -= lever['lift_world'][axis]
    box_vertices, box_indices, bounds = oriented_box(source.points_in_frame((LEVER,)), matrix, margin=CLICK_MARGIN)
    pivot = lever['normal']
    inverse = [-v for v in pivot]
    chain = ['ANIM_begin', f'ANIM_trans\t{_vec(pivot)}\t{_vec(pivot)}', f'ANIM_rotate_begin\t-1\t0\t0\t{PREFIX}lever_anim']
    chain += [f'ANIM_rotate_key\t{_num(h)}\t{_num(lever["rest_angle"] - a)}'
              for h, a in zip(lever['handle'], lever['angles'])]
    chain += ['ANIM_rotate_end', f'ANIM_trans\t{_vec(inverse)}\t{_vec(inverse)}',
              f'ANIM_trans_begin\t{PREFIX}lift_anim',
              f'ANIM_trans_key\t-1\t{_vec(_rotate_x(lever["push_local"], lever["rest_angle"]))}',
              'ANIM_trans_key\t0\t0\t0\t0', f'ANIM_trans_key\t1\t{_vec(lever["lift_world"])}', 'ANIM_trans_end']
    manip = manipulator(lever)
    commands = ['ATTR_manip_none', 'ATTR_no_cockpit', 'ATTR_draw_disable', CLICK_MARKER + 'lever', *chain, *manip,
                f'TRIS\t0\t{len(box_indices)}', 'ATTR_manip_none', 'ANIM_end', 'ATTR_draw_enable']
    row = dict(name='lever', node=LEVER, target=PREFIX + 'lever_input', range=[ARM_ZONE, 1.0],
               manipulator='ATTR_manip_drag_rotate', keyframes=len(manip) - 1, triangles=len(box_indices) // 3,
               **bounds)
    return box_vertices, box_indices, commands, row


def augment_controls(text: str, source: Source, motion: dict) -> tuple[str, list[dict]]:
    if MARKER in text:
        if CLICK_MARKER + 'lever' not in text:
            raise ValueError('Incomplete previous speed brake controls patch')
        return text, []
    if PREFIX in text:
        raise ValueError('Speed brake datarefs already in the controls object without the marker')
    vertices, indices, commands, row = click_geometry(source, motion['lever'])
    return _append(text, vertices, indices, commands, MARKER), [row]


def stage(aircraft: Path, source_path: Path, destination: Path) -> dict:
    if destination.exists():
        raise ValueError('Stage directory must not exist')
    source = Source(source_path)
    motion = motions(source)
    used = {PREFIX + n for n in ('lever_anim', 'lift_anim', 'lever_input')}
    if not used <= set(SPEEDBRAKE_DATAREFS):
        raise ValueError(f'Datarefs outside the native contract: {sorted(used - set(SPEEDBRAKE_DATAREFS))}')
    staged = {}
    staged[PEDESTAL], pedestal = rebind_pedestal((aircraft / PEDESTAL).read_text(), source, motion)
    staged[CONTROLS], clicks = augment_controls((aircraft / CONTROLS).read_text(), source, motion)
    destination.mkdir(parents=True)
    files = []
    for relative, text in staged.items():
        original = aircraft / relative
        target = destination / 'aircraft' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(text.encode('utf-8'))
        files.append(dict(root='aircraft', path=relative, before_sha256=sha(original), after_sha256=sha(target),
                          changed=sha(original) != sha(target)))
    lever = motion['lever']
    report = dict(format=1, aircraft=str(aircraft), source=str(source_path), source_sha256=sha(source_path),
                  marker=MARKER, files=files,
                  lever=dict(clip=CLIP, keys=len(lever['times']), ret_key=RET_KEY, full_key=FULL_KEY,
                             handle=lever['handle'], angles_deg=lever['angles'], rest_angle_deg=lever['rest_angle'],
                             arm_lift_m=lever['lift_length_m'], push_m=lever['push_m']),
                  pedestal=pedestal, click_targets=clicks, datarefs_used=sorted(used),
                  visible_vertex_index_uv_payload_preserved=True, simulator_tested=False)
    (destination / 'staging-report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--aircraft', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    report = stage(args.aircraft.resolve(), args.source.resolve(), args.stage.resolve())
    print(json.dumps({'stage': str(args.stage.resolve()),
                      'files': [{k: f[k] for k in ('path', 'changed', 'before_sha256', 'after_sha256')}
                                for f in report['files']],
                      'pedestal': report['pedestal']}, indent=2))


if __name__ == '__main__':
    main()
