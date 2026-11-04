#!/usr/bin/env python3
"""Stage the animated A380 FLAPS lever, the FBW flap/slat exterior bindings and the ACF slat/flap schedule.

Pinned FBW sources (docs/FLAPS_LEVER.md):

* Cockpit glTF A380_COCKPIT_LOD00.gltf: clip ``flaps_lever`` rotates LEVER_FLAPS (and its children
  LEVER_FLAPS_LOCK_Cylinder.001 and LOCK_FLAPS) through 5 keys, 0 / 13.07 / 26.21 / 40.80 / 53.44 deg
  about -X = lever 0 / 1 / 2 / 3 / FULL (FBW anim code ``L:A32NX_FLAPS_HANDLE_PERCENT * 100``, index / 4);
  clip ``LOCK_FLAPS`` translates the pull-out sleeve 7.8 mm along the lever (FBW ``lock_flaps`` while the
  pilot holds the lever).
* Exterior A380_WINGS.xml: ``[lr]_flap_percent_key`` / ``[lr]_slat_percent_key`` are keyed on
  ``L:A32NX_*_FLAPS/SLATS_ANIMATION_POSITION`` = FPPU synchro angle / max (338.99 / 327.39 deg), not on the
  surface angle (flaps translate before they rotate: FPPU 0..122.2 deg is 0 deg of surface).

Staged changes (the input aircraft and project are never written):

* cockpit_a380_cockpit_pedestal_misc01_4k.obj: the Blender export already animates LEVER_FLAPS,
  LEVER_FLAPS_LOCK_Cylinder.001 and LOCK_FLAPS (pivot-local vertices posed at clip angle 26.17 deg) on
  X-Plane's sim/cockpit2/controls/flap_handle_request_ratio, which now carries the SFCC configuration, not
  the lever. That one block is re-bound to a380x/xp12/flaps/lever_anim (lever units 0..4, the same five
  angles) and the LOCK_FLAPS range gets the nested sleeve translation (lock_anim 0..1, the clip vector
  rotated into the exported pose). Every triangle range is identified against the glTF (1e-5 m), and the
  VT/IDX/UV/texture payload, every other line (the thrust-lever wraps included) and the drawn index ranges
  stay identical (checked). The empty per-material copies of that block in other cockpit OBJs draw nothing
  and are left alone.
* a380x_cockpit_controls.obj: one draw-disabled box around the lever handle, animated with the lever,
  carrying X-Plane's lift-detent click-hold-drag: ``ATTR_manip_drag_rotate`` about the lever's own pivot
  and axis on lever_input (0..4, keyframes at the authored 1 / 2 / 3 angles) with a lift of the sleeve
  travel on lift_input (0..1) and ``ATTR_axis_detent_range`` pits at 0..4 (the lever must be pulled out of
  a pit before it moves, as Laminar's A330 flap lever). Gates, no-intermediate positions and the pull-out
  rule are also enforced natively (a380x_flaps.h).
* The six exterior wing OBJs: only the animation-header tokens sim/flightmodel2/controls/flap1_deploy_ratio
  and slat1_deploy_ratio change, to a380x/xp12/flaps/flap_anim_ratio and slat_anim_ratio (FBW FPPU ratios
  published by the plugin); keyframes, geometry and every other line are unchanged (checked).
* A380X-XP12.acf (aircraft) and acf/A380X-XP12-current.acf (project snapshot): the flap/slat detent schedule
  becomes FBW's seven SFCC configurations (ACF_CHANGES), so the plugin selects X-Plane detent = FBW
  FlapsConf index.

    python3 tools/flaps_lever_cockpit.py --aircraft output/A380X-XP12 --source <A380_COCKPIT_LOD00.gltf> \\
        --project-acf acf/A380X-XP12-current.acf --stage build/flaps-geometry-<timestamp>
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

from avionics_provider import FLAPS_DATAREFS
from cockpit_lever_geometry import (Source, clip_motion, describe, drag_rotate_lines, nested_block, oriented_box,
                                    part_points, tris_coverage, vertex_payload)
from isis_cockpit_device import _append
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

PREFIX = 'a380x/xp12/flaps/'
MARKER = '# A380X FLAPS LEVER V1'
MOTION_MARKER = MARKER + ' MOTION '
CLICK_MARKER = MARKER + ' CLICK '
CONTROLS = 'objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
PEDESTAL = 'objects/interior/cockpit/cockpit_a380_cockpit_pedestal_misc01_4k.obj'
PEDESTAL_MATERIAL = 'A380_COCKPIT_PEDESTAL_MISC01_4k'
ACF = 'A380X-XP12.acf'
PROJECT_ACF = 'acf/A380X-XP12-current.acf'
WINGS = tuple(f'objects/exterior/wings/a380x_{name}.obj'
              for name in ('pylons', 'wing1', 'wing1_2', 'wing2', 'wing2_2', 'wingfence'))
EXTERIOR_REBIND = {
    'sim/flightmodel2/controls/flap1_deploy_ratio': PREFIX + 'flap_anim_ratio',
    'sim/flightmodel2/controls/slat1_deploy_ratio': PREFIX + 'slat_anim_ratio',
}
CLICK_MARGIN = 0.002  # m around the handle
LEVER_POSITIONS = 4.0  # lever units: 0, 1, 2, 3, FULL = 0..4
# Manipulator lever value -> lever_anim value (identical units; the clip keys sit at 0..4).
LEVER_TO_ANIM = tuple((float(k), float(k)) for k in range(5))
# X-Plane lift-detent pits (height 0) at every lever position; travel between them needs full lift.
DETENT_RANGES = tuple(row for k in range(5) for row in
                      (((float(k), float(k), 0.0),) + (((float(k), float(k + 1), 1.0),) if k < 4 else ())))
TOOLTIP = 'FLAPS lever - pull the handle out of its detent, then drag: 0 / 1 / 2 / 3 / FULL (gates at 1 and 3)'

# FBW a380_systems hydraulic/mod.rs (FLAP/SLAT_FPPU_TO_SURFACE_ANGLE_*) + sfcc flaps_/slats_channel.rs:
# X-Plane detent k = FBW FlapsConf k (0 CONF0, 1 CONF1, 2 1+F, 3 CONF2, 4 2S, 5 CONF3, 6 FULL).
FLAP_DEG = (0.0, 0.0, 8.0, 17.0, 17.0, 26.0, 33.0)
SLAT_DEG = (0.0, 20.0, 20.0, 20.0, 23.0, 23.0, 23.0)
SLAT_MAX_DEG = 23.0
# Full-travel times at full pressure: flaps 338.99 FPPU deg at 0.07 rad/s x 314.98/140, slats 327.39 at 0.08.
FLAP_TIME_S = 338.99 / (0.07 * 180.0 / math.pi * 314.98 / 140.0)
SLAT_TIME_S = 327.39 / (0.08 * 180.0 / math.pi * 314.98 / 140.0)


def _acf_number(value: float) -> str:
    return f'{value:.9f}'


def acf_changes() -> dict[str, str]:
    changes = {'acf/_flap_detents': '6'}
    for table in ('_flap1_dn', '_flap2_dn'):
        for k, deg in enumerate(FLAP_DEG):
            changes[f'acf/{table}/{k}'] = _acf_number(deg)
    for table in ('_slat1_dn', '_slat2_dn'):
        for k, deg in enumerate(SLAT_DEG):
            changes[f'acf/{table}/{k}'] = _acf_number(deg / SLAT_MAX_DEG)
    changes.update({
        'acf/_slat1_dn_max_deg': _acf_number(SLAT_MAX_DEG), 'acf/_slat2_dn_max_deg': _acf_number(SLAT_MAX_DEG),
        'acf/_flap_ext_time': _acf_number(FLAP_TIME_S), 'acf/_flap_ret_time': _acf_number(FLAP_TIME_S),
        'acf/_slat_ext1_time': _acf_number(SLAT_TIME_S), 'acf/_slat_ext2_time': _acf_number(SLAT_TIME_S),
        'acf/_slat_ret1_time': _acf_number(SLAT_TIME_S), 'acf/_slat_ret2_time': _acf_number(SLAT_TIME_S),
        # No X-Plane auto-slats / extension veto: the SFCC (plugin) owns relief, alpha lock and baulk.
        'acf/_slat_with_stall_EQ': '0', 'acf/_flap_extend_protect': '0', 'acf/_flap_retract_protect': '0',
        # FBW PerformanceConstants VfeF1 / VfeFF (X-Plane's own overspeed reference).
        'acf/_Vfe1_kts': _acf_number(263.0), 'acf/_Vfem_kts': _acf_number(182.0),
    })
    return changes


ACF_CHANGES = acf_changes()


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def patch_acf(text: str) -> tuple[str, list[dict]]:
    """Replace exactly the ACF_CHANGES property values (each key must exist once); idempotent."""
    rows, lines = [], text.split('\n')
    index = {}
    for k, line in enumerate(lines):
        if line.startswith('P '):
            key = line.split(' ', 2)[1]
            if key in ACF_CHANGES:
                if key in index:
                    raise ValueError(f'Duplicate ACF property {key}')
                index[key] = k
    missing = sorted(set(ACF_CHANGES) - set(index))
    if missing:
        raise ValueError(f'ACF lacks properties: {missing}')
    for key, value in ACF_CHANGES.items():
        k = index[key]
        before = lines[k].split(' ', 2)[2] if lines[k].count(' ') >= 2 else ''
        lines[k] = f'P {key} {value}'
        rows.append(dict(key=key, before=before, after=value, changed=before != value))
    return '\n'.join(lines), rows


def motions(source: Source) -> dict[str, dict]:
    lever = clip_motion(source, 'flaps_lever', 'LEVER_FLAPS', PREFIX + 'lever_anim', LEVER_POSITIONS)
    lock = clip_motion(source, 'LOCK_FLAPS', 'LOCK_FLAPS', PREFIX + 'lock_anim', 1.0)
    if lever['kind'] != 'rotation' or len(lever['values']) != 5:
        raise ValueError('flaps_lever must be a 5-key rotation (0 / 1 / 2 / 3 / FULL)')
    if any(abs(v - k) > 1e-6 for k, v in enumerate(lever['values'])):
        raise ValueError('flaps_lever keys are not equally spaced in time')
    if lock['kind'] != 'translation' or len(lock['values']) != 2:
        raise ValueError('LOCK_FLAPS must be a two-key translation')
    # Exact lever units (the clip times carry float noise of ~1e-7).
    lever['values'] = [float(k) for k in range(5)]
    lock['values'] = [0.0, 1.0]
    return dict(lever=lever, lock=lock)


def lift_length(motion: dict[str, dict]) -> float:
    return math.sqrt(sum(v * v for v in motion['lock']['poses'][-1]))


PARTS = (
    ('LEVER_FLAPS', False),
    ('LEVER_FLAPS_LOCK_Cylinder.001', False),
    ('LOCK_FLAPS', True),           # the pull-out sleeve: nested lock translation
)
CLICK_NODES = ('LEVER_FLAPS', 'LOCK_FLAPS')
NATIVE_HANDLE = 'sim/cockpit2/controls/flap_handle_request_ratio'
POSE_TOLERANCE = 1e-5  # m, exported pivot-local vertices against the glTF


def _rotate_x(vector, degrees):
    a = math.radians(degrees)
    x, y, z = vector
    return (x, y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))


def _lever_block(lines: list[str]) -> tuple[int, int]:
    """(ANIM_begin, ANIM_end) line indices of the one exported lever block on the native handle."""
    hits = [k for k, line in enumerate(lines) if line.split()[:1] == ['ANIM_rotate_begin']
            and line.split()[-1] == NATIVE_HANDLE]
    if len(hits) != 1:
        raise ValueError(f'{PEDESTAL}: expected one exported flaps-lever animation, found {len(hits)}')
    begin = hits[0] - 2
    if lines[begin].strip() != 'ANIM_begin' or lines[begin + 1].split()[:1] != ['ANIM_trans']:
        raise ValueError(f'{PEDESTAL}: unexpected flaps-lever animation layout')
    depth, k = 0, begin
    while True:
        word = lines[k].split()[:1]
        depth += word == ['ANIM_begin']
        depth -= word == ['ANIM_end']
        if depth == 0:
            return begin, k
        k += 1


def rebind_pedestal(text: str, source: Source, motion: dict[str, dict]) -> tuple[str, list[dict]]:
    """Re-bind the exported flaps-lever block to lever_anim and nest the lock translation (idempotent)."""
    if MOTION_MARKER in text:
        if NATIVE_HANDLE in text or text.count(MOTION_MARKER) != 1 or motion['lock']['dataref'] not in text:
            raise ValueError(f'{PEDESTAL}: incomplete previous flaps-lever patch')
        return text, []
    if PREFIX in text:
        raise ValueError(f'{PEDESTAL}: flaps-lever datarefs already present without the marker')
    ending = '\r\n' if '\r\n' in text else '\n'
    lines = text.split(ending)
    begin, end = _lever_block(lines)
    indent = lines[begin][:len(lines[begin]) - len(lines[begin].lstrip())]
    pivot_words = lines[begin + 1].split()[1:]
    pivot = tuple(map(float, pivot_words[:3]))
    lever = motion['lever']
    if max(abs(a - b) for a, b in zip(pivot, lever['pivot'])) > POSE_TOLERANCE:
        raise ValueError('Exported lever pivot differs from the glTF pivot')
    head = lines[begin + 2].split()
    if [float(v) for v in head[1:4]] != [1.0, 0.0, 0.0]:
        raise ValueError('Exported lever axis is not +X')
    keys, k = [], begin + 3
    while lines[k].split()[:1] == ['ANIM_rotate_key']:
        keys.append(tuple(map(float, lines[k].split()[1:3])))
        k += 1
    if lines[k].split()[:1] != ['ANIM_rotate_end'] or [v for v, _ in keys] != [0.0, 0.25, 0.5, 0.75, 1.0]:
        raise ValueError('Exported lever keys are not the five handle positions')
    # Exported pose (deg about -X from the clip start): every key + clip pose must agree.
    exported = [angle + pose for (_, angle), pose in zip(keys, lever['poses'])]
    if max(exported) - min(exported) > 1e-3:
        raise ValueError(f'Exported lever keys disagree with the glTF clip: {exported}')
    pose = sum(exported) / len(exported)
    draws = [(k, *map(int, lines[k].split()[1:3])) for k in range(k + 1, end) if lines[k].split()[:1] == ['TRIS']]
    if any(lines[k].split()[:1] not in (['TRIS'],) for k in range(k + 1, end)):
        raise ValueError('Unexpected lines inside the exported lever block')
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith('VT\t')]
    indices = [int(v) for line in lines if line.startswith(('IDX\t', 'IDX10\t')) for v in line.split()[1:]]
    rows, lock_line = [], None
    for node, is_lock in PARTS:
        points, count = part_points(source, node, PEDESTAL_MATERIAL)
        local = [_rotate_x(tuple(p - q for p, q in zip(point, pivot)), -pose) for point in points]
        grid = {}
        for p in local:
            grid.setdefault(tuple(round(v / POSE_TOLERANCE) for v in p), True)

        def near(p):
            key = tuple(round(v / POSE_TOLERANCE) for v in p)
            return any((key[0] + a, key[1] + b, key[2] + c) in grid
                       for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1))
        found = [row for row in draws if row[2] == count
                 and all(near(vertices[i]) for i in indices[row[1]:row[1] + row[2]])]
        if len(found) != 1:
            raise ValueError(f'{node}: {len(found)} exported triangle ranges match the glTF part')
        rows.append(dict(name=node, tris=list(found[0][1:]), indices=count, lock=is_lock))
        if is_lock:
            lock_line = found[0][0]
    if sorted(r['tris'][0] for r in rows) != sorted(d[1] for d in draws):
        raise ValueError('The exported lever block draws something other than the three lever parts')
    lock_vector = _rotate_x(motion['lock']['poses'][-1], -pose)
    patched = list(lines)
    patched[begin + 2] = indent + '\t'.join(['ANIM_rotate_begin', *head[1:4], lever['dataref']])
    for n, (_, angle) in enumerate(keys):
        original = lines[begin + 3 + n]
        patched[begin + 3 + n] = original[:len(original) - len(original.lstrip())] + f'ANIM_rotate_key\t{n}\t' + \
            original.split()[2]
    tris = lines[lock_line]
    tris_indent = tris[:len(tris) - len(tris.lstrip())]
    patched[lock_line:lock_line + 1] = [
        tris_indent + MOTION_MARKER + 'LOCK_FLAPS', tris_indent + 'ANIM_begin',
        tris_indent + f'ANIM_trans_begin\t{motion["lock"]["dataref"]}',
        tris_indent + 'ANIM_trans_key\t0\t0\t0\t0',
        tris_indent + 'ANIM_trans_key\t1\t' + '\t'.join(f'{obj8_fmt(v)}' for v in lock_vector),
        tris_indent + 'ANIM_trans_end', tris, tris_indent + 'ANIM_end']
    result = ending.join(patched)
    if vertex_payload(text) != vertex_payload(result):
        raise ValueError('visible VT/IDX/UV/texture payload changed')
    if tris_coverage(text) != tris_coverage(result):
        raise ValueError('drawn index ranges changed')
    changed = [n for n, (a, b) in enumerate(zip(lines, patched[:len(lines)])) if a != b]
    before_lock = [n for n in range(lock_line) if lines[n] != patched[n]]
    if len(patched) != len(lines) + 7 or begin + 2 not in before_lock or not set(before_lock) <= set(
            range(begin + 2, begin + 8)):
        raise ValueError(f'Pedestal rebind changed unexpected lines: {changed[:10]}')
    if lines[lock_line + 1:] != patched[lock_line + 8:]:
        raise ValueError('Pedestal rebind changed lines after the lock range')
    for row in rows:
        row['chain'] = [lever['dataref'], *([motion['lock']['dataref']] if row['lock'] else [])]
    return result, [dict(exported_pose_deg=pose, lock_vector_local=list(lock_vector), parts=rows)]


def manipulator(motion: dict[str, dict]) -> list[str]:
    rows = drag_rotate_lines(motion['lever'], PREFIX + 'lever_input', list(LEVER_TO_ANIM), TOOLTIP,
                             cursor='hand', detent_ranges=DETENT_RANGES, lift=lift_length(motion),
                             lift_dataref=PREFIX + 'lift_input', lift_range=(0.0, 1.0))
    # Laminar's A330 flap lever (the XP12 reference for lift detents) lists the detent ranges first.
    return ([rows[0]] + [r for r in rows if r.startswith('ATTR_axis_detent_range')]
            + [r for r in rows if r.startswith('ATTR_manip_keyframe')])


def click_geometry(source: Source, motion: dict[str, dict]):
    box_vertices, box_indices, bounds = oriented_box(source.points_in_frame(CLICK_NODES),
                                                     source.world(source.names[CLICK_NODES[0]]), margin=CLICK_MARGIN)
    manip = manipulator(motion)
    commands = ['ATTR_manip_none', 'ATTR_no_cockpit', 'ATTR_draw_disable', CLICK_MARKER + 'lever',
                *nested_block([motion['lever']], [*manip, f'TRIS\t0\t{len(box_indices)}', 'ATTR_manip_none']),
                'ATTR_draw_enable']
    row = dict(name='lever', nodes=list(CLICK_NODES), target=PREFIX + 'lever_input', lift=PREFIX + 'lift_input',
               lift_length_m=lift_length(motion), chain=[motion['lever']['dataref']],
               manipulator=manip[0].split('\t')[0], keyframes=sum(l.startswith('ATTR_manip_keyframe') for l in manip),
               detent_ranges=[list(r) for r in DETENT_RANGES], triangles=len(box_indices) // 3, **bounds)
    return box_vertices, box_indices, commands, row


def augment_controls(text: str, source: Source, motion: dict[str, dict]) -> tuple[str, list[dict]]:
    if MARKER in text:
        if CLICK_MARKER + 'lever' not in text:
            raise ValueError('Incomplete previous flaps-lever controls patch')
        return text, []
    if PREFIX in text:
        raise ValueError('Flaps-lever datarefs already in the controls object without the marker')
    vertices, indices, commands, row = click_geometry(source, motion)
    return _append(text, vertices, indices, commands, MARKER), [row]


ANIM_HEADER = re.compile(r'^(\s*ANIM_\w+\b.*?)(\S+)(\s*)$')


def rebind_exterior(text: str) -> tuple[str, dict]:
    """Swap only the flap/slat dataref token of ANIM header lines; every other byte is unchanged."""
    counts = {old: 0 for old in EXTERIOR_REBIND}
    already = {new: text.count(new) for new in EXTERIOR_REBIND.values()}
    out = []
    for line in text.splitlines(keepends=True):
        stripped = line.lstrip()
        if stripped.startswith('ANIM_'):
            body = line.rstrip('\r\n')
            ending = line[len(body):]
            words = body.split()
            if words and words[-1] in EXTERIOR_REBIND:
                if words[0] not in ('ANIM_rotate_begin', 'ANIM_trans_begin'):
                    raise ValueError(f'Unexpected flap/slat animation command {words[0]}')
                old = words[-1]
                cut = body.rfind(old)
                line = body[:cut] + EXTERIOR_REBIND[old] + body[cut + len(old):] + ending
                counts[old] += 1
        out.append(line)
    patched = ''.join(out)
    for old in EXTERIOR_REBIND:
        if old in patched:
            raise ValueError(f'{old} still referenced outside animation headers')
    if not any(counts.values()) and not any(already.values()):
        raise ValueError('No flap/slat animation in this object')
    # Only the tokens changed: same line count, identical lines once the new tokens are mapped back.
    back = patched
    for old, new in EXTERIOR_REBIND.items():
        back = back.replace(new, old)
    original_back = text
    for old, new in EXTERIOR_REBIND.items():
        original_back = original_back.replace(new, old)
    if back != original_back:
        raise ValueError('Exterior rebind changed more than the flap/slat tokens')
    return patched, {'replaced': counts, 'already_bound': already}


def stage(aircraft: Path, source_path: Path, project_acf: Path, destination: Path) -> dict:
    if destination.exists():
        raise ValueError('Stage directory must not exist')
    source = Source(source_path)
    motion = motions(source)
    used = {m['dataref'] for m in motion.values()} | {PREFIX + 'lever_input', PREFIX + 'lift_input'} | set(
        EXTERIOR_REBIND.values())
    if not used <= set(FLAPS_DATAREFS):
        raise ValueError(f'Datarefs outside the native contract: {sorted(used - set(FLAPS_DATAREFS))}')
    staged = {}
    staged[('aircraft', PEDESTAL)], wrapped = rebind_pedestal((aircraft / PEDESTAL).read_text(), source, motion)
    staged[('aircraft', CONTROLS)], clicks = augment_controls((aircraft / CONTROLS).read_text(), source, motion)
    rebinds = {}
    for relative in WINGS:
        staged[('aircraft', relative)], rebinds[relative] = rebind_exterior((aircraft / relative).read_text())
    acf_rows = {}
    staged[('aircraft', ACF)], acf_rows[ACF] = patch_acf((aircraft / ACF).read_text(encoding='latin1'))
    staged[('project', PROJECT_ACF)], acf_rows[PROJECT_ACF] = patch_acf(project_acf.read_text(encoding='latin1'))
    destination.mkdir(parents=True)
    files = []
    for (root, relative), text in staged.items():
        original = aircraft / relative if root == 'aircraft' else project_acf
        target = destination / root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(text.encode('latin1' if relative.endswith('.acf') else 'utf-8'))
        files.append(dict(root=root, path=relative, before_sha256=sha(original), after_sha256=sha(target),
                          changed=sha(original) != sha(target)))
    report = dict(format=1, aircraft=str(aircraft), project_acf=str(project_acf), source=str(source_path),
                  source_sha256=sha(source_path), marker=MARKER, files=files,
                  motions={k: describe(v) for k, v in motion.items()}, wrapped=wrapped, click_targets=clicks,
                  exterior_rebind=rebinds, acf_changes=acf_rows, detent_ranges=[list(r) for r in DETENT_RANGES],
                  lift_length_m=lift_length(motion), datarefs_used=sorted(used),
                  visible_vertex_index_uv_payload_preserved=True, simulator_tested=False)
    (destination / 'staging-report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--aircraft', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--project-acf', type=Path, required=True)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    report = stage(args.aircraft.resolve(), args.source.resolve(), args.project_acf.resolve(), args.stage.resolve())
    print(json.dumps({'stage': str(args.stage.resolve()),
                      'files': [{k: f[k] for k in ('root', 'path', 'changed', 'after_sha256')} for f in report['files']],
                      'pedestal': report['wrapped'],
                      'exterior': {k: v['replaced'] for k, v in report['exterior_rebind'].items()},
                      'acf_changed': sum(r['changed'] for r in report['acf_changes'][ACF])}, indent=2))


if __name__ == '__main__':
    main()
