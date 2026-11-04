#!/usr/bin/env python3
"""Stage a selective FCU upgrade while retaining installed cockpit additions.

Writes only below --output. The installed aircraft remains read-only. The
report records source hashes, exact changed vertex IDs and preservation checks.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

from PIL import Image, ImageChops, ImageDraw
from avionics_panel_layout import FCU_PANEL_RECT, PANEL_SIZE
from build_cockpit_controls import (FCU_ANNUNCIATORS, FCU_ANNUNCIATOR_FIRST_CELL,
                                    FCU_LABEL_COLUMNS, FCU_LABEL_ROWS, FCU_LABELS,
                                    FCU_LEGACY_SCREEN_QUADS, load_controls)
from build_fcu_panel import build_label_atlas, build_panel_background
from ecam_button_geometry import CONTROLS_PATH, LAMP_MARKER
from patch_center_right_animations import animation_block
from patch_fcu_animations import fcu_target_groups, patch_object
from patch_efis_animations import vertex_payload
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation


def sha(data):
    return hashlib.sha256(data).hexdigest()


def manipulator_blocks(text):
    return re.findall(r'^ATTR_manip_(?!none)[^\n]*\n.*?^ATTR_manip_none\n', text, re.M | re.S)


def geometry_rows(text, prefixes):
    return [line for line in text.splitlines() if line.startswith(prefixes)]


def patch_controls(original, groups, config):
    if '# FCU BUTTON MOTION ' in original or '# FCU AXIAL MOTION ' in original:
        raise ValueError('Installed control object already contains FCU motion; inspect before restaging')
    targets = {t['name']: t for marker, rows in groups if 'ROTARY' not in marker for t in rows}
    command_nodes = {}
    for control in load_controls(config):
        if not control.node.startswith(('PUSH_FCU_', 'KNOB_FCU_')):
            continue
        for name in (control.command, control.command_up, control.command_down, control.command_push, control.command_pull):
            if name and name.startswith('a380x/xp12/fcu/'):
                command_nodes[name] = control.node
    lines = original.splitlines(keepends=True)
    vertices = [(at, line.split()) for at, line in enumerate(lines) if line.startswith('VT\t')]
    changed_vertices = []
    screen_height = math.hypot(3.0021-2.9828, 32.7603-32.7532)
    cy = (2.9828+3.0021)/2 + .347*.002
    cz = (32.7532+32.7603)/2 - .938*.002
    left, bottom, right, top = FCU_PANEL_RECT
    for x0, x1, u0, u1 in FCU_LEGACY_SCREEN_QUADS:
        expected = [(x0, cy-.938*screen_height/2, cz-.347*screen_height/2),
                    (x1, cy-.938*screen_height/2, cz-.347*screen_height/2),
                    (x1, cy+.938*screen_height/2, cz+.347*screen_height/2),
                    (x0, cy+.938*screen_height/2, cz+.347*screen_height/2)]
        matches = [index for index in range(len(vertices)-3)
                   if all(max(abs(float(vertices[index+corner][1][axis+1])-point[axis]) for axis in range(3))<2e-6
                          for corner, point in enumerate(expected))]
        if len(matches) != 1:
            raise ValueError(f'FCU screen quad is not unique: {x0}, {x1}: {matches}')
        us = ((left+u1*(right-left))/PANEL_SIZE[0], (left+u0*(right-left))/PANEL_SIZE[0])
        uv = [(us[0], bottom/PANEL_SIZE[1]), (us[1], bottom/PANEL_SIZE[1]),
              (us[1], top/PANEL_SIZE[1]), (us[0], top/PANEL_SIZE[1])]
        for corner, (u, v) in enumerate(uv):
            index = matches[0]+corner
            at, fields = vertices[index]
            lines[at] = '\t'.join([*fields[:7], f'{obj8_fmt(u)}', f'{obj8_fmt(v)}'])+'\n'
            changed_vertices.append(index)
    if len(set(changed_vertices)) != 16:
        raise ValueError('FCU screen vertices overlap unexpectedly')
    changed = ''.join(lines)
    applied = []

    def input_block(match):
        block = match.group(0)
        if 'a380x/xp12/fcu/' not in block:
            return block
        command = next(name for name in block.splitlines()[0].split('\t') if name.startswith('a380x/xp12/fcu/'))
        if command not in command_nodes:
            return block
        node = command_nodes[command]
        target = targets[node]
        draw = re.search(r'^TRIS\t(\d+)\t(\d+)$', block, re.M)
        if not draw or len(re.findall(r'^TRIS\t', block, re.M)) != 1:
            raise ValueError(f'FCU control has an unexpected existing draw block: {command}')
        start, count = map(int, draw.groups())
        marker = '# FCU AXIAL MOTION ' if node.startswith('KNOB_FCU_') and 'SELECTOR' not in node else '# FCU BUTTON MOTION '
        replacement = '\n'.join(animation_block(target, start, count, marker))
        block = block[:draw.start()] + replacement + block[draw.end():]
        block = block.replace('V/S/FPA PUSH - level off (upper center)', 'V/S/FPA PUSH - no mode change (upper center)')
        applied.append(dict(command=command, node=node, dataref=target['dataref'], start=start, indices=count))
        return block

    changed = re.sub(r'^ATTR_manip_(?!none)[^\n]*\n.*?^ATTR_manip_none\n', input_block, changed, flags=re.M | re.S)
    used = {row['command'] for row in applied}
    expected = set(command_nodes) - {name for name in command_nodes if name.endswith('_down')}
    if used != expected:
        raise ValueError(f'FCU input inventory mismatch: missing={sorted(expected-used)} extra={sorted(used-expected)}')
    for index, (_, node, light) in enumerate(FCU_ANNUNCIATORS):
        pattern = r'^# FCU ANNUNCIATOR '+re.escape(node)+r'\n(.*?)(?=^# FCU ANNUNCIATOR |\Z)'
        found = re.search(pattern, changed, re.M | re.S)
        if not found:
            raise ValueError(f'FCU lamp block missing: {node}')
        draw = re.search(r'^TRIS\t(\d+)\t(\d+)$', found.group(1), re.M)
        if not draw:
            raise ValueError(f'FCU lamp triangles missing: {node}')
        start, count = map(int, draw.groups())
        block = [f'# FCU ANNUNCIATOR {node}', f'ATTR_light_level\t0\t1\t{light}',
                 'ANIM_begin', f'ANIM_hide\t-0.0001\t0.0001\t{light}',
                 *animation_block(targets[node], start, count, '# FCU BUTTON MOTION '),
                 'ANIM_end', 'ATTR_light_level_reset']
        if index == len(FCU_ANNUNCIATORS)-1:
            # The retained EFIS/glare blocks share their original outer gate.
            block.append('ATTR_light_level\t0\t1\tsim/cockpit2/switches/avionics_power_on')
        changed = changed[:found.start()]+'\n'.join(block)+'\n'+changed[found.end():]
    old_vertices = geometry_rows(original, ('VT\t',))
    new_vertices = geometry_rows(changed, ('VT\t',))
    if len(old_vertices) != len(new_vertices):
        raise ValueError('Selective FCU patch changed vertex count')
    actual = [index for index, (a, b) in enumerate(zip(old_vertices, new_vertices)) if a != b]
    if set(actual) != set(changed_vertices):
        raise ValueError('Unexpected non-FCU vertex changes')
    for index in actual:
        if old_vertices[index].split()[:7] != new_vertices[index].split()[:7]:
            raise ValueError('FCU screen physical geometry changed')
    if geometry_rows(original, ('IDX\t', 'IDX10\t', 'POINT_COUNTS')) != geometry_rows(changed, ('IDX\t', 'IDX10\t', 'POINT_COUNTS')):
        raise ValueError('Selective FCU patch changed index topology')
    if Counter(re.findall(r'^TRIS\t(\d+)\t(\d+)$', original, re.M)) != Counter(re.findall(r'^TRIS\t(\d+)\t(\d+)$', changed, re.M)):
        raise ValueError('Selective FCU patch changed triangle draws')
    def non_fcu(block):
        return not any(token in command_nodes for token in block.splitlines()[0].split('\t'))
    old_blocks = [block for block in manipulator_blocks(original) if non_fcu(block)]
    new_blocks = [block for block in manipulator_blocks(changed) if non_fcu(block)]
    if old_blocks != new_blocks:
        raise ValueError('A non-FCU control changed')
    suffix_marker = '# A380X NATIVE ND DEVICE QUADS V1'
    if original.split(suffix_marker, 1)[1] != changed.split(suffix_marker, 1)[1]:
        raise ValueError('Native ND/SD/ENG additions changed')
    from stage_fcu_shared_displays import patch_shared_displays
    changed, shared_screens = patch_shared_displays(changed)
    return changed, dict(shared_screens=shared_screens, changed_screen_uv_vertices=actual, animated_inputs=applied,
                        untouched_non_fcu_manipulators=len(old_blocks), unchanged_index_topology=True,
                        unchanged_triangle_draws=True, unchanged_native_nd_sd_eng_suffix=True,
                        retained_eng_selection=LAMP_MARKER in changed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--aircraft', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    aircraft, output = args.aircraft.resolve(), args.output.resolve()
    if output.is_relative_to(aircraft):
        parser.error('--output must be outside the installed aircraft')
    output.mkdir(parents=True, exist_ok=True)
    groups = fcu_target_groups(args.source)
    manifest = dict(scope='Staged FCU-only upgrade; installed aircraft untouched', files=[], fcu_panel_rect=FCU_PANEL_RECT)

    def stage(relative, data, checks):
        before = (aircraft/relative).read_bytes()
        target = output/'aircraft'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        manifest['files'].append(dict(installed=str(aircraft/relative), staged=str(target),
                                      relative=relative, before_sha256=sha(before), after_sha256=sha(data), checks=checks))

    original = (aircraft/CONTROLS_PATH).read_text()
    changed, checks = patch_controls(original, groups, Path(__file__).resolve().parents[1]/'config/cockpit_controls.json')
    stage(CONTROLS_PATH, changed.encode(), checks)
    for name in ('panel_a380_cockpit_mip_knob01_4k.obj', 'decal_a380x_cockpit_decal_emis.obj'):
        relative = 'objects/interior/cockpit-panels/'+name
        before = (aircraft/relative).read_text()
        after, meshes = patch_object(before, name, groups)
        stage(relative, after.encode(), dict(unchanged_geometry_uv_textures=vertex_payload(before)==vertex_payload(after), animated_meshes=meshes))
    generated = output/'generated-panel'
    generated.mkdir(exist_ok=True)
    build_label_atlas(generated)
    build_panel_background(generated)
    images = [('cockpit_3d/-PANELS-/Panel.png', generated/'-PANELS-/Panel.png',
               [(FCU_PANEL_RECT[0], PANEL_SIZE[1]-FCU_PANEL_RECT[3], FCU_PANEL_RECT[2], PANEL_SIZE[1]-FCU_PANEL_RECT[1])])]
    for name in ('fcu_labels.png', 'fcu_labels_LIT.png'):
        boxes = []
        for index in range(len(FCU_LABELS)+len(FCU_ANNUNCIATORS)):
            col, row = index%FCU_LABEL_COLUMNS, index//FCU_LABEL_COLUMNS
            boxes.append((col*128, row*64, (col+1)*128, (row+1)*64))
        images.append(('cockpit_3d/'+name, generated/name, boxes))
    for relative, generated_path, boxes in images:
        before = Image.open(aircraft/relative).convert('RGBA')
        source = Image.open(generated_path).convert('RGBA')
        if before.size != source.size:
            raise ValueError(f'{relative}: source dimensions changed')
        after = before.copy()
        mask = Image.new('L', before.size, 0)
        for box in boxes:
            after.paste(source.crop(box), box)
            ImageDraw.Draw(mask).rectangle((box[0], box[1], box[2]-1, box[3]-1), fill=255)
        outside_before = before.copy(); outside_after = after.copy()
        outside_before.paste((0,0,0,0), mask=mask); outside_after.paste((0,0,0,0), mask=mask)
        unchanged = outside_before.tobytes()==outside_after.tobytes()
        if not unchanged:
            raise ValueError(f'{relative}: non-FCU pixels changed')
        checks = dict(unchanged_non_fcu_pixels=True, replaced_rectangles=boxes)
        if 'labels' in relative:
            # The ENG selection overlay deliberately samples this FCU green cell.
            x=(FCU_ANNUNCIATOR_FIRST_CELL%FCU_LABEL_COLUMNS)*128+64
            y=(FCU_ANNUNCIATOR_FIRST_CELL//FCU_LABEL_COLUMNS)*64+32
            checks['eng_selection_sample_before']=before.getpixel((x,y))
            checks['eng_selection_sample_after']=after.getpixel((x,y))
            if before.getpixel((x,y)) != after.getpixel((x,y)) or after.getpixel((x,y)) != (48,255,78,255):
                raise ValueError('ENG selection atlas sample changed')
        target = output/'aircraft'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        after.save(target)
        stage(relative, target.read_bytes(), checks)
    for row in manifest['files']:
        if sha(Path(row['installed']).read_bytes()) != row['before_sha256']:
            raise ValueError('Installed file changed during staging: '+row['installed'])
    (output/'staging-report.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(files=len(manifest['files']), report=str(output/'staging-report.json'))))


if __name__ == '__main__':
    main()
