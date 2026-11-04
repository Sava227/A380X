#!/usr/bin/env python3
"""Static package checks; these do not substitute for an X-Plane flight test."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import re
import struct
from pathlib import Path
from aileron_visual_provider import validate_provider
from avionics_provider import validate_avionics_provider
from avionics_panel_layout import (
    EFB_PANEL_RECT, EWD_PANEL_RECT, FCU_PANEL_RECT, FCU_LCD_PANEL_RECTS,
    MFD_AUTO_ADJUST, MFD_DEVICE_BUS, MFD_DEVICE_IDS, MFD_LIGHTING_CHANNELS,
    ND_PANEL_RECTS, PANEL_SIZE, PFD_PANEL_RECTS,
)
from cockpit_draw_order import cockpit_decal_order_errors
from obj8_numbers import has_exponent


PFD_EFIS_INSTRUMENTS = (
    'EFIS_horizon',
    'EFIS_horizon_CDIs',
    'EFIS_airspeed',
    'EFIS_altitude',
    'EFIS_vvi_vert',
    'EFIS_baro',
    'EFIS_decision_height',
    'EFIS_machnumber',
    'EFIS_hsi_fullround',
    'EFIS_glideslope',
    'EFIS_TAS_GS_wind',
)
# Visible MFD plugin-device screens (docs/mfd-spec/08 section 2.5 a and 10.3);
# their draw-disabled touch twins live in the cockpit-controls object.
MFD_SCREENS_ATTACHMENT = 'interior/cockpit-panels/a380x_mfd_screens.obj'
MFD_SCREENS_ANCHOR = 'interior/cockpit-panels/screen_powered_off_display_faces.obj'
PFD_EFIS_ASSETS = {
    kind: {
        'EFIS_horizon_CDIs': 'el_horizon_CDI.png',
        'EFIS_decision_height': 'EFIS_decision_heigh',
    }.get(kind, kind)
    for kind in PFD_EFIS_INSTRUMENTS
}


def png_spec(path: Path) -> tuple[int, int, int, int] | None:
    """Return width, height, bit depth and color type from a PNG IHDR."""
    try:
        with path.open('rb') as source:
            header = source.read(29)
    except OSError:
        return None
    if len(header) < 29 or header[:8] != b'\x89PNG\r\n\x1a\n' \
            or header[12:16] != b'IHDR':
        return None
    width, height, depth, color_type, _compression, _filter, _interlace = \
        struct.unpack('>2I5B', header[16:29])
    return width, height, depth, color_type


def panel_group(panel: str, name: str) -> str | None:
    match = re.search(
        rf'^GROUP {re.escape(name)}\r?\n(.*?)^END_GROUP$', panel, re.M | re.S
    )
    return match.group(1) if match else None


def native_panel_declaration(panel: str) -> bool:
    marker = '# A380X FBW AVIONICS PLUGIN OWNS PFD, ND, EWD, FCU AND EFB REGIONS'
    if marker in panel:
        return True
    # Saved ACF files can omit generator comments. Accept the existing hidden,
    # empty native group as its semantic equivalent; registry, display meshes
    # and absence of stock instruments are still checked independently below.
    records = [line.strip() for line in panel.splitlines()
               if line.strip() and not line.lstrip().startswith('#')]
    return (len(records)==4 and records[0] in ('GROUP A380 NATIVE AVIONIC',
            'GROUP A380 NATIVE AVIONICS') and records[1:]==['LOCKED','HIDE','END_GROUP'])


# Native trim plugin (native/trim/plugin.c): commands, out_names datarefs and cockpit device.
TRIM_PLUGIN = Path('plugins') / 'A380XTrim'
TRIM_PREFIX = 'a380x/xp12/trim/'
TRIM_COMMANDS = ('pitch_up', 'pitch_down', 'rudder_left', 'rudder_right', 'rudder_reset')
TRIM_DATAREFS = ('pitch_switch', 'rudder_knob', 'reset_pressed', 'pitch_ratio', 'rudder_ratio',
                 'rudder_degrees', 'powered', 'device_active', 'draw_count', 'control_law',
                 'manual_pitch_available', 'auto_pitch_active', 'rudder_resetting',
                 'device_bound', 'computer_active')
TRIM_DEVICE = 'a380x.xp12.trim.rudder'


def trim_provider(root: Path) -> dict | None:
    """The packaged trim plugin, only when its binary matches its build report."""
    report_path = root / TRIM_PLUGIN / 'build-report.json'
    binary = root / TRIM_PLUGIN / 'mac_x64' / 'A380XTrim.xpl'
    if not report_path.is_file() or not binary.is_file():
        return None
    try:
        report = json.loads(report_path.read_text(encoding='utf-8'))
    except ValueError:
        return None
    if hashlib.sha256(binary.read_bytes()).hexdigest() != report.get('sha256'):
        return None
    return {'commands': {TRIM_PREFIX + name for name in TRIM_COMMANDS},
            'datarefs': {TRIM_PREFIX + name for name in TRIM_DATAREFS},
            'device': TRIM_DEVICE}


def acf_attachments(text: str) -> list[tuple[str,str]]:
    # Plane Maker may serialize property keys lexically (0,1,10,100,...,2).
    # Attachment/draw order follows the numeric property index, not file order.
    rows = re.findall(r'^P _obja/(\d+)/_v10_att_file_stl (.+)$', text, re.M)
    return sorted(rows,key=lambda row:int(row[0]))


def cockpit_attachment_alignment_errors(text: str, controls: str) -> list[str]:
    """The authored cockpit screen, backing and touch geometry share a datum."""
    rows = {path: f'_obja/{index}/' for index, path in acf_attachments(text)}
    if controls not in rows or MFD_SCREENS_ATTACHMENT not in rows:
        return []  # Missing attachments are diagnosed by validate_native_panel.
    props = dict(re.findall(r'^P ([^ ]+) (.*)$', text, re.M))
    reference = MFD_SCREENS_ANCHOR if MFD_SCREENS_ANCHOR in rows else MFD_SCREENS_ATTACHMENT
    targets = [controls, MFD_SCREENS_ATTACHMENT]
    errors = []
    angles = ('_v10_att_phi_ref', '_v10_att_the_ref', '_v10_att_psi_ref')
    offsets = ('_v10_att_x_acf_prt_ref', '_v10_att_y_acf_prt_ref', '_v10_att_z_acf_prt_ref')
    parents = ('_v10_att_body', '_v10_att_gear', '_v10_att_wing')
    for target in targets:
        if target == reference:
            continue
        for suffix in (*angles, *offsets, *parents):
            default = '-1' if suffix in parents else '0'
            try:
                actual = float(props.get(rows[target] + suffix, default))
                expected = float(props.get(rows[reference] + suffix, default))
                if not math.isfinite(actual) or not math.isfinite(expected):
                    raise ValueError('nonfinite attachment transform')
            except ValueError:
                errors.append(f'{target}: invalid cockpit attachment transform {suffix}')
                continue
            difference = (actual - expected + 180) % 360 - 180 if suffix in angles else actual - expected
            if abs(difference) > 1e-5:
                errors.append(f'{target}: cockpit attachment {suffix}={actual:g} does not match '
                              f'{reference} ({expected:g}); displays and click targets are misaligned')
    return errors


def aerodynamic_wing_errors(text: str) -> list[str]:
    """Extended enabled wing slots are valid; serialized disabled slots are not.

    The old >=12 guard was specific to the 747 scaffold. A native A388 can
    legitimately use those slots for outboard panels, center sections/pylons.
    Preserve the actual safety check: no uninitialized disabled wing records.
    """
    rows = re.findall(r'^P _wing/(\d+)/([^ ]+) (.*)$', text, re.M)
    slots: dict[str, dict[str, str]] = {}
    for index, key, value in rows:
        slots.setdefault(index, {})[key] = value
    return [f'Inactive aerodynamic wing slot {index} is still serialized'
            for index, fields in sorted(slots.items(), key=lambda row: int(row[0]))
            if fields.get('_part_specs_eq') != '1']


def trim_configuration_errors(text: str) -> list[str]:
    """Reject the native XP12 flying-stabilizer/elevator-tab load conflict."""
    props = dict(re.findall(r'^P ([^ ]+) (.*)$', text, re.M))
    errors = []

    def number(key: str) -> float:
        try:
            value = float(props.get(key, '0'))
            if not math.isfinite(value):
                raise ValueError('nonfinite')
            return value
        except ValueError:
            errors.append(f'Invalid numeric trim property: {key}')
            return 0.0

    stabilizer_range = max(abs(number('acf/_stab_trim_up')), abs(number('acf/_stab_trim_dn')))
    elevator_range = max(abs(number('acf/_ptch_acft_up_trim_rat')),
                         abs(number('acf/_ptch_acft_dn_trim_rat')))
    moving_stabilizer = False
    for key in props:
        match = re.fullmatch(r'_wing/(\d+)/_inc_trim/\d+', key)
        if match and props.get(f'_wing/{match[1]}/_part_specs_eq') == '1':
            moving_stabilizer = number(key) != 0.0 or moving_stabilizer
    if moving_stabilizer and stabilizer_range > 0.0 and elevator_range > 0.0:
        errors.append('Pitch trim drives BOTH the horizontal stabilizer and elevator trim tab; '
                      'a trimmable stabilizer requires zero acf/_ptch_acft_up_trim_rat and '
                      'acf/_ptch_acft_dn_trim_rat')
    return errors


def cockpit_region_quad_bounds(obj: str) -> list[tuple[float, float, float, float]]:
    """Return UV bounds for six-index draws in cockpit region zero."""
    vertices: list[tuple[float, float]] = []
    indices: list[int] = []
    draws: list[tuple[int, int]] = []
    in_region = False
    for line in obj.splitlines():
        words = line.split()
        if not words:
            continue
        command = words[0]
        if command == 'VT' and len(words) == 9:
            vertices.append((float(words[7]), float(words[8])))
        elif command in ('IDX', 'IDX10'):
            indices.extend(map(int, words[1:]))
        elif command == 'ATTR_cockpit_region':
            in_region = len(words) == 2 and words[1] == '0'
        elif command == 'ATTR_no_cockpit':
            in_region = False
        elif command == 'TRIS' and in_region and len(words) >= 3:
            draws.append((int(words[1]), int(words[2])))
    bounds = []
    for first, count in draws:
        if count != 6 or first < 0 or first + count > len(indices):
            continue
        try:
            uv = [vertices[index] for index in indices[first:first + count]]
        except IndexError:
            continue
        bounds.append((
            min(point[0] for point in uv),
            min(point[1] for point in uv),
            max(point[0] for point in uv),
            max(point[1] for point in uv),
        ))
    return bounds


def cockpit_device_quads(obj: str, manipulated: bool = False) -> dict[str, list[tuple[float, float, float, float]]]:
    """Return UV bounds of six-index draws per plugin cockpit device.

    A draw counts for a device while ``ATTR_cockpit_device <id>`` is active
    (until ``ATTR_no_cockpit`` or a cockpit region). With ``manipulated`` it
    must also sit under ``ATTR_manip_device <cursor> <id>`` for the same id.
    """
    vertices: list[tuple[float, float]] = []
    indices: list[int] = []
    draws: list[tuple[str, int, int]] = []
    device = manip_device = None
    for line in obj.splitlines():
        words = line.split()
        if not words:
            continue
        command = words[0]
        if command == 'VT' and len(words) == 9:
            vertices.append((float(words[7]), float(words[8])))
        elif command in ('IDX', 'IDX10'):
            indices.extend(map(int, words[1:]))
        elif command == 'ATTR_cockpit_device':
            device = words[1] if len(words) == 5 else None
        elif command in ('ATTR_no_cockpit', 'ATTR_cockpit_region', 'ATTR_cockpit'):
            device = None
        elif command == 'ATTR_manip_device':
            manip_device = words[2] if len(words) >= 3 else None
        elif command.startswith('ATTR_manip_'):
            manip_device = None
        elif command == 'TRIS' and device is not None and len(words) >= 3:
            if not manipulated or manip_device == device:
                draws.append((device, int(words[1]), int(words[2])))
    quads: dict[str, list[tuple[float, float, float, float]]] = {}
    for name, first, count in draws:
        if count != 6 or first < 0 or first + count > len(indices):
            continue
        try:
            uv = [vertices[index] for index in indices[first:first + count]]
        except IndexError:
            continue
        quads.setdefault(name, []).append((
            min(point[0] for point in uv), min(point[1] for point in uv),
            max(point[0] for point in uv), max(point[1] for point in uv),
        ))
    return quads


def cockpit_device_attributes(obj: str) -> dict[str, set[tuple[str, ...]]]:
    """Return every (bus, lighting channel, auto adjust) used per device ID."""
    result: dict[str, set[tuple[str, ...]]] = {}
    for line in obj.splitlines():
        words = line.split()
        if words and words[0] == 'ATTR_cockpit_device' and len(words) == 5:
            result.setdefault(words[1], set()).add(tuple(words[2:5]))
    return result


def validate_native_panel(
    root: Path,
    acf_text: str,
    attachment_paths: list[str],
) -> tuple[list[str], dict[str, object]]:
    """Validate the packaged PFD/ND/EWD/flyPad/FCU contract from package files alone."""
    errors: list[str] = []
    required = bool(re.search(r'^P acf/_ICAO A388$', acf_text, re.M))
    report: dict[str, object] = {
        'required_for_a388': required,
        'panel_size': None,
        'pfd_groups': [],
        'nd_groups': [],
        'renderer': None,
        'cockpit_region_pfd_quads': 0,
        'cockpit_region_nd_quads': 0,
        'cockpit_region_ewd_quads': 0,
        'cockpit_region_efb_quads': 0,
        'cockpit_region_fcu_quads': 0,
        'efb_orientation': None,
        'cockpit_device_quads': {},
        'cockpit_device_touch_quads': {},
    }
    if not required:
        return errors, report

    panel_path = root / 'cockpit_3d/-PANELS-/Panel.png'
    spec = png_spec(panel_path)
    if spec is None:
        errors.append('Native 3-D panel is missing or not a valid PNG: cockpit_3d/-PANELS-/Panel.png')
    else:
        report['panel_size'] = list(spec[:2])
        if spec != (*PANEL_SIZE, 8, 6):
            errors.append(
                f'Native 3-D panel must be a {PANEL_SIZE[0]}x{PANEL_SIZE[1]} 8-bit RGBA PNG; '
                f'got {spec[0]}x{spec[1]} depth={spec[2]} color_type={spec[3]}'
            )

    panel_match = re.search(
        r'^PANEL_3D_BEGIN\r?\n(.*?)^PANEL_3D_END$', acf_text, re.M | re.S
    )
    panel = panel_match.group(1) if panel_match else ''
    if not panel_match:
        errors.append('A388 ACF has no native PANEL_3D definition')
    if not native_panel_declaration(panel):
        errors.append('A388 panel does not delegate the PFD, ND, EWD, flyPad and FCU displays to the FBW avionics plugin')
    forbidden = (*PFD_EFIS_INSTRUMENTS, 'EFIS_map_s_GA',
                 'sim/cockpit2/autopilot/', 'gen_LED')
    for token in forbidden:
        if token in panel:
            errors.append(f'A388 panel still consumes forbidden stock X-Plane avionics: {token}')
    provider, provider_errors = validate_avionics_provider(root)
    errors.extend(provider_errors)
    if provider is not None:
        report['renderer'] = provider['provider']
        report['pfd_groups'] = ['captain', 'first_officer']
        report['nd_groups'] = ['captain', 'first_officer']

    controls = next(
        (path for path in attachment_paths if path.startswith('interior/cockpit-controls/')),
        None,
    )
    if controls is None:
        errors.append('A388 has no cockpit-controls attachment for live panel regions')
        return errors, report
    controls_path = root / 'objects' / controls
    if not controls_path.is_file():
        errors.append(f'Missing cockpit-controls object for live panel regions: {controls}')
        return errors, report
    errors.extend(cockpit_attachment_alignment_errors(acf_text, controls))
    controls_text = controls_path.read_text(encoding='utf-8', errors='replace')
    for stock_route in ('sim/autopilot/', 'sim/aircraft/autopilot/', 'sim/cockpit2/autopilot/'):
        if stock_route in controls_text:
            errors.append(f'{controls}: physical FCU retains a stock autopilot route: {stock_route}')
    efb_orientation_marker = '# A380X FLYPAD ORIENTATION: UV0 TOUCH0 A380_CONTROL_UI_V3'
    if efb_orientation_marker in controls_text:
        report['efb_orientation'] = 'UV0 TOUCH0 A380_CONTROL_UI_V3'
    else:
        errors.append(
            f'{controls}: missing validated flyPad UV/touch orientation signature'
        )
    if not re.search(rf'^COCKPIT_REGION\s+0\s+0\s+{PANEL_SIZE[0]}\s+{PANEL_SIZE[1]}$',
                     controls_text, re.M):
        errors.append(f'{controls}: cockpit region zero is not {PANEL_SIZE[0]}x{PANEL_SIZE[1]}')
    bounds = cockpit_region_quad_bounds(controls_text)

    def panel_uv(rect: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        left, bottom, right, top = rect
        return left / PANEL_SIZE[0], bottom / PANEL_SIZE[1], right / PANEL_SIZE[0], top / PANEL_SIZE[1]

    expected_pfd = tuple(map(panel_uv, PFD_PANEL_RECTS))
    expected_nd = tuple(map(panel_uv, ND_PANEL_RECTS))
    matched_pfd = 0
    for target in expected_pfd:
        if any(all(abs(a - b) <= 1e-6 for a, b in zip(bounds_item, target))
               for bounds_item in bounds):
            matched_pfd += 1
    report['cockpit_region_pfd_quads'] = matched_pfd
    if matched_pfd != 2:
        errors.append(
            f'{controls}: expected captain and first-officer cockpit-region PFD quads; '
            f'found {matched_pfd} of 2'
        )
    matched_nd = 0
    for target in expected_nd:
        if any(all(abs(a - b) <= 1e-6 for a, b in zip(bounds_item, target))
               for bounds_item in bounds):
            matched_nd += 1
    report['cockpit_region_nd_quads'] = matched_nd
    if matched_nd != 2:
        errors.append(
            f'{controls}: expected captain and first-officer cockpit-region ND quads; '
            f'found {matched_nd} of 2'
        )
    expected_ewd = panel_uv(EWD_PANEL_RECT)
    matched_ewd = sum(
        1 for bounds_item in bounds
        if all(abs(a - b) <= 1e-6 for a, b in zip(bounds_item, expected_ewd))
    )
    report['cockpit_region_ewd_quads'] = matched_ewd
    if matched_ewd != 1:
        errors.append(f'{controls}: expected native EWD cockpit-region quad; found {matched_ewd}')
    expected_fcu = tuple(map(panel_uv, FCU_LCD_PANEL_RECTS))
    fcu_matches = [sum(all(abs(a-b) <= 1e-6 for a,b in zip(bounds_item, target))
                       for bounds_item in bounds) for target in expected_fcu]
    matched_fcu = sum(count == 1 for count in fcu_matches)
    report['cockpit_region_fcu_quads'] = matched_fcu
    if matched_fcu != 2:
        errors.append(f'{controls}: expected two shared native FCU cockpit-region display quads '
                      f'(SPD/HDG and ALT/V/S); found {matched_fcu} of 2; matches={fcu_matches}')
    expected_efb = panel_uv(EFB_PANEL_RECT)
    matched_efb = sum(
        1 for bounds_item in bounds
        if all(abs(a - b) <= 1e-6 for a, b in zip(bounds_item, expected_efb))
    )
    report['cockpit_region_efb_quads'] = matched_efb
    if matched_efb != 2:
        errors.append(
            f'{controls}: expected captain and first-officer cockpit-region EFB quads; '
            f'found {matched_efb} of 2'
        )

    # Both MFDs are plugin cockpit devices: one visible full-UV quad per ID in
    # the separate screen object, and one draw-disabled ATTR_manip_device
    # touch twin per ID in the cockpit object, with identical device tags.
    expected_devices = list(MFD_DEVICE_IDS)
    if provider is not None and provider.get('cockpit_devices') != expected_devices:
        errors.append('Avionics provider cockpit_devices do not match the MFD device IDs '
                      + ', '.join(expected_devices))
    expected_attributes = {
        device: {(str(MFD_DEVICE_BUS), str(channel), str(MFD_AUTO_ADJUST))}
        for device, channel in zip(MFD_DEVICE_IDS, MFD_LIGHTING_CHANNELS)
    }
    if provider and provider.get('xpkit_nd'):
        from nd_cockpit_devices import ND_DEVICES, ND_CHANNELS
        expected_devices.extend(ND_DEVICES)
        expected_attributes.update({device:{('1',str(channel),'1')} for device,channel in zip(ND_DEVICES,ND_CHANNELS)})
    if provider and provider.get('xpkit_pfd'):
        from pfd_cockpit_devices import PFD_DEVICES, PFD_CHANNELS
        expected_devices.extend(PFD_DEVICES)
        expected_attributes.update({device:{('1',str(channel),'1')} for device,channel in zip(PFD_DEVICES,PFD_CHANNELS)})
    if provider and provider.get('isis'):
        from isis_cockpit_device import ISIS_DEVICES, ISIS_CHANNELS, ISIS_COMMANDS
        expected_devices.extend(device for device in ISIS_DEVICES if device in provider['isis'].get('cockpit_devices', ()))
        expected_attributes.update({device:{('1',str(ISIS_CHANNELS[device]),'1')} for device in ISIS_DEVICES
                                    if device in provider['isis'].get('cockpit_devices', ())})
        for command in provider['isis'].get('commands', ()):  # every physical control; popup is screen only
            if command.endswith('/popup'):
                continue
            if command not in controls_text:
                errors.append(f'{controls}: missing physical ISIS {command} manipulator')
    full_uv = (0.0, 0.0, 1.0, 1.0)
    refresh_hint = ' (regenerate with build_cockpit_controls.py and the cockpit-controls refresh)'

    def device_quad_counts(text: str, manipulated: bool) -> dict[str, int]:
        quads = cockpit_device_quads(text, manipulated)
        return {device: sum(all(abs(a - b) <= 1e-6 for a, b in zip(item, full_uv))
                            for item in quads.get(device, ()))
                for device in sorted(set(quads) | set(expected_devices))}

    touch_counts = device_quad_counts(controls_text, True)
    report['cockpit_device_touch_quads'] = touch_counts
    for device in expected_devices:
        if touch_counts.get(device) != 1:
            errors.append(f'{controls}: expected one full-UV ATTR_manip_device touch quad for MFD device '
                          f'{device}; found {touch_counts.get(device, 0)}' + refresh_hint)
    unexpected = sorted(set(touch_counts) - set(expected_devices))
    if unexpected:
        errors.append(f'{controls}: unexpected cockpit devices: ' + ', '.join(unexpected))
    if cockpit_device_attributes(controls_text) != expected_attributes and \
            any(touch_counts.get(device) for device in expected_devices):
        errors.append(f'{controls}: MFD ATTR_cockpit_device bus/channel/auto values differ from '
                      'avionics_panel_layout')
    if MFD_SCREENS_ATTACHMENT not in attachment_paths:
        errors.append(f'A388 has no visible MFD screen attachment {MFD_SCREENS_ATTACHMENT}' + refresh_hint)
        return errors, report
    if MFD_SCREENS_ANCHOR in attachment_paths and \
            attachment_paths.index(MFD_SCREENS_ATTACHMENT) < attachment_paths.index(MFD_SCREENS_ANCHOR):
        errors.append(f'{MFD_SCREENS_ATTACHMENT} must be attached after {MFD_SCREENS_ANCHOR}')
    screens_path = root / 'objects' / MFD_SCREENS_ATTACHMENT
    if not screens_path.is_file():
        errors.append(f'Missing visible MFD screen object: {MFD_SCREENS_ATTACHMENT}')
        return errors, report
    # SD is not a touchscreen: its ENGINE/APU selection uses the physical ECP
    # command meshes. Require a visible quad, not an ATTR_manip_device twin.
    if provider and provider.get('ecam_sd'):
        from sd_cockpit_device import SD_DEVICE, SD_CHANNEL, PAGE_ACTIONS, ACTION_NODES
        expected_devices.append(SD_DEVICE)
        expected_attributes[SD_DEVICE]={('1',str(SD_CHANNEL),'1')}
        for action in (*PAGE_ACTIONS,*ACTION_NODES.values(),'brightness_up','brightness_down'):
            if 'a380x/xp12/ecam/sd/'+action not in controls_text:
                errors.append(f'{controls}: missing physical ECAM SD {action} command')
    screens_text = screens_path.read_text(encoding='utf-8', errors='replace')
    screen_counts = device_quad_counts(screens_text, False)
    report['cockpit_device_quads'] = screen_counts
    for device in expected_devices:
        if screen_counts.get(device) != 1:
            errors.append(f'{MFD_SCREENS_ATTACHMENT}: expected one full-UV visible quad for MFD device '
                          f'{device}; found {screen_counts.get(device, 0)}')
    unexpected = sorted(set(screen_counts) - set(expected_devices))
    if unexpected:
        errors.append(f'{MFD_SCREENS_ATTACHMENT}: unexpected cockpit devices: ' + ', '.join(unexpected))
    screen_attributes = cockpit_device_attributes(screens_text)
    if trim_provider(root) is not None:
        # The trim plugin's rudder trim display shares the screen object (trim_cockpit_panel.py).
        screen_attributes.pop(TRIM_DEVICE, None)
    if screen_attributes != expected_attributes:
        errors.append(f'{MFD_SCREENS_ATTACHMENT}: MFD ATTR_cockpit_device bus/channel/auto values differ '
                      'from avionics_panel_layout')
    if 'ATTR_manip_' in screens_text:
        errors.append(f'{MFD_SCREENS_ATTACHMENT}: visible screen object must not carry manipulators')
    from mfd_cockpit_devices import refresh_touch_geometry
    try:
        if refresh_touch_geometry(controls_text) != controls_text:
            errors.append(f'{controls}: MFD touch surfaces are not 1 mm in front of the visible screens'
                          + refresh_hint)
    except (ValueError, IndexError) as error:
        errors.append(f'{controls}: invalid MFD touch geometry: {error}')
    return errors, report


def file_lines(path: Path):
    with path.open() as source:
        yield from source


def validate_fmod_package(root: Path) -> tuple[list[str], list[str]]:
    errors, warnings = [], []
    directory = root / 'fmod'
    if not directory.exists():
        warnings.append('FMOD package not installed; aircraft uses no custom X-Plane sound bank')
        return errors, warnings
    snd = directory / 'A380X-XP12.snd'
    guids = directory / 'GUIDs.txt'
    master = directory / 'Master Bank.bank'
    for path in (snd, guids, master):
        if not path.is_file():
            errors.append(f'Incomplete FMOD package: missing fmod/{path.name}')
    if not snd.is_file():
        return errors, warnings
    snd_text = snd.read_text(encoding='utf-8', errors='replace')
    if not snd_text.startswith('A\n1000\nACF_SOUNDS\n'):
        errors.append('FMOD SND has an invalid X-Plane header')
    if snd_text.count('BEGIN_SOUND_ATTACHMENT') != snd_text.count('END_SOUND_ATTACHMENT'):
        errors.append('FMOD SND has unbalanced sound attachments')
    objects = set(re.findall(r'^(?:EVENT_NAME|SNAPSHOT_NAME)\s+(.+)$', snd_text, re.M))
    if guids.is_file():
        guid_objects = set(re.findall(
            r'\b(?:event|snapshot):/[^\r\n]+',
            guids.read_text(encoding='utf-8', errors='replace'),
        ))
        for name in sorted(objects - guid_objects):
            errors.append(f'FMOD GUIDs.txt lacks attached object: {name}')
    return errors, warnings


def validate(root: Path, datarefs: Path | None = None, commands: Path | None = None) -> dict:
    errors, warnings, objects = [], [], []
    packaged_commands = set()
    for lua in (root / 'plugins' / 'FlyWithLua' / 'Scripts (optional)').glob('*.lua'):
        packaged_commands.update(re.findall(
            r'create_command\(\s*["\']([^"\']+)["\']',
            lua.read_text(encoding='utf-8', errors='replace'),
        ))
    custom_refs, provider_errors = validate_provider(root)
    errors.extend(provider_errors)
    if custom_refs:
        warnings.append('Aileron dance requires global FlyWithLua visual driver; package validation does not prove runtime execution')
    acf = root / 'A380X-XP12.acf'
    text = acf.read_text(encoding='latin1')
    attachments = acf_attachments(text)
    paths = [path for _index, path in attachments]
    errors.extend(cockpit_decal_order_errors(paths))
    native_panel_errors, native_panel = validate_native_panel(root, text, paths)
    errors.extend(native_panel_errors)
    avionics_provider, _ignored_provider_errors = validate_avionics_provider(root)
    packaged_devices: set[str] = set()
    if avionics_provider is not None:
        packaged_commands.update(avionics_provider.get('commands', ()))
        custom_refs.update(avionics_provider.get('datarefs', ()))
        packaged_devices.update(avionics_provider.get('cockpit_devices', ()))
        if avionics_provider.get('xpkit_nd'):
            packaged_devices.update(avionics_provider['xpkit_nd']['cockpit_devices'])
            packaged_commands.update(avionics_provider['xpkit_nd']['commands'])
            custom_refs.update(avionics_provider['xpkit_nd']['datarefs'])
        if avionics_provider.get('xpkit_pfd'):
            packaged_devices.update(avionics_provider['xpkit_pfd']['cockpit_devices'])
            packaged_commands.update(avionics_provider['xpkit_pfd']['commands'])
            custom_refs.update(avionics_provider['xpkit_pfd']['datarefs'])
        if avionics_provider.get('ecam_sd'):
            packaged_devices.update(avionics_provider['ecam_sd']['cockpit_devices'])
        if avionics_provider.get('isis'):
            packaged_devices.update(avionics_provider['isis']['cockpit_devices'])
            packaged_commands.update(avionics_provider['isis']['commands'])
            custom_refs.update(avionics_provider['isis']['datarefs'])
    trim = trim_provider(root)
    if trim is not None:
        packaged_commands.update(trim['commands'])
        custom_refs.update(trim['datarefs'])
        packaged_devices.add(trim['device'])
    flags = {
        index: int(value)
        for index, value in re.findall(r'^P _obja/(\d+)/_obj_flags (-?\d+)$', text, re.M)
    }
    if len(paths) != len(set(paths)):
        errors.append('Duplicate object attachment paths')
    known = None
    if datarefs:
        known = {line.split()[0] for line in datarefs.read_text().splitlines()
                 if line.startswith('sim/')}
    known_commands = None
    if commands:
        known_commands = {line.split()[0] for line in commands.read_text().splitlines()
                          if line.startswith('sim/')}
    cockpit_objects = [index for index, value in flags.items() if value & 2048]
    if len(cockpit_objects) > 1:
        errors.append(f'Multiple cockpit objects configured: {", ".join(cockpit_objects)}')
    for attachment_index, relative in attachments:
        obj = root / 'objects' / relative
        if not obj.is_file():
            errors.append(f'Missing object: {relative}')
            continue
        verts = 0
        indices = 0
        max_index = -1
        min_index = 0
        points = None
        animation_depth = 0
        key_table, previous_key, key_order_reported = None, None, False
        # Sample enough declared vertices/indices to detect a globally reversed
        # export without retaining the entire multi-million-vertex aircraft.
        sample_vertices, sample_indices = [], []
        textures, animated_refs, manipulator_commands, manipulator_refs = [], set(), set(), set()
        manipulator_devices, cockpit_device = set(), None
        exponent_lines, first_exponent = 0, None
        for line_number, line in enumerate(file_lines(obj), 1):
            words = line.split()
            if not words:
                continue
            if has_exponent(line):
                exponent_lines += 1
                first_exponent = first_exponent or line_number
            command = words[0]
            if command == 'VT':
                verts += 1
                if len(words) != 9:
                    errors.append(f'{relative}: malformed vertex')
                elif len(sample_vertices) < 12000:
                    sample_vertices.append(tuple(map(float, words[1:7])))
            elif command in ('IDX', 'IDX10'):
                expected = 10 if command == 'IDX10' else 1
                if len(words) != expected + 1:
                    errors.append(f'{relative}: malformed {command} record')
                    continue
                parsed = list(map(int, words[1:]))
                if len(sample_indices) < 12000:
                    sample_indices.extend(parsed[:12000 - len(sample_indices)])
                indices += len(parsed)
                max_index = max(max_index, *parsed)
                min_index = min(min_index, *parsed)
            elif command == 'TRIS':
                first, count = map(int, words[1:3])
                if first < 0 or count < 0 or first + count > indices or count % 3:
                    errors.append(f'{relative}: invalid TRIS range {first} {count}')
            elif command == 'POINT_COUNTS':
                points = list(map(int, words[1:]))
            elif command.startswith('TEXTURE'):
                value = line.strip().split(maxsplit=2 if command == 'TEXTURE_MAP' else 1)[-1]
                if value.upper() in ('TEXTURE', 'NONE'):
                    continue
                texture = obj.parent / value
                textures.append(value)
                if Path(value).is_absolute():
                    errors.append(f'{relative}: non-portable absolute texture path {value}')
                if not texture.is_file():
                    errors.append(f'{relative}: missing texture {value}')
                if not texture.resolve().is_relative_to(root.resolve()):
                    errors.append(f'{relative}: texture outside package {value}')
                if texture.with_suffix('.dds').is_file():
                    warnings.append(f'{relative}: DDS overrides portable PNG: {value}')
            elif command == 'ANIM_begin':
                animation_depth += 1
            elif command == 'ANIM_end':
                animation_depth -= 1
                if animation_depth < 0:
                    errors.append(f'{relative}: unbalanced animation end')
            elif command in ('ANIM_rotate_begin', 'ANIM_trans_begin', 'ANIM_rotate', 'ANIM_trans', 'ANIM_hide', 'ANIM_show'):
                ref = words[-1]
                try:
                    float(ref)
                    numeric = True
                except ValueError:
                    numeric = False
                if ref != 'none' and not numeric:
                    animated_refs.add(ref)
            elif command == 'ATTR_manip_command':
                if len(words) < 4:
                    errors.append(f'{relative}: malformed command manipulator')
                else:
                    manipulator_commands.add(words[2])
            elif command in ('ATTR_manip_command_knob', 'ATTR_manip_command_switch_up_down'):
                if len(words) < 5:
                    errors.append(f'{relative}: malformed two-command manipulator')
                else:
                    manipulator_commands.update(words[2:4])
            elif command == 'ATTR_manip_drag_axis':
                if len(words) < 9:
                    errors.append(f'{relative}: malformed drag-axis manipulator')
                else:
                    manipulator_refs.add(words[7])
            elif command == 'ATTR_manip_toggle':
                if len(words) < 6:
                    errors.append(f'{relative}: malformed toggle manipulator')
                else:
                    manipulator_refs.add(words[4])
            elif command == 'ATTR_manip_drag_rotate':
                # <cursor> <x y z> <dx dy dz> <angle1> <angle2> <lift> <v1min v1max> <v2min v2max>
                # <dataref1> <dataref2|none> <tooltip>
                if len(words) < 18:
                    errors.append(f'{relative}: malformed drag-rotate manipulator')
                else:
                    try:
                        numbers = [float(value) for value in words[2:15]]
                    except ValueError:
                        numbers = []
                    if len(numbers) != 13 or not all(math.isfinite(value) for value in numbers) \
                            or not any(numbers[3:6]):
                        errors.append(f'{relative}: drag-rotate manipulator with invalid geometry')
                    manipulator_refs.add(words[15])
                    if words[16].lower() != 'none':
                        manipulator_refs.add(words[16])
            elif command == 'ATTR_cockpit_device':
                # <device> <bus bitfield> <lighting channel> <auto adjust>
                if len(words) != 5:
                    errors.append(f'{relative}: malformed ATTR_cockpit_device (expected 4 values)')
                    cockpit_device = None
                else:
                    cockpit_device = words[1]
            elif command in ('ATTR_no_cockpit', 'ATTR_cockpit_region', 'ATTR_cockpit'):
                cockpit_device = None
            elif command == 'ATTR_manip_device':
                # <cursor> <device> <tooltip>
                if len(words) < 3:
                    errors.append(f'{relative}: malformed device manipulator')
                else:
                    manipulator_devices.add(words[2])
                    if cockpit_device != words[2]:
                        errors.append(f'{relative}: ATTR_manip_device {words[2]} is not preceded by a '
                                      'matching ATTR_cockpit_device')
            if command in ('ANIM_rotate_begin', 'ANIM_trans_begin'):
                if key_table is not None:
                    errors.append(f'{relative}: nested animation key tables')
                key_table, previous_key, key_order_reported = command, None, False
            elif command in ('ANIM_rotate_key', 'ANIM_trans_key'):
                expected = command.replace('_key', '_begin')
                if key_table != expected:
                    errors.append(f'{relative}: key outside matching animation table')
                value = float(words[1])
                if previous_key is not None and value < previous_key and not key_order_reported:
                    errors.append(f'{relative}: animation key table is not ordered low-to-high')
                    key_order_reported = True
                previous_key = value
            elif command in ('ANIM_rotate_end', 'ANIM_trans_end'):
                if key_table != command.replace('_end', '_begin'):
                    errors.append(f'{relative}: mismatched animation key-table end')
                key_table = None
        if points is None or points[0] != verts or points[3] != indices:
            errors.append(f'{relative}: vertex/index count mismatch {points} vs {verts}/{indices}')
        if exponent_lines:
            errors.append(f'{relative}: {exponent_lines} lines use exponent notation (first line {first_exponent}); '
                          'X-Plane reads 4.7e-07 as 4.7 - run tools/obj8_numbers.py --fix')
        if min_index < 0 or max_index >= verts:
            errors.append(f'{relative}: vertex index outside 0..{verts-1}')
        if animation_depth:
            errors.append(f'{relative}: unclosed animation blocks')
        if key_table is not None:
            errors.append(f'{relative}: unclosed animation key table')
        winding = {'clockwise': 0, 'counterclockwise': 0, 'degenerate_or_unsampled': 0}
        for offset in range(0, len(sample_indices) - 2, 3):
            ids = sample_indices[offset:offset + 3]
            if any(index < 0 or index >= len(sample_vertices) for index in ids):
                winding['degenerate_or_unsampled'] += 1
                continue
            a, b, c = (sample_vertices[index] for index in ids)
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            cross = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
            dot = sum(cross[i] * (a[i+3] + b[i+3] + c[i+3]) for i in range(3))
            key = 'counterclockwise' if dot > 1e-10 else 'clockwise' if dot < -1e-10 else 'degenerate_or_unsampled'
            winding[key] += 1
        oriented = winding['clockwise'] + winding['counterclockwise']
        if oriented >= 100 and winding['counterclockwise'] / oriented > 0.9:
            errors.append(f'{relative}: sampled faces are wound backwards for OBJ8 (front must be clockwise)')
        for ref in animated_refs:
            bare = re.sub(r'\[\d+\]$', '', ref)
            if known is not None and bare not in known and ref not in custom_refs:
                errors.append(f'{relative}: animation dataref not in native catalog or verified provider: {ref}')
            elif not bare.startswith('sim/') and ref not in custom_refs:
                errors.append(f'{relative}: custom animation dataref has no verified provider: {ref}')
        for ref in manipulator_refs:
            bare = re.sub(r'\[\d+\]$', '', ref)
            if known is not None and bare not in known and ref not in custom_refs:
                errors.append(f'{relative}: manipulator dataref not in native catalog or verified provider: {ref}')
        if (manipulator_commands or manipulator_refs or manipulator_devices) \
                and not (flags.get(attachment_index, 0) & 2048):
            errors.append(f'{relative}: manipulators require the ACF cockpit-object flag')
        for device in sorted(manipulator_devices - packaged_devices):
            errors.append(f'{relative}: plugin cockpit-device manipulator has no packaged provider device: {device}')
        for command_name in sorted(manipulator_commands):
            if command_name.startswith('sim/'):
                if known_commands is not None and command_name not in known_commands:
                    errors.append(f'{relative}: manipulator command not in native catalog: {command_name}')
            elif command_name not in packaged_commands:
                errors.append(f'{relative}: aircraft-local manipulator command has no packaged provider: {command_name}')
        objects.append({'path': relative, 'vertices': verts, 'indices': indices,
                        'textures': textures, 'animation_datarefs': sorted(animated_refs),
                        'manipulator_datarefs': sorted(manipulator_refs),
                        'manipulator_commands': sorted(manipulator_commands),
                        'manipulator_devices': sorted(manipulator_devices),
                        'winding_sample': winding})
    if not (root / 'A380X-XP12_cockpit.obj').is_file():
        warnings.append('Missing native cockpit marker')
    if not (root / 'cockpit/-PANELS-/Panel.png').is_file():
        warnings.append('Missing transparent custom panel; generic 2-D artwork may be shown')
    if len(paths) < 30:
        warnings.append('Partial asset build: check that detailed cockpit and all gear assemblies are included')
    errors.extend(aerodynamic_wing_errors(text))
    errors.extend(trim_configuration_errors(text))
    fmod_errors, fmod_warnings = validate_fmod_package(root)
    errors.extend(fmod_errors)
    warnings.extend(fmod_warnings)
    return {'static_validation_only': True, 'aircraft': str(root), 'errors': errors,
            'warnings': warnings, 'native_panel': native_panel,
            'objects': objects, 'passed': not errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--datarefs', type=Path)
    parser.add_argument('--commands', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    result = validate(args.root, args.datarefs, args.commands)
    encoded = json.dumps(result, indent=2) + '\n'
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded)
    print(json.dumps({k: v for k, v in result.items() if k != 'objects'}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
