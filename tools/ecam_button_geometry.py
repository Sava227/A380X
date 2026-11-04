"""ENG-first ECAM pushbutton: retain source mesh/UVs and animate its draw calls.

This module only returns preflighted text; the aircraft installer owns writes.
It never exports a second button shell or rebuilds unrelated cockpit geometry.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

from build_cockpit_controls import (
    FCU_ANNUNCIATOR_FIRST_CELL, FCU_LABEL_COLUMNS, FCU_LABEL_ROWS,
)
from patch_center_right_animations import animation_block, source_targets, wrap_geometry
from patch_efis_animations import vertex_payload
from sd_cockpit_device import append_geometry, MARKER as SD_MARKER
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

PRESS_REF = "a380x/xp12/ecam/buttons/eng/press_ratio"
SELECTED_REF = "a380x/xp12/ecam/buttons/eng/selected"
COMMAND = "a380x/xp12/ecam/sd/engine"
MARKER = "# A380X ECAM ENG BUTTON V1 "
LAMP_MARKER = "# A380X ECAM ENG SELECTION V1"
PANEL_DIR = "objects/interior/cockpit-panels/"
CONTROLS_PATH = "objects/interior/cockpit-controls/a380x_cockpit_controls.obj"
BODY_PATH = PANEL_DIR + "panel_a380_cockpit_pedestal_knobs02_4k.obj"
LEGEND_PATH = PANEL_DIR + "decal_a380x_cockpit_decal_emis.obj"
SELECTION_PATH = PANEL_DIR + "decal_a380x_cockpit_decal_lovis.obj"
ENG_OBJECT_PATHS = (BODY_PATH, LEGEND_PATH, SELECTION_PATH, CONTROLS_PATH)


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def button_targets(source: Path, press_distance: float | None = None):
    """Decode the authored local-axis stroke, never guess a global axis.

    The retained caption repair lifted SEQ1 2 mm outward. Its acceptance box
    permits that existing repair; the six-index exact-count check still makes
    a wrong/ambiguous material export fail before installation.
    """
    targets = source_targets(source, motions=((
        "PUSH_ECAM_ENG", "PUSH_ECAM_ENG", "ecam/buttons/eng/press_ratio", 1),))
    if {t["name"] for t in targets} != {
            "PUSH_ECAM_ENG", "PUSH_ECAM_ENG_SEQ1", "PUSH_ECAM_ENG_SEQ2"}:
        raise ValueError("ENG source mesh grouping changed; inspect it before installing")
    reference = next(t for t in targets if t["name"] == "PUSH_ECAM_ENG")
    if reference["kind"] != "translation" or len(reference["poses"]) != 2:
        raise ValueError("ENG source must contain a two-pose linear pressing stroke")
    if max(abs(v) for v in reference["poses"][0]) > 1e-8:
        raise ValueError("ENG first authored translation is not the rest pose")
    stroke = reference["poses"][1]
    distance = math.sqrt(sum(v*v for v in stroke))
    if not .0001 < distance < .005:
        raise ValueError("ENG source stroke is outside the inspected physical button range")
    if press_distance is not None:
        if not math.isfinite(press_distance) or not .0001 <= press_distance <= .003:
            raise ValueError("Configured press distance must be 0.1..3 mm")
        scale = press_distance / distance
        for target in targets:
            target["poses"] = [tuple(v*scale for v in pose) for pose in target["poses"]]
    for target in targets:
        if target["name"].endswith("_SEQ1"):
            target["high"] = tuple(v + (.0015 if axis == 1 else 0)
                                   for axis, v in enumerate(target["high"]))
    return targets


def parsed_geometry(text):
    vertices = [tuple(map(float, line.split()[1:])) for line in text.splitlines()
                if line.startswith("VT\t")]
    indices = [int(v) for line in text.splitlines()
               if line.startswith(("IDX\t", "IDX10\t")) for v in line.split()[1:]]
    return vertices, indices


def selection_face(text):
    """Use the actual exported ENG SEQ2 lamp, including any retained repairs."""
    lines = [line.strip() for line in text.splitlines()]
    marker = MARKER + "PUSH_ECAM_ENG_SEQ2"
    if lines.count(marker) != 1:
        raise ValueError("ENG selection face was not uniquely identified")
    after = lines[lines.index(marker)+1:]
    at, count = map(int, next(line for line in after if line.startswith("TRIS")).split()[1:3])
    if count != 6:
        raise ValueError("ENG selection indicator is not the inspected two-triangle face")
    vertices, indices = parsed_geometry(text)
    selected = indices[at:at+count]
    if len(set(selected)) != 4:
        raise ValueError("ENG selection face is not a four-corner quad")
    return vertices, selected


def add_selection_light(text, selection_text, target):
    """An independent state-gated emissive overlay, moving with the cap.

    Cell 16 is the existing solid-green FCU atlas cell. Its centre is opaque
    green (48,255,78,255) in both day and LIT textures; sampling only its centre
    avoids stretching a caption or introducing an extra texture/attachment.
    """
    if LAMP_MARKER in text:
        if text.count(LAMP_MARKER) != 1 or SELECTED_REF not in text:
            raise ValueError("Incomplete existing ENG selection-light patch")
        endpoint = "ANIM_trans_key\t1\t" + "\t".join(f"{obj8_fmt(v)}" for v in target["poses"][1])
        if endpoint not in text.split(LAMP_MARKER, 1)[1]:
            raise ValueError("Installed ENG lamp uses a different press distance")
        return text
    if "TEXTURE_LIT\t../../../cockpit_3d/fcu_labels_LIT.png" not in text:
        raise ValueError("Cockpit controls no longer use the inspected emissive atlas")
    if text.count(COMMAND) != 1 or not any(f"ATTR_manip_command\t{cursor}\t" + COMMAND in text for cursor in ("hand", "button")):
        raise ValueError("ENG must have exactly one ordinary click command target")
    source_vertices, source_indices = selection_face(selection_text)
    u = (FCU_ANNUNCIATOR_FIRST_CELL % FCU_LABEL_COLUMNS + .5) / FCU_LABEL_COLUMNS
    v = 1 - (FCU_ANNUNCIATOR_FIRST_CELL // FCU_LABEL_COLUMNS + .5) / FCU_LABEL_ROWS
    vertices, indices, remap = [], [], {}
    for index in source_indices:
        if index not in remap:
            source = source_vertices[index]
            point, normal = source[:3], source[3:6]
            if sum(a*b for a, b in zip(normal, target["poses"][1])) >= 0:
                raise ValueError("Authored press axis points out of the button face")
            lifted = tuple(point[a] + .00006*normal[a] for a in range(3))
            remap[index] = len(vertices)
            vertices.append("VT\t" + "\t".join(f"{obj8_fmt(value)}" for value in (*lifted, *normal, u, v)))
        indices.append(remap[index])
    block = animation_block(target, 0, 6, marker="# ECAM ENG LAMP MOTION ")
    commands = [
        "ATTR_no_cockpit", "ATTR_draw_enable", "ATTR_manip_none", "ATTR_poly_os\t2",
        f"ATTR_light_level\t0\t1\t{SELECTED_REF}", "ANIM_begin",
        f"ANIM_hide\t-0.5\t0.5\t{SELECTED_REF}", *block,
        "ANIM_end", "ATTR_light_level_reset", "ATTR_poly_os\t0", "ATTR_manip_none",
    ]
    result = append_geometry(text, vertices, indices, commands)
    # append_geometry is shared with SD installation and supplies its marker;
    # replace only the newly appended marker, never the existing SD patch.
    before, separator, after = result.rpartition("\n" + SD_MARKER + "\n")
    if not separator:
        raise ValueError("Failed to append ENG light geometry")
    return before + "\n" + LAMP_MARKER + "\n" + after


def patch_eng_geometry(source: Path, objects: dict[str, str], press_distance: float | None = None):
    """Preflight all four OBJ replacements, returning texts and hash metadata.

    The caller must stage/write every replacement only after this succeeds.
    Passing an already-patched aircraft is a byte-identical no-op. A changed
    stroke on a previously patched aircraft is deliberately rejected.
    """
    missing = set(ENG_OBJECT_PATHS) - objects.keys()
    if missing:
        raise ValueError(f"Missing ENG cockpit objects: {sorted(missing)}")
    targets = button_targets(source, press_distance)
    changed, draw_reports = {}, []
    endpoint = "ANIM_trans_key\t1\t" + "\t".join(f"{obj8_fmt(v)}" for v in targets[0]["poses"][1])
    for relative in ENG_OBJECT_PATHS[:-1]:
        original = objects[relative]
        patched, report, gear_count = wrap_geometry(original, Path(relative).name, targets, marker=MARKER)
        if gear_count or vertex_payload(original) != vertex_payload(patched):
            raise ValueError("ENG wrapping changed unrelated geometry or texture data")
        if endpoint not in patched or PRESS_REF not in patched:
            raise ValueError("ENG existing animation does not match the configured stroke")
        changed[relative] = patched
        draw_reports.extend(report)
    changed[CONTROLS_PATH] = add_selection_light(objects[CONTROLS_PATH], changed[SELECTION_PATH], targets[0])
    report = {
        "button": "ENG", "source": str(source.resolve()),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "press_dataref": PRESS_REF, "selected_dataref": SELECTED_REF, "command": COMMAND,
        "inward_vector_m": list(targets[0]["poses"][1]),
        "press_distance_m": math.sqrt(sum(v*v for v in targets[0]["poses"][1])),
        "rest_pivot_m": list(targets[0]["pivot"]),
        "pressed_pivot_m": [p+v for p, v in zip(targets[0]["pivot"], targets[0]["poses"][1])],
        "source_nodes": [t["name"] for t in targets],
        "source_mesh_indices": {t["name"]: t["count"] for t in targets},
        "animated_draws": draw_reports,
        "objects": [{"file": key, "before_sha256": digest(objects[key]),
                     "sha256": digest(value)} for key, value in changed.items()],
        "selection_atlas": "cockpit_3d/fcu_labels.png",
        "selection_atlas_lit": "cockpit_3d/fcu_labels_LIT.png",
        "selection_uv": [.0625, .6875],
        "selection_overlay_indices": 6,
        "original_mesh_geometry_unchanged": True,
        "click_target_unchanged": True,
    }
    return changed, report


def prepare_eng_geometry(source: Path, aircraft: Path, press_distance: float | None = None):
    return patch_eng_geometry(source, {name: (aircraft/name).read_text()
                                      for name in ENG_OBJECT_PATHS}, press_distance)
