"""Replace Euler-decomposed fan revolutions with their exact fixed spin axis.

The glTF fan clips contain 0/90/180/270/360-degree quaternion samples.
XPlane2Blender decomposes them into three Euler tables whose primary angle
wraps from +180 to -90. Linear OBJ interpolation would reverse the fan between
these samples. Collapse only the verified native fan-angle table triplets to a
single 0..360 rotation followed by the unchanged initial orientation.
"""
from __future__ import annotations

import math
from pathlib import Path
import re

from obj8_preview import IDENTITY, multiply, rotation

def obj8_fmt(value: float) -> str:
    """Fixed-point OBJ8 number (same rule as tools/obj8_numbers.py): X-Plane cannot read exponent notation."""
    text = f"{float(value):.10f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


FAN_REF = re.compile(r"sim/flightmodel2/engines/prop_rotation_angle_deg\[([0-3])\]")


def transpose_rotation(matrix):
    return tuple(matrix[c * 4 + r] if r < 3 and c < 3 else IDENTITY[r * 4 + c]
                 for r in range(4) for c in range(4))


def normalize_text(text: str) -> tuple[str, int]:
    lines = text.splitlines(keepends=True)
    output, index, changed = [], 0, 0
    while index < len(lines):
        words = lines[index].split()
        if len(words) != 5 or words[0] != "ANIM_rotate_begin" or not FAN_REF.fullmatch(words[-1]):
            output.append(lines[index]); index += 1; continue
        # Already-normalized fixed-axis table is marked for stable rebuilding.
        if output and output[-1].strip() == "# A380X continuous fan rotation":
            output.append(lines[index]); index += 1; continue
        reference = words[-1]
        tables, end = [], index
        for _ in range(3):
            header = lines[end].split()
            if len(header) != 5 or header[0] != "ANIM_rotate_begin" or header[-1] != reference:
                raise ValueError(f"Expected three consecutive fan Euler tables for {reference}")
            axis, keys = tuple(map(float, header[1:4])), []
            end += 1
            while lines[end].split()[0] == "ANIM_rotate_key":
                keys.append(tuple(map(float, lines[end].split()[1:])))
                end += 1
            if lines[end].strip() != "ANIM_rotate_end" or [k[0] for k in keys] != [0.,90.,180.,270.,360.]:
                raise ValueError(f"Unexpected authored fan keyframes for {reference}")
            tables.append((axis, keys)); end += 1
        poses = []
        for key in range(5):
            pose = IDENTITY
            for axis, keys in tables:
                pose = multiply(pose, rotation(axis, keys[key][1]))
            poses.append(pose)
        delta = multiply(poses[1], transpose_rotation(poses[0]))
        axis = (delta[9]-delta[6], delta[2]-delta[8], delta[4]-delta[1])
        length = math.sqrt(sum(value * value for value in axis))
        if length < 1e-8:
            raise ValueError("Fan quarter-turn has no stable rotation axis")
        axis = tuple(value / length for value in axis)
        for angle, pose in zip((0.,90.,180.,270.,360.), poses):
            expected = multiply(rotation(axis, angle), poses[0])
            if max(abs(a-b) for a,b in zip(expected,pose)) > 2e-6:
                raise ValueError(f"{reference}: authored fan is not a fixed-axis revolution")
        indent = lines[index][:len(lines[index])-len(lines[index].lstrip())]
        rows = ["# A380X continuous fan rotation",
                "ANIM_rotate_begin\t" + "\t".join(obj8_fmt(v) for v in axis) + "\t" + reference,
                "ANIM_rotate_key\t0\t0", "ANIM_rotate_key\t360\t360", "ANIM_rotate_end"]
        for original_axis, keys in tables:
            fields = (*original_axis, keys[0][1], keys[0][1])
            rows.append("ANIM_rotate\t" + "\t".join(obj8_fmt(v) for v in fields))
        output.extend(indent + line + "\n" for line in rows)
        index, changed = end, changed + 1
    return "".join(output), changed


def normalize_engine_fan_rotation(path: Path) -> dict:
    original = path.read_text()
    normalized, changed = normalize_text(original)
    if changed:
        path.write_text(normalized)
    return {"continuous_fan_rotations": changed}
