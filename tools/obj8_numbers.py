#!/usr/bin/env python3
"""OBJ8 numbers without exponent notation.

X-Plane's OBJ8 reader does not understand scientific notation: it stops at the "e", so
`4.694689366e-07` is read as 4.694689366 and `3.187809668e-10` as 3.19. Python's `:g` / `:.9g` /
`:.10g` formats and `repr()` write exactly such numbers for tiny values (an axis component of
1e-7, a translation of 1e-10). In the sim this turned the DU, autobrake and A-SKID rotation axes
sideways, threw the flap/speed-brake levers metres away when lifted and the ELEC pushbuttons
3 m when pressed, and misplaced click meshes.

`fmt()` is the formatter every OBJ writer should use. The CLI checks or repairs existing OBJs:
only whole numeric tokens written with an exponent change, every other byte stays the same.

    python3 tools/obj8_numbers.py --check <aircraft or obj> ...   # exit 1 if any exponent remains
    python3 tools/obj8_numbers.py --fix   <aircraft or obj> ...   # rewrite them in fixed point
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

DECIMALS = 10
EXPONENT_TOKEN = re.compile(r'(?<!\S)[-+]?(?:\d+\.?\d*|\.\d+)[eE][-+]?\d+(?!\S)')


def fmt(value: float, decimals: int = DECIMALS) -> str:
    """Fixed-point text for an OBJ8 number: no exponent, no trailing zeros, no '-0'."""
    value = float(value)
    if value != value or value in (float('inf'), float('-inf')):
        raise ValueError(f'OBJ8 numbers must be finite, got {value}')
    text = f'{value:.{decimals}f}'
    if '.' in text:
        text = text.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '', '+0') else text


def has_exponent(line: str) -> bool:
    stripped = line.lstrip()
    return not stripped.startswith('#') and EXPONENT_TOKEN.search(line) is not None


def fix_line(line: str) -> tuple[str, int]:
    """Rewrite the exponent tokens of one non-comment line; returns (line, tokens changed)."""
    if line.lstrip().startswith('#'):
        return line, 0
    changed = 0

    def replace(match: re.Match) -> str:
        nonlocal changed
        changed += 1
        return fmt(float(match.group(0)))

    return EXPONENT_TOKEN.sub(replace, line), changed


def fix_text(text: str) -> tuple[str, int]:
    total = 0
    out = []
    for line in text.splitlines(keepends=True):
        fixed, n = fix_line(line)
        total += n
        out.append(fixed)
    return ''.join(out), total


def obj_files(paths):
    for path in paths:
        path = Path(path)
        candidates = [path] if path.is_file() else sorted(path.rglob('*.obj'))
        for obj in candidates:
            if not obj.name.startswith('._'):
                yield obj


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--fix', action='store_true')
    parser.add_argument('paths', nargs='+', type=Path)
    args = parser.parse_args()
    files = tokens = 0
    for obj in obj_files(args.paths):
        data = obj.read_bytes()
        text = data.decode('latin-1')          # byte-preserving round trip
        fixed, count = fix_text(text)
        if not count:
            continue
        files += 1
        tokens += count
        print(f'{"fixed" if args.fix else "exponent notation"}: {obj} ({count} numbers)')
        if args.fix:
            tmp = obj.with_name(obj.name + '.tmp-obj8-numbers')
            tmp.write_bytes(fixed.encode('latin-1'))
            tmp.replace(obj)
    print(f'{tokens} numbers in exponent notation in {files} OBJ files' + (' rewritten' if args.fix else ''))
    return 1 if (args.check and tokens) else 0


if __name__ == '__main__':
    sys.exit(main())
