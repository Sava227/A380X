#!/usr/bin/env python3
"""A380 hydraulics: remap X-Plane's native pumps in the ACF to the A380 layout (docs/HYDRAULICS.md section 3).

Native system 1 = GREEN: engines 1+2 and electric pump 1. Native system 2 = YELLOW: engines 3+4 and electric
pump 2. (Plane Maker bitmask per pump: 1 = system 1, 2 = system 2.) Previously engines 1/4 fed system 1,
2/3 system 2, and electric pump 1 fed both.

Usage: acf_hyd_remap.py <aircraft.acf> [--check]   (edits in place; prints the changes)
"""
import sys
from pathlib import Path

WANT = {
    b"P acf/_hyd_engn_pump_0123/0 ": b"1",
    b"P acf/_hyd_engn_pump_0123/1 ": b"1",
    b"P acf/_hyd_engn_pump_0123/2 ": b"2",
    b"P acf/_hyd_engn_pump_0123/3 ": b"2",
    b"P acf/_hyd_ele_pump_0123/0 ": b"1",
    b"P acf/_hyd_ele_pump_0123/1 ": b"2",
}


def main() -> int:
    path = Path(sys.argv[1])
    check = "--check" in sys.argv
    data = path.read_bytes()
    eol = b"\r\n" if b"\r\n" in data[:4096] else b"\n"
    lines = data.split(eol)
    found = {key: 0 for key in WANT}
    changes = []
    for i, line in enumerate(lines):
        for key, value in WANT.items():
            if line.startswith(key):
                found[key] += 1
                if line[len(key):] != value:
                    changes.append((line.decode(), (key + value).decode()))
                    lines[i] = key + value
    missing = [key.decode() for key, n in found.items() if n != 1]
    if missing:
        print("ERROR: expected exactly one line for:", missing)
        return 2
    for old, new in changes:
        print(f"{old!r} -> {new!r}")
    if check:
        print("needs change" if changes else "already remapped")
        return 1 if changes else 0
    if changes:
        path.write_bytes(eol.join(lines))
    print(f"{len(changes)} line(s) changed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
