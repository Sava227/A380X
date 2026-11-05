#!/usr/bin/env python3
"""Run validate_aircraft.py on the live aircraft with selected files replaced by staged copies.

The project volume has no room for a scratch copy of the aircraft (objects alone are > 1 GB), so the
staged files are overlaid in-process instead: every pathlib read of an overlaid aircraft path (open,
read_text, read_bytes, is_file, exists) is served from the staged file. Nothing is written to the
aircraft. Typical use, with the files an installation would write (install_xp_kit_cockpit.py
--check-only --stage-dir):

  validate_aircraft_overlay.py <aircraft> --stage-dir <stage> \\
      --file plugins/A380XAvionics/mac_x64/A380XAvionics.xpl=<build>/mac.xpl \\
      --file plugins/A380XAvionics/build-report.json=<build>/build-report.json --report out.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_aircraft  # noqa: E402

OVERLAY: dict[Path, Path] = {}
_open, _read_text, _read_bytes = Path.open, Path.read_text, Path.read_bytes
_is_file, _exists = Path.is_file, Path.exists


def _target(path: Path) -> Path:
    try:
        return OVERLAY.get(Path(path).absolute(), path)
    except (OSError, ValueError):
        return path


def install(aircraft: Path, files: dict[str, Path]) -> None:
    for relative, staged in files.items():
        if Path(relative).is_absolute() or '..' in Path(relative).parts:
            raise ValueError('Overlay paths are aircraft-relative: ' + relative)
        if not _is_file(staged):
            raise ValueError('Missing staged file: ' + str(staged))
        OVERLAY[(aircraft / relative).absolute()] = staged.absolute()
    Path.open = lambda self, *a, **k: _open(_target(self), *a, **k)
    Path.read_text = lambda self, *a, **k: _read_text(_target(self), *a, **k)
    Path.read_bytes = lambda self: _read_bytes(_target(self))
    Path.is_file = lambda self: _is_file(_target(self))
    Path.exists = lambda self, *a, **k: _exists(_target(self), *a, **k)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('aircraft', type=Path)
    parser.add_argument('--stage-dir', type=Path, help='directory of aircraft-relative staged files')
    parser.add_argument('--file', action='append', default=[], help='RELATIVE=STAGED (repeatable)')
    parser.add_argument('--datarefs', type=Path)
    parser.add_argument('--commands', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    aircraft = args.aircraft.absolute()
    files: dict[str, Path] = {}
    if args.stage_dir:
        for staged in args.stage_dir.rglob('*'):
            if staged.is_file() and not staged.name.startswith('._'):
                files[str(staged.relative_to(args.stage_dir))] = staged
    for item in args.file:
        relative, _, staged = item.partition('=')
        files[relative] = Path(staged)
    install(aircraft, files)
    result = validate_aircraft.validate(aircraft, args.datarefs, args.commands)
    result['overlay'] = sorted(files)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'objects'}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
