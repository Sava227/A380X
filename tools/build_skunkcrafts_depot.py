#!/usr/bin/env python3
"""Generate and verify SkunkCrafts v3/v4 metadata for A380X-XP12."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
import stat
import sys
import tempfile
import unicodedata
import zlib


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "output/A380X-XP12"
MODULE = "https://github.com/Sava227/A380X/raw/refs/heads/skunkcrafts/output/A380X-XP12/"
NAME = "A380X XP12 Port (Development)"
LFS_HEADER = b"version https://git-lfs.github.com/spec/v1\n"
CONTROL = {
    "skunkcrafts_updater.cfg", "skunkcrafts_updater_config.txt",
    "skunkcrafts_updater_whitelist.txt", "skunkcrafts_updater_sizeslist.txt",
    "skunkcrafts_updater_blacklist.txt", "skunkcrafts_updater_ignore.txt",
    "skunkcrafts_updater_ignorelist.txt", "skunkcrafts_updater_oncelist.txt",
    "skunkcrafts_updater_beta.cfg",
}
EXCLUDED = {
    "A380X-XP12_prefs.txt", "large-assets-animation-inventory.json",
    "runtime-smoke-test.json", "source-animation-inventory.json",
    "plugins/A380XAvionics/build-report.json",
    # SkunkCrafts does not preserve POSIX execute bits. Do not repair this
    # helper until the aircraft plugin can restore its mode before launch.
    "plugins/A380XAvionics/mac_x64/A380XDisplayHost",
    "plugins/A380XAvionics/mac_x64/xp-kit-build-report.json",
}
ONCE = {"X-Camera_A380X-XP12.csv", "fcu-calibration.ini"}


class DepotError(RuntimeError):
    pass


@dataclass(frozen=True)
class Row:
    path: str
    crc: int
    size: int


def safe_path(value: str) -> str:
    pure = PurePosixPath(value)
    if (not value or value != value.strip() or "|" in value or "\r" in value
            or "\n" in value or "\\" in value or pure.is_absolute()
            or any(part in {"", ".", ".."} for part in pure.parts)
            or str(pure) != value):
        raise DepotError(f"unsafe manifest path: {value!r}")
    return value


def collision_key(value: str) -> str:
    return unicodedata.normalize("NFC", value).casefold()


def excluded(value: str) -> bool:
    parts = PurePosixPath(value).parts
    return (value in EXCLUDED or parts[0] in {"build", "skunkcrafts"}
            or any(part.startswith(".") for part in parts)
            or PurePosixPath(value).name in CONTROL)


def crc32(path: Path) -> int:
    value = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            value = zlib.crc32(chunk, value)
    return value & 0xFFFFFFFF


def scan(package: Path) -> list[Row]:
    if not package.is_dir():
        raise DepotError(f"package does not exist: {package}")
    files: list[tuple[str, Path]] = []
    for path in package.rglob("*"):
        if path.is_symlink():
            raise DepotError(f"symlink is not publishable: {path}")
        mode = path.stat().st_mode
        if stat.S_ISDIR(mode):
            continue
        if not stat.S_ISREG(mode):
            raise DepotError(f"non-regular file is not publishable: {path}")
        relative = safe_path(path.relative_to(package).as_posix())
        if not excluded(relative):
            files.append((relative, path))
    files.sort()
    seen: dict[str, str] = {}
    for relative, _ in files:
        previous = seen.setdefault(collision_key(relative), relative)
        if previous != relative:
            raise DepotError(f"path collision: {previous!r} and {relative!r}")
    rows: list[Row] = []
    for relative, path in files:
        with path.open("rb") as stream:
            if stream.read(len(LFS_HEADER)) == LFS_HEADER:
                raise DepotError(f"Git LFS pointer found instead of payload: {relative}")
        rows.append(Row(relative, -1 if relative in ONCE else crc32(path), path.stat().st_size))
    return rows


def config(version: str, module: str = MODULE) -> str:
    if (not version or any(char in version for char in "|\r\n")
            or not module.startswith("https://") or not module.endswith("/")):
        raise DepotError("invalid version or module URL")
    return "\n".join((
        f"name|{NAME}", f"module|{module}", f"version|{version}",
        "locked|false", "zone|custom", "disabled|false", "liveries|false",
        "readme|README.txt", "changelog|changelog.txt", "",
    ))


def blacklist(path: Path) -> str:
    if not path.exists():
        return ""
    entries = [safe_path(line.strip()) for line in path.read_text(encoding="utf-8-sig").splitlines()
               if line.strip()]
    if any(PurePosixPath(entry).name in CONTROL for entry in entries):
        raise DepotError("blacklist cannot delete updater control files")
    return "".join(f"{entry}\n" for entry in entries)


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n",
                                     dir=path.parent, delete=False) as stream:
        stream.write(text)
        temporary = Path(stream.name)
    temporary.replace(path)


def build(package: Path, version: str, module: str, check: bool) -> list[Row]:
    package = package.resolve()
    version_text = version.strip() + "\n"
    package_version = package / "VERSION"
    if check:
        if not package_version.is_file() or package_version.read_text() != version_text:
            raise DepotError("package VERSION is missing or stale")
    else:
        write(package_version, version_text)
    changelog = (package / "changelog.txt").read_text(encoding="utf-8-sig").splitlines()
    if next((line.strip() for line in changelog if line.strip()), "") != version.strip():
        raise DepotError("changelog first version does not match VERSION")
    rows = scan(package)
    outputs = {
        "skunkcrafts_updater.cfg": config(version.strip(), module),
        "skunkcrafts_updater_whitelist.txt": "".join(f"{r.path}|{r.crc}\n" for r in rows),
        "skunkcrafts_updater_sizeslist.txt": "".join(f"{r.path}|{r.size}\n" for r in rows),
        "skunkcrafts_updater_blacklist.txt": blacklist(package / "skunkcrafts_updater_blacklist.txt"),
    }
    stale = []
    for name, text in outputs.items():
        destination = package / name
        if check:
            if not destination.is_file() or destination.read_text() != text:
                stale.append(name)
        else:
            write(destination, text)
    if stale:
        raise DepotError("stale metadata: " + ", ".join(stale))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, default=PACKAGE)
    parser.add_argument("--version")
    parser.add_argument("--module", default=MODULE)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        version = args.version or (ROOT / "VERSION").read_text().strip()
        rows = build(args.package, version, args.module, args.check)
    except (DepotError, OSError, UnicodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"{'verified' if args.check else 'generated'} {len(rows)} files, "
          f"{sum(row.size for row in rows)} bytes, version {version}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
