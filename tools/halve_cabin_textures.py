#!/usr/bin/env python3
"""Halve the passenger cabin, seat and cargo-hold textures to save texture memory for the flight deck.

X-Plane 12 downscales every texture, the flight deck's included, once total texture demand outgrows the
memory it thinks it has (Log.txt "Target scale moved to 2/4/8"), which made the panel captions illegible.
The cabin and cargo hold are rarely seen up close, so their textures give memory back first: every texture
used only by objects under objects/interior/cabin*, objects/interior/cargo (not named *cockpit*) is resized
to half (Lanczos), never below --floor px; anything a cockpit or exterior object also uses is left alone.
halved-cabin.json records each result, so a re-run changes nothing; normal-map-manifest.json entries are
updated so halve_normal_maps.py does not halve those normal maps again. Afterwards re-run
tools/convert_textures_to_dds.py so the DDS copies follow.

    python3 tools/halve_cabin_textures.py --aircraft "<X-Plane>/Aircraft/A380X-XP12" --backup <dir>
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import shutil
from pathlib import Path

from PIL import Image

from convert_textures_to_dds import sha256

STATE = "halved-cabin.json"
TEXTURE_LINES = ("TEXTURE", "TEXTURE_LIT", "TEXTURE_NORMAL", "TEXTURE_MAP")


def attached_objects(acf_text: str) -> list[str]:
    return sorted({"objects/" + line.split()[2] for line in acf_text.splitlines()
                   if line.startswith("P _obja/") and "/_v10_att_file_stl " in line and len(line.split()) > 2})


def texture_users(root: Path, objects: list[str]) -> dict[str, set[str]]:
    users = collections.defaultdict(set)
    for obj in objects:
        path = root / obj
        if not path.is_file():
            continue
        with open(path, errors="replace") as f:
            for line in f:
                if line.startswith(("POINT_COUNTS", "VT")):
                    break
                field = line.split()
                if len(field) >= 2 and field[0] in TEXTURE_LINES:
                    target = os.path.normpath(os.path.join(os.path.dirname(obj), field[-1]))
                    users[target.replace(os.sep, "/")].add(obj)
    return users


def is_cabin_object(obj: str) -> bool:
    return ("/cabin" in obj or "/cargo" in obj) and "cockpit" not in os.path.basename(obj)


def cabin_only(users: dict[str, set[str]]) -> list[str]:
    return sorted(t for t, us in users.items() if us and all(is_cabin_object(u) for u in us))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", type=Path, required=True)
    ap.add_argument("--backup", type=Path)
    ap.add_argument("--floor", type=int, default=512)
    args = ap.parse_args()
    root = args.aircraft
    objects = attached_objects((root / "A380X-XP12.acf").read_text(errors="replace"))
    state_path = root / STATE
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    manifest_path = root / "normal-map-manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    done = before = after = 0
    for rel in cabin_only(texture_users(root, objects)):
        path = root / rel
        if not path.is_file() or state.get(rel) == sha256(path):
            continue
        with Image.open(path) as im:
            w, h = im.size
            if min(w, h) // 2 < args.floor:
                continue
            if args.backup:
                (args.backup / rel).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, args.backup / rel)
            small = im.resize((w // 2, h // 2), Image.Resampling.LANCZOS)
        small.save(path)
        state[rel] = sha256(path)
        if manifest is not None and rel in manifest:
            manifest[rel]["halved_sha256"] = state[rel]
        done += 1; before += w * h; after += (w // 2) * (h // 2)
    state_path.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
    if manifest is not None:
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"halved {done} cabin/cargo textures: {before / 1e6:.1f} -> {after / 1e6:.1f} Mpx")
    print("now re-run tools/convert_textures_to_dds.py on the aircraft so the DDS copies follow")


if __name__ == "__main__":
    main()
