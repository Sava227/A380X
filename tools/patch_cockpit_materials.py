#!/usr/bin/env python3
"""Restore FBW cockpit atlases and patch the existing split X-Plane 12 OBJs.

Requires Pillow and this project's blender/convert_ktx2_textures.py. Dry-run
is the default; --apply installs the prepared changes with a rollback manifest.

Example (run from xplane-port):
  python3 tools/patch_cockpit_materials.py
  # Add --apply to install, or --restore /path/to/backup/manifest.json to undo.
  # --aircraft and --source-gltf override this project's default paths.

The source's label atlas already contains the FBW words and symbols. Retaining
the authored vertex UVs is essential; this tool does not invent glyph locations,
move vertices, add missing meshes, or replace working instrument screen regions.
The earlier 2 mm decal displacement, if present, is retained without repeating it.
An unchanged UV hash is a mapping-integrity check, not proof of in-simulator
visibility. Reload the aircraft and inspect the cockpit after applying.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
import re
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import unquote

from PIL import Image, ImageChops, ImageOps
from audit_cockpit_uvs import audit_cockpit_uvs

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (PROJECT.parent / "upstream-aircraft-large-files/fbw-a380x/src/base/"
                  "flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/"
                  "attachments/flybywire/Part_Interior_Cockpit/model/A380_COCKPIT_LOD00.gltf")
OBJ_DIR = Path("objects/interior/cockpit-panels")
ASSET_DIR = Path("textures/interior/cockpit-material-fix")
TARGETS = (
    ("decal_a380x_cockpit_decal_emis.obj", "A380X_COCKPIT_DECAL_EMIS", "labels"),
    ("panel_a380_cockpit_mip01_8k.obj", "A380_COCKPIT_MIP01_8k", "mip"),
    ("panel_a380_cockpit_mip02_4k.obj", "A380_COCKPIT_MIP02_4k", "mip"),
    ("panel_a380_cockpit_mip04_4k.obj", "A380_COCKPIT_MIP04_4k", "mip"),
    ("panel_a380_cockpit_glareshield_4k.obj", "A380_COCKPIT_GLARESHIELD_4k", "matte"),
    ("panel_a380_cockpit_fcu_4k.obj", "A380_COCKPIT_FCU_4k", "fcu"),
)


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def png_bytes(image: Image.Image) -> bytes:
    out = io.BytesIO()
    image.save(out, format="PNG", optimize=True)
    return out.getvalue()


def fields(text: str):
    for line in text.splitlines():
        words = line.split("#", 1)[0].split()
        if words:
            yield words


def inspect_obj(text: str) -> dict:
    records = list(fields(text))
    if [r[0] for r in records[:3]] not in (["I", "800", "OBJ"], ["A", "800", "OBJ"]):
        raise ValueError("Expected an X-Plane OBJ8 file")
    if any(r[0] == "ATTR_reset" for r in records) or sum(r[0] == "ATTR_LOD" for r in records) > 1:
        raise ValueError("Target resets material state or has multiple LODs; requires a per-section patch")
    vertices, indices, draws, counts = [], [], [], []
    protected = []
    depth = 0
    for row in records:
        cmd = row[0]
        if cmd.startswith("ATTR_cockpit") or cmd == "TEXTURE_COCKPIT":
            raise ValueError("Target contains cockpit screen mapping; refuse whole-object material patch")
        if cmd == "VT":
            if len(row) != 9:
                raise ValueError("Invalid VT record")
            values = list(map(float, row[1:]))
            if not all(math.isfinite(x) for x in values):
                raise ValueError("Nonfinite vertex/normal/UV value")
            vertices.append(values)
        elif cmd in ("IDX", "IDX10"):
            if len(row) != (2 if cmd == "IDX" else 11):
                raise ValueError("Invalid index record")
            indices.extend(map(int, row[1:]))
        elif cmd == "TRIS":
            draws.append(tuple(map(int, row[1:])))
        elif cmd == "POINT_COUNTS":
            counts.append(tuple(map(int, row[1:])))
        if cmd == "ANIM_begin":
            depth += 1
        elif cmd == "ANIM_end":
            depth -= 1
            if depth < 0:
                raise ValueError("Unbalanced animation scope")
        if cmd in {"VT", "IDX", "IDX10", "TRIS", "POINT_COUNTS"} or cmd.startswith(("ANIM_", "ATTR_manip")):
            protected.append(row)
    if depth or not vertices or not draws or len(counts) != 1:
        raise ValueError("Incomplete geometry or unbalanced animations")
    if counts[0] != (len(vertices), 0, 0, len(indices)):
        raise ValueError("POINT_COUNTS does not match triangle-only target")
    if any(i < 0 or i >= len(vertices) for i in indices):
        raise ValueError("Index outside vertex table")
    for draw in draws:
        if len(draw) != 2 or draw[0] < 0 or draw[1] < 0 or draw[1] % 3 or sum(draw) > len(indices):
            raise ValueError("TRIS range outside index table")
    uvs = [v[6:8] for v in vertices]
    return {
        "vertices": len(vertices), "triangles_drawn": sum(n for _, n in draws) // 3,
        "uv_bounds": [[min(v[i] for v in uvs), max(v[i] for v in uvs)] for i in (0, 1)],
        "uv_sha256": digest(json.dumps(uvs).encode()),
        "geometry_animation_sha256": digest(json.dumps(protected).encode()),
        "vertices_data": vertices, "indices": indices, "draws": draws,
    }


def texture_references(text: str):
    for row in fields(text):
        if row[0] in {"TEXTURE", "TEXTURE_LIT", "TEXTURE_NORMAL"}:
            yield row[0], " ".join(row[1:])
        elif row[0] == "TEXTURE_MAP":
            yield " ".join(row[:2]), " ".join(row[2:])


def patch_obj(text: str, textures: dict[str, str], kind: str, light_dataref: str | None) -> str:
    """Only replace material state; vertex, UV, draw and animation records survive."""
    before = inspect_obj(text)
    if light_dataref and not re.fullmatch(r"[A-Za-z0-9_./\[\]-]+", light_dataref):
        raise ValueError("Invalid light-level dataref")
    remove = {"TEXTURE", "TEXTURE_LIT", "TEXTURE_NORMAL", "TEXTURE_MAP",
              "NORMAL_METALNESS", "GLOBAL_specular", "ATTR_shiny_rat"}
    if kind == "labels":
        remove |= {"ATTR_blend", "ATTR_no_blend", "GLOBAL_no_blend", "ATTR_poly_os",
                   "ATTR_light_level", "ATTR_light_level_reset"}
    lines = []
    for line in text.splitlines():
        row = line.split()
        if row and row[0] in remove:
            continue
        lines.append(line)
    obj_line = next(i for i, line in enumerate(lines) if line.strip() == "OBJ")
    header = [f"{key} {value}" for key, value in textures.items()]
    if "TEXTURE_MAP material_gloss" in textures:
        header.append("NORMAL_METALNESS")
    lines[obj_line + 1:obj_line + 1] = header
    first_draw = next(i for i, line in enumerate(lines) if line.lstrip().startswith("TRIS"))
    state = ["ATTR_shiny_rat 0.05" if kind in {"labels", "matte"} else "ATTR_shiny_rat 1.0"]
    if kind == "labels":
        # This exported legend mesh already has physical surface separation.
        # Polygon-offset mode suppresses depth writes: a later panel can erase
        # the lettering. Keep ordinary depth writes for the lifted mesh.
        # Blended, not alpha-tested: at the default view a caption is a few pixels
        # tall, and an alpha cut-off turns its minified letters into blobs and dots.
        state += ["ATTR_blend", "ATTR_poly_os 0"]
        if light_dataref:
            state.append(f"ATTR_light_level 0 1 {light_dataref}")
    lines[first_draw:first_draw] = state
    result = "\n".join(lines) + "\n"
    after = inspect_obj(result)
    if after["geometry_animation_sha256"] != before["geometry_animation_sha256"]:
        raise AssertionError("Patch changed geometry, UVs, indices, or animation records")
    return result


class SourceTextures:
    def __init__(self, source: Path, asset_root: Path, max_size: int):
        self.doc = json.loads(source.read_text())
        self.asset_root = asset_root
        self.max_size = max_size
        self.cache = {}
        self.provenance = {}
        self.materials = {m["name"]: m for m in self.doc["materials"]}

    def load(self, spec: dict) -> Image.Image:
        if spec.get("texCoord", 0) != 0 or spec.get("extensions", {}).get("KHR_texture_transform"):
            raise ValueError("Source uses another UV set or texture transform; explicit remapping is required")
        tex = self.doc["textures"][spec["index"]]
        idx = tex.get("extensions", {}).get("MSFT_texture_dds", {}).get("source", tex.get("source"))
        if idx is None:
            raise ValueError("Source texture has no image")
        if idx not in self.cache:
            name = unquote(self.doc["images"][idx]["uri"]).replace("\\", "/").split("/")[-1]
            matches = [p for p in self.asset_root.rglob(name) if not p.name.startswith("._")]
            if len(matches) != 1:
                raise ValueError(f"Expected one source asset {name}, found {len(matches)}")
            path = matches[0]
            raw = path.read_bytes()
            if path.suffix.lower() == ".ktx2":
                sys.path.insert(0, str(PROJECT / "blender"))
                from convert_ktx2_textures import ktx2_to_dds
                dds, _ = ktx2_to_dds(path)
                with Image.open(io.BytesIO(dds)) as src:
                    im = src.convert("RGBA")
            else:
                with Image.open(path) as src:
                    im = src.convert("RGBA")
            source_size = im.size
            im.thumbnail((self.max_size, self.max_size), Image.Resampling.LANCZOS)
            self.cache[idx] = im
            self.provenance[str(idx)] = {"path": str(path), "sha256": digest(raw),
                                         "source_size": source_size, "output_size": im.size}
        return self.cache[idx].copy()


def tint_albedo(im: Image.Image, factor: list) -> Image.Image:
    bands = list(im.split())
    for c in range(3):
        if factor[c] == 1:
            continue
        lut = []
        for v in range(256):
            s = v / 255
            linear = s / 12.92 if s <= 0.04045 else ((s + .055) / 1.055) ** 2.4
            linear *= factor[c]
            s = 12.92 * linear if linear <= .0031308 else 1.055 * linear ** (1 / 2.4) - .055
            lut.append(round(255 * max(0, min(1, s))))
        bands[c] = bands[c].point(lut)
    if factor[3] != 1:
        bands[3] = bands[3].point([round(v * factor[3]) for v in range(256)])
    return Image.merge("RGBA", bands)


def make_materials(source: SourceTextures, material: dict, kind: str) -> dict[str, Image.Image]:
    pbr = material.get("pbrMetallicRoughness", {})
    if "baseColorTexture" not in pbr:
        raise ValueError(f"No source albedo for {material['name']}")
    albedo = tint_albedo(source.load(pbr["baseColorTexture"]), pbr.get("baseColorFactor", [1, 1, 1, 1]))
    # The source glareshield is already charcoal. Preserve wear and FCU paint;
    # change reflectance and gloss, rather than darkening the authored artwork.
    result = {"albedo": albedo}
    if material.get("normalTexture"):
        result["normal"] = source.load(material["normalTexture"])
    if kind in {"matte", "labels"}:
        # XP12 material_gloss: R=F0/metalness (zero deliberately suppresses
        # specularity here), G=gloss. Preserve source normals separately.
        result["material_gloss"] = Image.new("RGBA", (4, 4), (0, 20, 0, 255))
    elif kind == "fcu":
        from prepare_fcu_materials import convert_comp
        result["material_gloss"] = convert_comp(source.load(pbr["metallicRoughnessTexture"]),
            pbr.get("metallicFactor", 1), pbr.get("roughnessFactor", 1))
    elif pbr.get("metallicRoughnessTexture"):
        comp = source.load(pbr["metallicRoughnessTexture"])
        _, rough, metal, _ = comp.split()
        rough = rough.point([round(v * pbr.get("roughnessFactor", 1)) for v in range(256)])
        # X-Plane shares F0 and metalness. A dielectric needs F0 about .04;
        # map glTF metalness 0..1 onto .04..1 rather than black nonreflectance.
        metal = metal.point([round(255 * (.04 + .96 * v / 255 * pbr.get("metallicFactor", 1))) for v in range(256)])
        result["material_gloss"] = Image.merge("RGBA", (metal, ImageOps.invert(rough),
            Image.new("L", comp.size, 0), Image.new("L", comp.size, 255)))
    if kind == "labels" and material.get("emissiveTexture"):
        lit = source.load(material["emissiveTexture"])
        # X-Plane adds LIT alpha to albedo alpha. Keep transparency in the
        # albedo only; mask emissive RGB and write an RGB LIT texture.
        result["lit"] = Image.merge("RGB", tuple(ImageChops.multiply(b, lit.getchannel("A")) for b in lit.split()[:3]))
    return result


def label_uv_audit(info: dict, atlas: Image.Image) -> dict:
    # Texture t=0 is the bottom row; PNG row zero is the top row.
    verts, idx = info["vertices_data"], info["indices"]
    samples = []
    for start, count in info["draws"]:
        for i in range(start, start + count, 3):
            tri = [verts[j] for j in idx[i:i + 3]]
            center = [sum(v[k] for v in tri) / 3 for k in range(8)]
            if abs(center[0]) < .4 and 3.2 < center[1] < 3.9 and 31.4 < center[2] < 32.6:
                samples.append(center[6:8])
    if not samples:
        raise ValueError("No overhead decal triangles found in the expected FBW coordinate range")
    px = atlas.load()
    visible = 0
    for u, v in samples:
        if not 0 <= u <= 1 or not 0 <= v <= 1:
            raise ValueError("Overhead decal UV outside authored atlas")
        rgba = px[min(atlas.width - 1, int(u * atlas.width)),
                  min(atlas.height - 1, max(0, int((1 - v) * atlas.height)))]
        visible += rgba[3] >= 39 and max(rgba[:3]) > 128
    coverage = visible / len(samples)
    if coverage < .05:
        raise ValueError("Overhead UVs sample too little visible label ink; do not apply this atlas")
    return {"overhead_triangles_sampled": len(samples), "centroids_on_visible_ink": visible,
            "ink_fraction": round(coverage, 5),
            "limit": "Checks atlas orientation/content and preserves UVs; runtime alignment must be visually checked."}


def atomic_write(path: Path, data: bytes):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".cockpit-patch-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as out:
            out.write(data)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def install(root: Path, plan: dict[Path, bytes]) -> Path | None:
    changes = {p: data for p, data in plan.items() if not (root / p).is_file() or (root / p).read_bytes() != data}
    if not changes:
        return None
    backup = root / ".cockpit-material-backups" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    backup.mkdir(parents=True)
    entries = []
    for relative, data in changes.items():
        path = root / relative
        old = path.read_bytes() if path.exists() else None
        if old is not None:
            atomic_write(backup / "original" / relative, old)
        entries.append({"path": str(relative), "existed": old is not None,
                        "before": digest(old) if old is not None else None, "after": digest(data)})
    manifest = backup / "manifest.json"
    atomic_write(manifest, json.dumps({"aircraft": str(root), "entries": entries}, indent=2).encode())
    committed = []
    try:
        for entry in entries:
            relative = Path(entry["path"])
            atomic_write(root / relative, changes[relative])
            committed.append(entry)
    except BaseException:
        for entry in reversed(committed):
            path = root / entry["path"]
            if entry["existed"]:
                atomic_write(path, (backup / "original" / entry["path"]).read_bytes())
            else:
                path.unlink(missing_ok=True)
        raise
    return manifest


def restore(manifest: Path):
    data = json.loads(manifest.read_text())
    root = Path(data["aircraft"]).resolve()
    for entry in data["entries"]:
        path = (root / entry["path"]).resolve()
        if not path.is_relative_to(root) or not path.is_file() or digest(path.read_bytes()) != entry["after"]:
            raise ValueError(f"Changed or missing patched file; refusing to overwrite: {path}")
        if entry["existed"]:
            old = manifest.parent / "original" / entry["path"]
            if digest(old.read_bytes()) != entry["before"]:
                raise ValueError(f"Backup hash mismatch: {old}")
    for entry in reversed(data["entries"]):
        path = root / entry["path"]
        if entry["existed"]:
            atomic_write(path, (manifest.parent / "original" / entry["path"]).read_bytes())
        else:
            path.unlink()
    print(f"Restored {len(data['entries'])} files")


def build_plan(root: Path, source_path: Path, asset_root: Path, max_size: int, light_ref: str | None):
    uv_report = audit_cockpit_uvs(source_path, root)
    if not uv_report["ok"]:
        failures = [r for r in uv_report["materials"] if not r["ok"]]
        raise ValueError("Source UV verification failed: " + json.dumps(failures))
    source = SourceTextures(source_path, asset_root, max_size)
    plan, report = {}, {"objects": [], "warnings": [], "runtime_verified": False,
                       "source_uv_audit": uv_report}
    acf_files = [p for p in root.glob("*.acf") if not p.name.startswith("._")]
    if len(acf_files) != 1:
        raise ValueError("Expected one aircraft ACF")
    acf = acf_files[0].read_text()
    for filename, material_name, kind in TARGETS:
        relative = OBJ_DIR / filename
        path = root / relative
        if not path.is_file():
            raise FileNotFoundError(f"Required mesh is missing; a texture patch cannot create it: {path}")
        if f"interior/cockpit-panels/{filename}" not in acf:
            raise ValueError(f"Object is not attached in the ACF: {filename}")
        text = path.read_text()
        info = inspect_obj(text)
        current = [{"directive": key, "path": value,
                    "exists": bool(value) and (path.parent / value).is_file()}
                   for key, value in texture_references(text)]
        images = make_materials(source, source.materials[material_name], kind)
        textures = {}
        for role, image in images.items():
            asset = ASSET_DIR / f"{material_name.lower()}_{role}.png"
            plan[asset] = png_bytes(image)
            directive = {"albedo": "TEXTURE", "lit": "TEXTURE_LIT"}.get(role, "TEXTURE_MAP " + role)
            textures[directive] = Path(os.path.relpath(root / asset, path.parent)).as_posix()
        patched = patch_obj(text, textures, kind, light_ref if kind == "labels" else None)
        plan[relative] = patched.encode()
        row = {"object": str(relative), "material": material_name, "kind": kind,
               "current_textures": current, "new_textures": textures,
               "uv_sha256_before_and_after": info["uv_sha256"], "uv_bounds": info["uv_bounds"],
               "geometry_animation_sha256_before_and_after": info["geometry_animation_sha256"],
               "vertices": info["vertices"], "triangles_drawn": info["triangles_drawn"]}
        if kind == "labels":
            row["uv_audit"] = label_uv_audit(info, images["albedo"])
        report["objects"].append(row)
        source.cache.clear()
        for livery in (root / "liveries").glob("*"):
            if (livery / relative).exists():
                report["warnings"].append(f"Livery overrides this object: {livery / relative}")
    if not (root / OBJ_DIR / "panel_a380_cockpit_mip03_4k.obj").exists():
        report["warnings"].append("MIP03 mesh is not exported. Missing geometry requires a source-mesh export; this patch only restores existing objects.")
    report["source_assets"] = source.provenance
    report["changed_files"] = [str(p) for p, data in plan.items() if not (root / p).is_file() or (root / p).read_bytes() != data]
    return plan, report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", type=Path, default=PROJECT / "output/A380X-XP12")
    parser.add_argument("--source-gltf", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--asset-root", type=Path, help="Default: source model's sibling texture directory")
    parser.add_argument("--max-size", type=int, default=4096, choices=(2048, 4096, 8192))
    parser.add_argument("--light-dataref", default="a380x/xp12/lighting/integral", help="Empty string uses X-Plane's default LIT lighting")
    parser.add_argument("--apply", action="store_true", help="Install after all source/OBJ/UV checks pass; default is dry-run")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--restore", type=Path, help="Restore an apply backup, refusing changed files")
    args = parser.parse_args()
    if args.restore:
        restore(args.restore.resolve())
        return
    if not args.aircraft or not args.source_gltf:
        parser.error("--aircraft and --source-gltf are required")
    root, source = args.aircraft.resolve(), args.source_gltf.resolve()
    assets = (args.asset_root or source.parent.parent / "texture").resolve()
    plan, report = build_plan(root, source, assets, args.max_size, args.light_dataref or None)
    report["mode"] = "apply" if args.apply else "dry-run"
    if args.apply:
        backup = install(root, plan)
        report["backup_manifest"] = str(backup) if backup else None
    if args.report:
        atomic_write(args.report.resolve(), (json.dumps(report, indent=2) + "\n").encode())
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, KeyError) as error:
        raise SystemExit(f"Cockpit patch aborted: {error}")
