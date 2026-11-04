#!/usr/bin/env python3
"""Pre-compress the aircraft's PNG textures to DDS next to them (X-Plane prefers NAME.dds over a
referenced NAME.png), cutting texture memory from ~7 GB to under ~2.7 GB and removing X-Plane's
load-time PNG decoding and mipmap generation, the cause of the memory pressure, swapping and
input lag seen on 16 GB Macs.

Only textures the attached OBJs reference are converted (plus the same files in liveries/*/, so
liveries keep overriding them). Normal maps (TEXTURE_MAP normal / TEXTURE_NORMAL) stay PNG unless
--include-normals: block compression damages their fine detail (25 dB vs 38-43 dB for colour and
gloss maps), and Laminar ships its own normal maps as PNG too (118 PNG vs 1 DDS). Non-power-of-two
and tiny (< 64 px) images are skipped.

Every DDS is written by tools/png2dds/png2dds.c (built on demand), verified against its PNG and
recorded in <aircraft>/dds-manifest.json; a re-run converts only changed sources (or files from an
older encoder), --remove deletes exactly the files this tool wrote, and a pre-existing DDS it did
not write is never touched.

Transparent textures: png2dds fills the colour under transparent texels from their visible
neighbours (the FBW decals store black there, which drew dark smeared outlines around the panel
lettering) and averages mip colour by alpha. Textures drawn alpha-tested (ATTR_no_blend <cutoff> in
an attached OBJ) also keep their level-0 coverage at that cutoff in every mip level.

    python3 tools/convert_textures_to_dds.py "<X-Plane>/Aircraft/A380X-XP12"            # convert
    python3 tools/convert_textures_to_dds.py "<aircraft>" --dry-run                      # list only
    python3 tools/convert_textures_to_dds.py "<aircraft>" --remove                       # undo
"""
from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tools/png2dds/png2dds.c'
TOOL = ROOT / 'build/png2dds/png2dds'
MANIFEST = 'dds-manifest.json'
MIN_SIZE = 64
ENCODER = 3          # 1: plain box mips; 2: transparent-colour fill, alpha-weighted mips, alpha-test coverage; 3: + lettering edge repair
WARN_PSNR = 30.0
TEXTURE_LINE = re.compile(r'\s*(TEXTURE_LIT|TEXTURE_NORMAL|TEXTURE_MAP\s+(\S+)|TEXTURE)\s+(\S+)')


def build_tool() -> Path:
    if TOOL.exists() and TOOL.stat().st_mtime >= SOURCE.stat().st_mtime:
        return TOOL
    TOOL.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['clang', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', str(SOURCE), '-lz', '-lm',
                    '-o', str(TOOL)], check=True)
    return TOOL


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def png_size(path: Path):
    with path.open('rb') as f:
        head = f.read(24)
    if head[:8] != b'\x89PNG\r\n\x1a\n':
        return None
    return struct.unpack('>II', head[16:24])


def referenced_textures(aircraft: Path) -> tuple[dict[Path, set[str]], dict[Path, float]]:
    """Texture path -> roles ('albedo', 'lit', 'normal', 'material_gloss', ...) over all attached OBJs,
    and albedo texture -> the lowest ATTR_no_blend cutoff it is drawn with (alpha-tested textures)."""
    candidates = sorted(p for p in aircraft.glob('*.acf') if not p.name.startswith('._'))
    acf = next((p for p in candidates if p.stem == aircraft.name), candidates[0] if candidates else None)
    if acf is None:
        raise SystemExit(f'no .acf in {aircraft}')
    objs = sorted(set(re.findall(r'_v10_att_file_stl (\S+\.obj)', acf.read_text(errors='ignore'))))
    roles: dict[Path, set[str]] = {}
    cutoffs: dict[Path, float] = {}
    for rel in objs:
        obj = aircraft / 'objects' / rel
        if not obj.exists():
            continue
        albedo, tests = None, []
        with obj.open(errors='ignore') as f:
            for line in f:
                if line.startswith(('VT', 'IDX', 'TRIS', 'ANIM')):
                    continue
                if line.lstrip().startswith('ATTR_no_blend'):
                    words = line.split()
                    tests.append(float(words[1]) if len(words) > 1 else 0.5)
                    continue
                if line.startswith('ATTR'):
                    continue
                m = TEXTURE_LINE.match(line)
                if not m:
                    continue
                if m.group(1) == 'TEXTURE_NORMAL' or m.group(2) == 'normal':
                    role = 'normal'
                elif m.group(1) == 'TEXTURE_LIT':
                    role = 'lit'
                else:
                    role = m.group(2) or 'albedo'
                path = Path(os.path.normpath(obj.parent / m.group(3)))
                if path.suffix.lower() == '.png':
                    roles.setdefault(path, set()).add(role)
                    if role == 'albedo' and albedo is None:
                        albedo = path
        tests = [c for c in tests if c > 0.01]
        if albedo is not None and tests:
            cutoffs[albedo] = min(cutoffs.get(albedo, 1.0), min(tests))
    return roles, cutoffs


def plan(aircraft: Path, include_normals: bool):
    jobs, skipped = [], {}
    roles, cutoffs = referenced_textures(aircraft)
    liveries = [p for p in (aircraft / 'liveries').glob('*') if p.is_dir()] if (aircraft / 'liveries').exists() else []
    for png, kinds in sorted(roles.items()):
        try:
            rel = png.relative_to(aircraft)
        except ValueError:
            skipped[str(png)] = 'outside the aircraft folder'
            continue
        candidates = [png] + [liv / rel for liv in liveries if (liv / rel).exists()]
        for source in candidates:
            key = str(source.relative_to(aircraft))
            if not source.exists():
                skipped[key] = 'missing'
                continue
            if 'normal' in kinds and not include_normals:
                skipped[key] = 'normal map (kept PNG)'
                continue
            size = png_size(source)
            if not size:
                skipped[key] = 'not a PNG'
                continue
            w, h = size
            if w & (w - 1) or h & (h - 1):
                skipped[key] = f'{w}x{h} not a power of two'
                continue
            if max(w, h) < MIN_SIZE:
                skipped[key] = 'tiny'
                continue
            jobs.append((source, source.with_suffix('.dds'), '+'.join(sorted(kinds)), cutoffs.get(png, 0.0)))
    return jobs, skipped


def convert_one(tool: Path, source: Path, target: Path, alpha_test: float = 0.0):
    command = [str(tool)] + (['--alpha-test', f'{alpha_test:g}'] if alpha_test > 0 else []) + [str(source), str(target)]
    r = subprocess.run(command, capture_output=True, text=True)
    if r.returncode:
        return None, r.stderr.strip()
    v = subprocess.run([str(tool), '--verify', str(source), str(target)], capture_output=True, text=True)
    if v.returncode:
        target.unlink(missing_ok=True)
        return None, v.stderr.strip()
    m = re.search(r'psnr_rgb=([\d.]+) psnr_alpha=([\d.]+)', v.stdout)
    fmt = r.stdout.split()[1] if len(r.stdout.split()) > 1 else '?'
    return {'format': fmt, 'psnr_rgb': float(m.group(1)), 'psnr_alpha': float(m.group(2))}, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('aircraft', type=Path)
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--include-normals', action='store_true')
    ap.add_argument('--remove', action='store_true', help='delete every DDS this tool wrote')
    args = ap.parse_args()
    aircraft = args.aircraft.resolve()
    manifest_path = aircraft / MANIFEST
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {'files': {}}

    if args.remove:
        removed = 0
        for rel in list(manifest['files']):
            p = aircraft / rel
            if p.exists():
                p.unlink(); removed += 1
        manifest_path.unlink(missing_ok=True)
        print(f'removed {removed} DDS files written by this tool')
        return 0

    jobs, skipped = plan(aircraft, args.include_normals)
    todo, done_already, foreign = [], 0, []
    for source, target, kinds, alpha_test in jobs:
        rel = str(target.relative_to(aircraft))
        entry = manifest['files'].get(rel)
        if target.exists() and not entry:
            foreign.append(rel)
            continue
        # Opaque DXT1 output is identical across encoders 1 and 2 (alpha-weighted mips of alpha 255).
        current = entry and (entry.get('encoder', 1) == ENCODER or entry.get('format') == 'DXT1')
        if (entry and target.exists() and entry.get('source_sha256') == sha256(source)
                and current and entry.get('alpha_test', 0.0) == alpha_test):
            done_already += 1
            continue
        todo.append((source, target, kinds, alpha_test))
    print(f'{len(jobs)} textures to DDS ({done_already} up to date, {len(todo)} to convert), '
          f'{len(skipped)} skipped, {len(foreign)} existing DDS not written by this tool (left alone)')
    reasons: dict[str, int] = {}
    for why in skipped.values():
        reasons[why if 'power of two' not in why else 'not a power of two'] = reasons.get(why, 0) + 1
    for why, n in sorted(reasons.items()):
        print(f'  skipped {n:4d}: {why}')
    if args.dry_run or not todo:
        return 0

    tool = build_tool()
    failures, low = [], []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(convert_one, tool, s, t, a): (s, t, k, a) for s, t, k, a in todo}
        for n, future in enumerate(concurrent.futures.as_completed(futures), 1):
            source, target, kinds, alpha_test = futures[future]
            info, error = future.result()
            rel = str(target.relative_to(aircraft))
            if error:
                failures.append((rel, error))
            else:
                info.update({'source': str(source.relative_to(aircraft)), 'source_sha256': sha256(source),
                             'roles': kinds, 'bytes': target.stat().st_size, 'encoder': ENCODER,
                             'alpha_test': alpha_test})
                manifest['files'][rel] = info
                if info['psnr_rgb'] < WARN_PSNR:
                    low.append((rel, info['psnr_rgb']))
            if n % 25 == 0 or n == len(todo):
                print(f'  {n}/{len(todo)}', flush=True)
                manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True) + '\n')
    manifest_path.write_text(json.dumps(manifest, indent=1, sort_keys=True) + '\n')
    total = sum(e['bytes'] for e in manifest['files'].values())
    print(f'{len(manifest["files"])} DDS files ({total / 2**30:.2f} GB on disk), {len(failures)} failed')
    for rel, error in failures:
        print('  FAILED', rel, error)
    for rel, value in sorted(low, key=lambda x: x[1])[:10]:
        print(f'  low quality {value:.1f} dB: {rel}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
