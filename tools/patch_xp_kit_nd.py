#!/usr/bin/env python3
"""TERRAIN on ND layer for the reused native ND page (nd.html), docs/ND_TERRAIN.md.

The ND page is FlyByWire's ND/OANS bundle compiled through xp-kit plus the aircraft bridges, all inlined
in one nd.html (tools/package_xp_kit_cockpit.py package_nd). The JS build workspace no longer exists,
so - as tools/patch_xp_kit_sd.py does for the SD - this tool makes one anchored, verified insertion
into the verified page: the repository's native/avionics/xp-kit-nd/terrain-layer.js as its own inline
<script> right after the native-bridge.js block (it wraps that bridge's apply/needsSnapshot). Nothing
of the FBW bundle or the bridges is changed.

Every input must be a known original page, or a page this tool patched (its terrain block removed, it
must hash to a known original). A patched page whose layer differs from the repository's gets the
current layer; re-running on an up-to-date page is a byte-identical no-op. Unknown pages are rejected.
A full ND rebuild (package_nd) inlines the same layer itself and is not patched.
"""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYER = ROOT / 'native/avionics/xp-kit-nd/terrain-layer.js'
VERSION = 1
MARKER = '// A380X-ND-TERRAIN v1'
# nd.html of the verified native ND build installed in the aircraft (nd-source-manifest bundle_sha256 e171d9a6...).
KNOWN_INPUT_SHA256 = {
    '411d47405d46d79646ea300a691ac72568ed5939323574fe48cd99053755da62',
}
SIGNATURE = 'window.__a380NdSource='
BRIDGE_HEAD = '<script>// SPDX-License-Identifier: GPL-3.0-or-later\n// Live aircraft inputs only. Loaded after the common, validated PFD transport.'
LAYER_HEAD = '<script>// SPDX-License-Identifier: GPL-3.0-or-later\n' + MARKER


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def layer_block(source=None):
    script = (source if source is not None else LAYER.read_text())
    if MARKER not in script.split('\n', 2)[1]:
        raise ValueError('terrain-layer.js lost its version marker line')
    if '</script' in script:
        raise ValueError('terrain-layer.js must not terminate its inline element')
    return '<script>' + script + '</script>'


def is_native_nd_page(data):
    """Only the aircraft-local native ND page (bundle + native bridge inlined) is ever patched."""
    return SIGNATURE.encode() in data and BRIDGE_HEAD.encode() in data


def split(text):
    """(head through the native-bridge block, existing terrain block or '', rest)."""
    if text.count(BRIDGE_HEAD) != 1:
        raise ValueError('ND page must contain exactly one native-bridge block')
    start = text.index(BRIDGE_HEAD)
    end = text.index('</script>', start) + len('</script>')
    if text.count(LAYER_HEAD) > 1:
        raise ValueError('ND page contains more than one terrain layer')
    if LAYER_HEAD in text:
        if not text.startswith(LAYER_HEAD, end):
            raise ValueError('Terrain layer is not right after the native bridge')
        layer_end = text.index('</script>', end) + len('</script>')
        return text[:end], text[end:layer_end], text[layer_end:]
    return text[:end], '', text[end:]


def original_of(data):
    """The page without its terrain block (bytes)."""
    head, _layer, rest = split(data.decode('utf-8'))
    return (head + rest).encode('utf-8')


def patch_page(data, layer_source=None):
    """Return the patched bytes of a known nd.html (unchanged if already up to date)."""
    if not is_native_nd_page(data):
        raise ValueError('Not the native ND page')
    original = original_of(data)
    if sha256(original) not in KNOWN_INPUT_SHA256:
        raise ValueError('Unknown ND page %s; refusing to patch' % sha256(original))
    head, _layer, rest = split(original.decode('utf-8'))
    patched = (head + layer_block(layer_source) + rest).encode('utf-8')
    return data if patched == data else patched


def patch_file(path):
    """Patch nd.html in place. Returns a manifest record, or None for a non-ND fixture."""
    path = Path(path)
    data = path.read_bytes()
    if not is_native_nd_page(data):
        return None
    patched = patch_page(data)
    if patched != data:
        path.write_bytes(patched)
    return {'tool': 'tools/patch_xp_kit_nd.py', 'version': VERSION, 'marker': MARKER,
            'input_sha256': sha256(data), 'original_sha256': sha256(original_of(patched)),
            'layer_sha256': sha256(LAYER.read_bytes()), 'output_sha256': sha256(patched),
            'doc': 'docs/ND_TERRAIN.md'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('nd_html', type=Path)
    parser.add_argument('--check', action='store_true', help='Verify that a page carries the current layer')
    args = parser.parse_args()
    data = args.nd_html.read_bytes()
    if args.check:
        if patch_page(data) != data:
            parser.error('page does not carry the current terrain layer')
        print('ND terrain layer current', sha256(data))
        return
    print(patch_file(args.nd_html))


if __name__ == '__main__':
    main()
