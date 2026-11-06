#!/usr/bin/env python3
"""Web layers of the reused native ND page (nd.html): TERRAIN on ND (docs/ND_TERRAIN.md) and the FMGC
pseudo waypoints (docs/AUTOPILOT_DEVELOPMENT.md).

The ND page is FlyByWire's ND/OANS bundle compiled through xp-kit plus the aircraft bridges, all inlined
in one nd.html (tools/package_xp_kit_cockpit.py package_nd). The JS build workspace no longer exists,
so - as tools/patch_xp_kit_sd.py does for the SD - this tool makes one anchored, verified insertion
into the verified page: the repository's layer scripts (LAYERS, in that order: native/avionics/xp-kit-nd/
terrain-layer.js, fmgc-pwp-layer.js), each as its own inline <script>, right after the native-bridge.js
block (they wrap that bridge's apply / onUpdate / needsSnapshot). Nothing of the FBW bundle or the
bridges is changed.

Every input must be a known original page, or a page this tool patched (its layer blocks removed, it
must hash to a known original). A patched page whose layers differ from the repository's (an older
layer, or a missing one) gets the current ones; re-running on an up-to-date page is a byte-identical
no-op. Unknown pages are rejected. A full ND rebuild (package_nd) inlines the same layers itself and is
not patched.
"""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYER_DIR = ROOT / 'native/avionics/xp-kit-nd'
# (file, version marker on its second line), in page order
LAYERS = (
    ('terrain-layer.js', '// A380X-ND-TERRAIN v1'),
    ('fmgc-pwp-layer.js', '// A380X-ND-PWP v1'),
)
VERSION = 2
LAYER = LAYER_DIR / LAYERS[0][0]   # the TERRAIN layer (first)
MARKER = LAYERS[0][1]
# nd.html of the verified native ND build installed in the aircraft (nd-source-manifest bundle_sha256 e171d9a6...).
KNOWN_INPUT_SHA256 = {
    '411d47405d46d79646ea300a691ac72568ed5939323574fe48cd99053755da62',
}
SIGNATURE = 'window.__a380NdSource='
BRIDGE_HEAD = '<script>// SPDX-License-Identifier: GPL-3.0-or-later\n// Live aircraft inputs only. Loaded after the common, validated PFD transport.'


def layer_head(marker):
    return '<script>// SPDX-License-Identifier: GPL-3.0-or-later\n' + marker


LAYER_HEAD = layer_head(MARKER)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def layer_block(source=None, marker=MARKER):
    name = dict((m, f) for f, m in LAYERS).get(marker, 'layer')
    script = source if source is not None else (LAYER_DIR / name).read_text()
    lines = script.split('\n', 2)
    if len(lines) < 2 or marker not in lines[1]:
        raise ValueError(name + ' lost its version marker line')
    if '</script' in script:
        raise ValueError(name + ' must not terminate its inline element')
    return '<script>' + script + '</script>'


def is_native_nd_page(data):
    """Only the aircraft-local native ND page (bundle + native bridge inlined) is ever patched."""
    return SIGNATURE.encode() in data and BRIDGE_HEAD.encode() in data


def split(text):
    """(head through the native-bridge block, [existing layer blocks in page order], rest)."""
    if text.count(BRIDGE_HEAD) != 1:
        raise ValueError('ND page must contain exactly one native-bridge block')
    start = text.index(BRIDGE_HEAD)
    end = text.index('</script>', start) + len('</script>')
    heads = [layer_head(marker) for _name, marker in LAYERS]
    for head in heads:
        if text.count(head) > 1:
            raise ValueError('ND page contains more than one ' + head.split('\n')[1] + ' layer')
    blocks, position = [], end
    while any(text.startswith(head, position) for head in heads):
        block_end = text.index('</script>', position) + len('</script>')
        blocks.append(text[position:block_end])
        position = block_end
    if any(head in text[:end] or head in text[position:] for head in heads):
        raise ValueError('ND layers are not right after the native bridge')
    return text[:end], blocks, text[position:]


def original_of(data):
    """The page without its layer blocks (bytes)."""
    head, _layers, rest = split(data.decode('utf-8'))
    return (head + rest).encode('utf-8')


def patch_page(data, layer_source=None, layer_sources=None):
    """Return the patched bytes of a known nd.html (unchanged if already up to date).
    layer_source replaces the TERRAIN layer, layer_sources {marker: source} any layer (tests)."""
    if not is_native_nd_page(data):
        raise ValueError('Not the native ND page')
    original = original_of(data)
    if sha256(original) not in KNOWN_INPUT_SHA256:
        raise ValueError('Unknown ND page %s; refusing to patch' % sha256(original))
    sources = dict(layer_sources or {})
    if layer_source is not None:
        sources[MARKER] = layer_source
    head, _layers, rest = split(original.decode('utf-8'))
    blocks = ''.join(layer_block(sources.get(marker), marker) for _name, marker in LAYERS)
    patched = (head + blocks + rest).encode('utf-8')
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
            'layers': [{'file': 'native/avionics/xp-kit-nd/' + name, 'marker': marker,
                        'sha256': sha256((LAYER_DIR / name).read_bytes())} for name, marker in LAYERS],
            'doc': ['docs/ND_TERRAIN.md', 'docs/AUTOPILOT_DEVELOPMENT.md']}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('nd_html', type=Path)
    parser.add_argument('--check', action='store_true', help='Verify that a page carries the current layers')
    args = parser.parse_args()
    data = args.nd_html.read_bytes()
    if args.check:
        if patch_page(data) != data:
            parser.error('page does not carry the current ND layers')
        print('ND layers current', sha256(data))
        return
    print(patch_file(args.nd_html))


if __name__ == '__main__':
    main()
