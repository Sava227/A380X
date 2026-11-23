#!/usr/bin/env python3
"""CSS animation clock for the reused native PFD page (pfd.html), docs/ND_TERRAIN.md "Terrain alerts".

The PFD page is FlyByWire's A380X PFD bundle compiled through xp-kit plus the aircraft bridge, all inlined
in one pfd.html (tools/package_xp_kit_cockpit.py package). In the offscreen display helper WebKit
suspends the document timeline, so the FBW warnings that blink with CSS animations (Blink9Seconds:
PULL UP, TERRAIN, SINK RATE, W/S AHEAD ...) stay on their first, transparent keyframe and are never
seen. As tools/patch_xp_kit_nd.py does for the ND, this tool makes one anchored, verified insertion into
the verified page: native/avionics/xp-kit/animation-clock.js as its own inline <script> right after the
bridge.js block (it wraps the bridge's native frame clock). Nothing of the FBW bundle or the bridge is
changed.

Every input must be a known original page, or a page this tool patched (its clock block removed, it must
hash to a known original). A patched page whose clock differs from the repository's gets the current one;
re-running on an up-to-date page is a byte-identical no-op. Unknown pages are rejected. A full PFD
package (package) inlines the same script itself and is not patched.
"""
import argparse
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CLOCK = ROOT / 'native/avionics/xp-kit/animation-clock.js'
VERSION = 1
MARKER = '// A380X-PFD-ANIMATION-CLOCK v1'
# pfd.html of the verified native PFD package installed in the aircraft.
KNOWN_INPUT_SHA256 = {
    '00271ace78fc341c448c5a9da01c1ebc100c9b15d42fd476274465922a1a7180',
}
SIGNATURE = 'window.__a380PfdSource='
BRIDGE_HEAD = ('<script>// SPDX-License-Identifier: GPL-3.0-or-later\n(() => {\n'
               '  // WebKit suspends requestAnimationFrame for an offscreen window.')
CLOCK_HEAD = '<script>// SPDX-License-Identifier: GPL-3.0-or-later\n' + MARKER


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def clock_block(source=None):
    script = source if source is not None else CLOCK.read_text()
    if MARKER not in script.split('\n', 2)[1]:
        raise ValueError('animation-clock.js lost its version marker line')
    if '</script' in script:
        raise ValueError('animation-clock.js must not terminate its inline element')
    return '<script>' + script + '</script>'


def is_native_pfd_page(data):
    """Only the aircraft-local native PFD page (bundle + bridge inlined) is ever patched."""
    return SIGNATURE.encode() in data and BRIDGE_HEAD.encode() in data


def split(text):
    """(head through the bridge block, existing clock block or '', rest)."""
    if text.count(BRIDGE_HEAD) != 1:
        raise ValueError('PFD page must contain exactly one bridge block')
    start = text.index(BRIDGE_HEAD)
    end = text.index('</script>', start) + len('</script>')
    if text.count(CLOCK_HEAD) > 1:
        raise ValueError('PFD page contains more than one animation clock')
    if CLOCK_HEAD in text:
        if not text.startswith(CLOCK_HEAD, end):
            raise ValueError('Animation clock is not right after the bridge')
        clock_end = text.index('</script>', end) + len('</script>')
        return text[:end], text[end:clock_end], text[clock_end:]
    return text[:end], '', text[end:]


def original_of(data):
    """The page without its clock block (bytes)."""
    head, _clock, rest = split(data.decode('utf-8'))
    return (head + rest).encode('utf-8')


def patch_page(data, clock_source=None):
    """Return the patched bytes of a known pfd.html (unchanged if already up to date)."""
    if not is_native_pfd_page(data):
        raise ValueError('Not the native PFD page')
    original = original_of(data)
    if sha256(original) not in KNOWN_INPUT_SHA256:
        raise ValueError('Unknown PFD page %s; refusing to patch' % sha256(original))
    head, _clock, rest = split(original.decode('utf-8'))
    patched = (head + clock_block(clock_source) + rest).encode('utf-8')
    return data if patched == data else patched


def patch_file(path):
    """Patch pfd.html in place. Returns a manifest record, or None for a non-PFD fixture."""
    path = Path(path)
    data = path.read_bytes()
    if not is_native_pfd_page(data):
        return None
    patched = patch_page(data)
    if patched != data:
        path.write_bytes(patched)
    return {'tool': 'tools/patch_xp_kit_pfd.py', 'version': VERSION, 'marker': MARKER,
            'input_sha256': sha256(data), 'original_sha256': sha256(original_of(patched)),
            'clock_sha256': sha256(CLOCK.read_bytes()), 'output_sha256': sha256(patched),
            'doc': 'docs/ND_TERRAIN.md'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('pfd_html', type=Path)
    parser.add_argument('--check', action='store_true', help='Verify that a page carries the current clock')
    args = parser.parse_args()
    data = args.pfd_html.read_bytes()
    if args.check:
        if patch_page(data) != data:
            parser.error('page does not carry the current animation clock')
        print('PFD animation clock current', sha256(data))
        return
    print(patch_file(args.pfd_html))


if __name__ == '__main__':
    main()
