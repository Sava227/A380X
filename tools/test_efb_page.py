"""Web EFB page: JS logic on real native frames, page -> native verb contract, offline packaging.

The JS checks use macOS JavaScriptCore (`jsc`, as the SD bundle tests do) and frames produced by the
production adapter (native/avionics/test_efb_adapter.c --frame); both steps are skipped, not faked,
when clang, the X-Plane SDK headers or jsc are missing.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from package_xp_kit_cockpit import EFB_FONTS, NINE_VIEWS, SEVEN_VIEWS, package_efb

ROOT = Path(__file__).resolve().parents[1]
NATIVE = ROOT / 'native/avionics'
PAGE = NATIVE / 'xp-kit-efb'
SDK = ROOT / 'build/sdk/SDK/CHeaders/XPLM'
JSC = Path('/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc')
VIEWS = ('home', 'ofp', 'ground/doors', 'ground/services', 'ground/push', 'loading/payload', 'loading/fuel',
         'loading/wb', 'cabin', 'flight/perf', 'flight/nav', 'flight/comms', 'ops/checklists', 'ops/presets',
         'failures', 'aircraft', 'settings/general', 'settings/sounds')
LOCAL_VERBS = {'~clock', '~doormode', '~filter', '~kp', 'openchecklist'}


def native_verbs():
    source = (NATIVE / 'a380x_efb_web.inc').read_text()
    body = source[source.index('static int efb_execute('):source.index('void a380x_efb_web_action(')]
    return set(re.findall(r'!strcmp\(v, "([a-z_]+)"\)', body))


def build_frames(directory):
    clang = shutil.which('clang')
    if not clang or not (SDK / 'XPLMDataAccess.h').is_file():
        raise unittest.SkipTest('clang or X-Plane SDK headers unavailable')
    binary = Path(directory) / 'test_efb_adapter'
    subprocess.run([clang, '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-deprecated-declarations', '-DAPL=1',
                    '-I', str(SDK), str(NATIVE / 'test_efb_adapter.c'), str(NATIVE / 'a380x_avionics_core.c'),
                    str(NATIVE / 'a380x_efb_core.c'), '-lm', '-o', str(binary)], check=True, capture_output=True)
    frames = {view: json.loads(subprocess.check_output([str(binary), '--frame', *view.split('/')], text=True))
              for view in VIEWS}
    path = Path(directory) / 'frames.json'
    path.write_text(json.dumps(frames))
    return path, frames


class PageOnNativeFrames(unittest.TestCase):
    def setUp(self):
        if not JSC.exists():
            self.skipTest('JavaScriptCore shell not available')
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.frames_path, self.frames = build_frames(self.temporary.name)

    def run_check(self):
        result = subprocess.run([str(JSC), str(ROOT / 'tools/efb_page_check.js'), '--', str(PAGE / 'efb.js'),
                                 str(self.frames_path)], capture_output=True, text=True, timeout=120)
        return result.stdout + result.stderr

    def test_every_section_renders_and_behaves(self):
        output = self.run_check()
        self.assertIn('EFB PAGE OK %d views' % len(VIEWS), output, output)

    def test_page_verbs_are_native_verbs(self):
        output = self.run_check()
        rendered = set(next(line for line in output.splitlines() if line.startswith('VERBS ')).split()[1:])
        script = (PAGE / 'efb.js').read_text()
        keypad = set(re.findall(r"action: '([a-z_]+)", script))
        posted = (rendered - LOCAL_VERBS) | keypad | {'slide', 'cl_group'}
        self.assertTrue(posted <= native_verbs(), sorted(posted - native_verbs()))
        self.assertTrue({'pax', 'cargo', 'radio', 'fuel', 'tod', 'cabin'} <= keypad)

    def test_frames_are_small_and_never_carry_nan(self):
        for view, frame in self.frames.items():
            text = json.dumps(frame)
            self.assertLess(len(text), 16384, view)
            self.assertNotIn('NaN', text)
            self.assertEqual(frame['ui']['section'], view.split('/')[0])


class OfflinePackage(unittest.TestCase):
    def test_package_is_self_contained_and_offline(self):
        with tempfile.TemporaryDirectory() as directory:
            digest = package_efb(directory)
            html = (Path(directory) / 'efb.html').read_text()
            self.assertEqual(hashlib.sha256(html.encode()).hexdigest(), digest)
            self.assertNotIn('/*EFB_CSS*/', html)
            self.assertNotIn('/*EFB_JS*/', html)
            self.assertIn("connect-src 'none'", html)
            self.assertEqual(html.count('<script>'), 1)
            self.assertIn('window.__a380Efb', html.replace('global.__a380Efb', 'window.__a380Efb'))
            for name in EFB_FONTS:
                self.assertEqual((Path(directory) / 'Fonts/efb' / name).read_bytes(),
                                 (PAGE / 'Fonts/efb' / name).read_bytes())
            for url in re.findall(r'url\(([^)]*)\)', html):
                self.assertTrue(url.startswith('Fonts/efb/'), url)
                self.assertTrue((Path(directory) / url).is_file(), url)
            self.assertNotRegex(html, r'https?://(?!scripts\.sil\.org)')
            license = (Path(directory) / 'Fonts/efb/OFL.txt').read_text()
            self.assertEqual(license.count('SIL OPEN FONT LICENSE Version 1.1'), 2)
            self.assertIn('The Manrope Project Authors', license)
            self.assertIn('The JetBrains Mono Project Authors', license)

    def test_view_contract(self):
        self.assertEqual(NINE_VIEWS, [*SEVEN_VIEWS, 'captain_efb', 'first_officer_efb'])
        source = (NATIVE / 'a380x_xpkit_shared.h').read_text()
        self.assertIn('#define XPKIT_VIEWS 9', source)
        self.assertIn('#define XPKIT_EFB_W 1440', source)
        self.assertIn('#define XPKIT_EFB_H 1000', source)


if __name__ == '__main__':
    unittest.main()
