"""Check the real C/Python shared-memory ABI and independent display buffers."""
import ctypes as C
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from xp_kit_transport import EFB_H, EFB_W, MFD_H, MFD_W, ND_H, ND_W, Shared, VERSION, H, W, view_buffer, view_dimensions

ROOT = Path(__file__).resolve().parents[1]


class TransportTests(unittest.TestCase):
    def test_all_view_buffers_have_independent_native_resolution(self):
        previous_end = 0
        for view in range(9):
            width, height = view_dimensions(view)
            self.assertEqual((width, height), (W, H) if view < 2 else (ND_W, ND_H) if view < 4 else
                             (MFD_W, MFD_H) if view < 7 else (EFB_W, EFB_H))
            start, length = view_buffer(view)
            self.assertEqual(length, width * height * 4)
            self.assertGreaterEqual(start, previous_end)
            previous_end = start + length
        self.assertLessEqual(previous_end, C.sizeof(Shared))

    def test_python_matches_compiled_c_abi(self):
        compiler = shutil.which('clang') or shutil.which('cc')
        if not compiler:
            self.skipTest('C compiler unavailable')
        fields = ['ready', 'frame_time', 'pixels', 'nd_pixels', 'mfd_pixels', 'mfd', 'sd', 'nd_profile', 'efb_pixels', 'efb']
        prints = ','.join('\\"%s\\":%%zu' % field for field in fields)
        offsets = ','.join('offsetof(A380XXpKitShared,%s)' % field for field in fields)
        dims = ','.join('xpkit_view_width(%d),xpkit_view_height(%d)' % (v, v) for v in range(9))
        source = ('#include <stdio.h>\n#include <stddef.h>\n#include "a380x_xpkit_shared.h"\n'
                  'int main(void){printf("{\\"version\\":%u,\\"size\\":%zu,' + prints + ',\\"dims\\":['
                  + ','.join(['%d,%d'] * 9) + ']}\\n",'
                  'XPKIT_VERSION,sizeof(A380XXpKitShared),' + offsets + ',' + dims + ');return 0;}\n')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'layout.c').write_text(source)
            subprocess.run([compiler, '-std=c11', '-I', str(ROOT / 'native/avionics'),
                            str(path / 'layout.c'), '-o', str(path / 'layout')], check=True, capture_output=True)
            actual = json.loads(subprocess.check_output([str(path / 'layout')]))
        self.assertEqual(actual.pop('version'), VERSION)
        self.assertEqual(actual.pop('size'), C.sizeof(Shared))
        dims = actual.pop('dims')
        self.assertEqual([tuple(dims[2 * v:2 * v + 2]) for v in range(9)], [view_dimensions(v) for v in range(9)])
        self.assertEqual(actual, {field: getattr(Shared, field).offset for field in fields})


if __name__ == '__main__':
    unittest.main()
