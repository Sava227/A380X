#!/usr/bin/env python3
"""tools/png2dds/png2dds.c and tools/convert_textures_to_dds.py: DXT encoding of transparent textures."""
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import convert_textures_to_dds as conv  # noqa: E402

SIZE = 64


def write_png(path, rows):
    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    raw = b''.join(b'\0' + bytes(v for px in row for v in px) for row in rows)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', SIZE, SIZE, 8, 6, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def glyph_rows():
    """White 'L' strokes (alpha 255) and a soft edge (alpha 128) on transparent black."""
    rows = []
    for y in range(SIZE):
        row = []
        for x in range(SIZE):
            if (8 <= x < 14 and 8 <= y < 56) or (8 <= x < 40 and 50 <= y < 56):
                row.append((255, 255, 255, 255))
            elif x == 14 and 8 <= y < 50:
                row.append((255, 255, 255, 128))
            else:
                row.append((0, 0, 0, 0))
        rows.append(row)
    return rows


def dds_levels(path):
    data = path.read_bytes()
    height, width = struct.unpack_from('<II', data, 12)
    count = struct.unpack_from('<I', data, 28)[0]
    assert data[84:88] == b'DXT5'
    offset, levels = 128, []
    for _ in range(count):
        blocks_w, blocks_h = (width + 3) // 4, (height + 3) // 4
        pixels = [[None] * width for _ in range(height)]
        for by in range(blocks_h):
            for bx in range(blocks_w):
                block = data[offset:offset + 16]
                offset += 16
                a0, a1 = block[0], block[1]
                if a0 > a1:
                    apal = [a0, a1] + [((7 - k) * a0 + k * a1) // 7 for k in range(1, 7)]
                else:
                    apal = [a0, a1] + [((5 - k) * a0 + k * a1) // 5 for k in range(1, 5)] + [0, 255]
                abits = int.from_bytes(block[2:8], 'little')
                c0, c1, cbits = struct.unpack_from('<HHI', block, 8)
                def rgb(c):
                    return ((c >> 11) & 31) * 255 // 31, ((c >> 5) & 63) * 255 // 63, (c & 31) * 255 // 31
                p0, p1 = rgb(c0), rgb(c1)
                pal = [p0, p1, tuple((2 * a + b) // 3 for a, b in zip(p0, p1)), tuple((a + 2 * b) // 3 for a, b in zip(p0, p1))]
                for i in range(16):
                    x, y = bx * 4 + i % 4, by * 4 + i // 4
                    if x < width and y < height:
                        pixels[y][x] = (*pal[(cbits >> (2 * i)) & 3], apal[(abits >> (3 * i)) & 7])
        levels.append(pixels)
        width, height = max(1, width // 2), max(1, height // 2)
    return levels


def coverage(pixels, cutoff):
    flat = [p for row in pixels for p in row]
    return sum(p[3] > cutoff * 255 for p in flat) / len(flat)


class Png2Dds(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tool = conv.build_tool()
        cls.tmp = tempfile.TemporaryDirectory()
        cls.png = Path(cls.tmp.name) / 'glyph.png'
        write_png(cls.png, glyph_rows())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def convert(self, name, *flags):
        out = Path(self.tmp.name) / name
        run = subprocess.run([str(self.tool), *flags, str(self.png), str(out)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        return out

    def test_transparent_colour_filled_and_visible_texels_kept(self):
        top = dds_levels(self.convert('fill.dds'))[0]
        # transparent texels next to the stroke take the stroke colour, not black
        self.assertGreater(min(top[30][16][:3]), 200)
        self.assertGreater(min(top[40][60][:3]), 200)
        for x, y in ((10, 20), (20, 52)):
            self.assertGreater(min(top[y][x][:3]), 240)
            self.assertGreater(top[y][x][3], 240)

    def test_mip_colour_does_not_darken_at_the_edge(self):
        levels = dds_levels(self.convert('mips.dds'))
        for level in levels[1:4]:
            visible = [p for row in level for p in row if p[3] > 32]
            self.assertTrue(visible)
            self.assertGreater(min(min(p[:3]) for p in visible), 200)

    def test_alpha_test_keeps_coverage(self):
        plain = dds_levels(self.convert('plain.dds'))
        kept = dds_levels(self.convert('kept.dds', '--alpha-test', '0.5'))
        target = coverage(kept[0], 0.5)
        for level in (2, 3):
            self.assertLess(abs(coverage(kept[level], 0.5) - target), abs(coverage(plain[level], 0.5) - target) + 1e-9)
            self.assertLess(abs(coverage(kept[level], 0.5) - target), 0.06)

    def test_verify_weights_colour_by_alpha(self):
        out = self.convert('verify.dds')
        run = subprocess.run([str(self.tool), '--verify', str(self.png), str(out)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        psnr = float(run.stdout.split('psnr_rgb=')[1].split()[0])
        self.assertGreater(psnr, 40)

    def convert_rows(self, name, rows, *flags):
        png = Path(self.tmp.name) / (name + '.png')
        write_png(png, rows)
        out = Path(self.tmp.name) / (name + '.dds')
        run = subprocess.run([str(self.tool), *flags, str(png), str(out)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        return dds_levels(out)[0]

    def test_dark_rim_on_bright_lettering_repaired(self):
        rows = glyph_rows()
        for y in range(20, 28):                      # a few premultiplied rim texels (alpha 90), as in the
            rows[y][15] = (80, 80, 80, 90)           # FBW decal: most edge texels stay bright straight alpha
        top = self.convert_rows('rim', rows)
        self.assertGreater(min(top[24][15][:3]), 200)
        self.assertGreater(top[24][15][3], 80)

    def test_dark_edges_by_design_kept(self):
        rows = [[(0, 0, 0, 0)] * SIZE for _ in range(SIZE)]
        for y in range(8, 56):
            for x in range(8, 56):
                rows[y][x] = (40, 40, 40, 120)       # a dark translucent shadow, like the FCU labels
        top = self.convert_rows('shadow', rows)
        self.assertLess(max(top[30][30][:3]), 60)

    def test_bad_cutoff_rejected(self):
        run = subprocess.run([str(self.tool), '--alpha-test', '1.5', str(self.png), str(Path(self.tmp.name) / 'x.dds')],
                             capture_output=True, text=True)
        self.assertEqual(run.returncode, 2)


class Converter(unittest.TestCase):
    def test_alpha_test_cutoff_comes_from_attached_objs(self):
        with tempfile.TemporaryDirectory() as tmp:
            aircraft = Path(tmp) / 'Plane'
            (aircraft / 'objects').mkdir(parents=True)
            (aircraft / 'textures').mkdir()
            write_png(aircraft / 'textures' / 'decal.png', glyph_rows())
            (aircraft / 'Plane.acf').write_text('P _obja/0/_v10_att_file_stl a.obj\nP _obja/1/_v10_att_file_stl b.obj\n')
            for name, cutoff in (('a.obj', '0.5'), ('b.obj', '0.15')):
                (aircraft / 'objects' / name).write_text(
                    f'I\n800\nOBJ\nTEXTURE ../textures/decal.png\nPOINT_COUNTS 0 0 0 0\nATTR_no_blend {cutoff}\n')
            roles, cutoffs = conv.referenced_textures(aircraft)
            decal = aircraft / 'textures' / 'decal.png'
            self.assertEqual(roles[decal], {'albedo'})
            self.assertEqual(cutoffs[decal], 0.15)
            jobs, _ = conv.plan(aircraft, include_normals=False)
            self.assertEqual([(j[0].name, j[3]) for j in jobs], [('decal.png', 0.15)])


if __name__ == '__main__':
    unittest.main()
