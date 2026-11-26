import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from halve_cabin_textures import cabin_only, is_cabin_object

TOOL = Path(__file__).with_name("halve_cabin_textures.py")


class HalveCabin(unittest.TestCase):
    def test_selection(self):
        self.assertTrue(is_cabin_object("objects/interior/cabin-seats/a.obj"))
        self.assertTrue(is_cabin_object("objects/interior/cargo/b.obj"))
        self.assertFalse(is_cabin_object("objects/interior/cabin/cabin_a380_cockpit_floor.obj"))
        self.assertFalse(is_cabin_object("objects/interior/cockpit-panels/c.obj"))
        users = {"textures/interior/cabin/x.png": {"objects/interior/cabin/a.obj"},
                 "textures/interior/cabin/shared.png": {"objects/interior/cabin/a.obj", "objects/interior/cockpit/b.obj"}}
        self.assertEqual(cabin_only(users), ["textures/interior/cabin/x.png"])

    def test_halves_once_above_floor(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "objects/interior/cabin").mkdir(parents=True)
            (root / "textures/interior/cabin").mkdir(parents=True)
            (root / "A380X-XP12.acf").write_text("P _obja/0/_v10_att_file_stl interior/cabin/a.obj\n")
            (root / "objects/interior/cabin/a.obj").write_text(
                "I\n800\nOBJ\nTEXTURE ../../../textures/interior/cabin/big.png\n"
                "TEXTURE_MAP normal ../../../textures/interior/cabin/small.png\nPOINT_COUNTS 0 0 0 0\n")
            Image.new("RGBA", (2048, 2048), (10, 20, 30, 255)).save(root / "textures/interior/cabin/big.png")
            Image.new("RGB", (512, 512), (128, 128, 128)).save(root / "textures/interior/cabin/small.png")
            for _ in range(2):
                subprocess.run([sys.executable, str(TOOL), "--aircraft", str(root)], check=True, capture_output=True)
            self.assertEqual(Image.open(root / "textures/interior/cabin/big.png").size, (1024, 1024))   # once only
            self.assertEqual(Image.open(root / "textures/interior/cabin/small.png").size, (512, 512))   # floor


if __name__ == "__main__":
    unittest.main()
