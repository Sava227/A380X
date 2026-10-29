#!/usr/bin/env python3
from __future__ import annotations

import tempfile
from pathlib import Path
import unittest
import zlib

import build_skunkcrafts_depot as depot


class DepotTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.package = Path(self.temp.name) / "plane"
        self.package.mkdir()
        (self.package / "README.txt").write_text("test\n")
        (self.package / "changelog.txt").write_text("test.1\n- test\n")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def build(self, check: bool = False):
        return depot.build(self.package, "test.1", "https://example.invalid/depot/", check)

    def test_sorted_crc_size_once_and_exclusions(self) -> None:
        (self.package / "objects").mkdir()
        data = b"OBJ bytes\n"
        (self.package / "objects/z.obj").write_bytes(data)
        (self.package / "X-Camera_A380X-XP12.csv").write_text("camera\n")
        (self.package / "A380X-XP12_prefs.txt").write_text("private\n")
        rows = self.build()
        self.assertEqual([row.path for row in rows], sorted(row.path for row in rows))
        by_path = {row.path: row for row in rows}
        self.assertEqual(by_path["objects/z.obj"].crc, zlib.crc32(data) & 0xFFFFFFFF)
        self.assertEqual(by_path["objects/z.obj"].size, len(data))
        self.assertEqual(by_path["X-Camera_A380X-XP12.csv"].crc, -1)
        self.assertNotIn("A380X-XP12_prefs.txt", by_path)
        self.assertNotIn("skunkcrafts_updater.cfg", by_path)
        self.build(check=True)

    def test_check_is_read_only_and_detects_tamper(self) -> None:
        self.build()
        before = {p.name: p.read_bytes() for p in self.package.glob("skunkcrafts_*")}
        self.build(check=True)
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.package.glob("skunkcrafts_*")})
        (self.package / "skunkcrafts_updater_sizeslist.txt").write_text("bad\n")
        with self.assertRaisesRegex(depot.DepotError, "stale"):
            self.build(check=True)

    def test_lfs_pointer_and_symlink_are_rejected(self) -> None:
        (self.package / "asset.png").write_bytes(depot.LFS_HEADER + b"oid sha256:0\n")
        with self.assertRaisesRegex(depot.DepotError, "LFS pointer"):
            self.build()
        (self.package / "asset.png").unlink()
        link = self.package / "link"
        try:
            link.symlink_to(self.package / "README.txt")
        except OSError as error:
            self.skipTest(str(error))
        with self.assertRaisesRegex(depot.DepotError, "symlink"):
            self.build()

    def test_path_and_unicode_collisions(self) -> None:
        for value in ("../x", "/x", "a|b", "a\\b", " x"):
            with self.subTest(value=value), self.assertRaises(depot.DepotError):
                depot.safe_path(value)
        self.assertEqual(depot.collision_key("Cafe\N{COMBINING ACUTE ACCENT}.txt"),
                         depot.collision_key("Caf\N{LATIN SMALL LETTER E WITH ACUTE}.txt"))


if __name__ == "__main__":
    unittest.main()
