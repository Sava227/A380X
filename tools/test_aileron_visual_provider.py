import json
from pathlib import Path
import tempfile
import unittest

from aileron_visual_provider import ALL_DATAREFS, BOOTSTRAP_FILES, DATAREF_CONTRACT, DATAREFS, FILES, HYDRAULIC_DATAREFS, MANIFEST, package_provider, validate_provider, verify_installed
from install_aileron_visuals import install
from validate_aircraft import validate


class AileronProviderTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.project, self.package, self.xplane = (self.base / name for name in ("project", "package", "sim"))
        for source, _, _ in FILES:
            path = self.project / source
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("-- fixture " + path.name)
        for source, _ in BOOTSTRAP_FILES:
            path = self.project / source
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"bootstrap binary fixture")
        self.plugin = self.xplane / "Resources/plugins/FlyWithLua"
        for directory in ("Scripts", "Modules"):
            (self.plugin / directory).mkdir(parents=True)

    def package_files(self):
        return package_provider(self.package, self.project)

    def mutate_manifest(self, transform):
        path = self.package / MANIFEST
        doc = json.loads(path.read_text())
        transform(doc)
        path.write_text(json.dumps(doc))

    def test_exact_contract_and_file_hashes(self):
        self.package_files()
        refs, errors = validate_provider(self.package)
        self.assertEqual(refs, set(ALL_DATAREFS))
        self.assertEqual(len(DATAREFS), 6)
        self.assertEqual(len(HYDRAULIC_DATAREFS), 16)
        self.assertEqual(len(refs), 22)
        self.assertEqual(len(FILES), 4)
        for row in DATAREF_CONTRACT:
            self.assertEqual((row["minimum"], row["maximum"]),
                             (0, 65) if row["name"].endswith("_deg") else (-1, 1))
        self.assertEqual(errors, [])

    def test_native_driver_contract(self):
        from aileron_visual_provider import NATIVE_DRIVER, NATIVE_DRIVER_FLAG
        document = self.package_files()
        self.assertEqual(document["native_driver"], NATIVE_DRIVER)
        self.assertEqual(NATIVE_DRIVER["flag_dataref"], NATIVE_DRIVER_FLAG)
        self.assertFalse(NATIVE_DRIVER["writes_simulator_datarefs"])
        self.assertEqual(NATIVE_DRIVER["ports"], [source for source, _, _ in FILES])
        self.assertIn("native_driver = 1", (self.package / "INSTALL_AILERON_VISUALS.txt").read_text())
        for change in (lambda d: d["native_driver"].update(flag_dataref="a380x/other"),
                       lambda d: d["native_driver"].update(writes_simulator_datarefs=True)):
            self.package_files()
            self.mutate_manifest(change)
            refs, errors = validate_provider(self.package)
            self.assertEqual(refs, set())
            self.assertIn("native driver", errors[0])
        # A manifest written before the native driver (no key) is still valid.
        self.package_files()
        self.mutate_manifest(lambda d: d.pop("native_driver"))
        self.assertEqual(validate_provider(self.package), (set(ALL_DATAREFS), []))

    def test_absent_manifest_does_not_allow_custom_refs(self):
        self.assertEqual(validate_provider(self.package), (set(), []))

    def test_wrong_names_types_and_duplicate_entries_fail_closed(self):
        changes = (
            lambda d: d["datarefs"][0].update(name="a380x/arbitrary"),
            lambda d: d["datarefs"][0].update(type="FloatArray"),
            lambda d: d["datarefs"][6].update(name="a380x/xp12/hydraulics/not_registered"),
            lambda d: d["datarefs"][6].update(minimum=0),
            lambda d: d["datarefs"][-1].update(minimum=-1),
            lambda d: d["datarefs"][-1].update(maximum=1),
            lambda d: d["datarefs"].pop(),
            lambda d: d["files"].pop(),
            lambda d: d["datarefs"].append(d["datarefs"][0]),
            lambda d: d.update(visual_only=False),
            lambda d: d["files"][0].update(path="../../outside.lua"),
            lambda d: d["files"][0].update(install_path="Scripts/other.lua"),
        )
        for change in changes:
            with self.subTest(change=change):
                self.package_files()
                self.mutate_manifest(change)
                refs, errors = validate_provider(self.package)
                self.assertFalse(refs)
                self.assertTrue(errors)

    def test_missing_or_changed_core_is_rejected(self):
        self.package_files()
        core = self.package / FILES[1][1]
        core.write_text("-- altered")
        self.assertTrue(validate_provider(self.package)[1])
        core.unlink()
        self.assertTrue(validate_provider(self.package)[1])

    def test_missing_or_changed_hydraulic_files_are_rejected(self):
        for _, packaged, _ in FILES[2:]:
            with self.subTest(packaged=packaged):
                self.package_files()
                path = self.package / packaged
                path.write_text("-- changed hydraulic file")
                self.assertTrue(validate_provider(self.package)[1])
                path.unlink()
                self.assertTrue(validate_provider(self.package)[1])

    def test_no_partial_package_when_source_missing(self):
        (self.project / FILES[1][0]).unlink()
        with self.assertRaises(ValueError):
            self.package_files()
        self.assertFalse((self.package / MANIFEST).exists())

    def test_bootstrap_is_required_and_hash_checked(self):
        self.package_files()
        self.mutate_manifest(lambda document: document.pop("bootstrap"))
        self.assertTrue(validate_provider(self.package)[1])
        self.package_files()
        (self.package / BOOTSTRAP_FILES[0][1]).write_bytes(b"changed binary")
        self.assertTrue(validate_provider(self.package)[1])
        (self.project / BOOTSTRAP_FILES[1][0]).unlink()
        with self.assertRaisesRegex(ValueError, "early-registration"):
            self.package_files()

    def test_installer_only_installs_four_visual_files_and_is_idempotent(self):
        self.package_files()
        existing = self.plugin / "Scripts/A380X_XP12_FBW.lua"
        existing.write_text("-- preserve disabled experimental law")
        report = install(self.package, self.xplane)
        self.assertTrue(report["files_match_package"])
        self.assertFalse(report["runtime_execution_verified"])
        self.assertEqual(install(self.package, self.xplane), report)
        self.assertEqual(existing.read_text(), "-- preserve disabled experimental law")
        self.assertEqual(len(report["files"]), 4)

    def test_installer_refuses_conflict_before_copying_anything(self):
        self.package_files()
        (self.plugin / FILES[1][2]).write_text("-- existing user module")
        with self.assertRaisesRegex(ValueError, "overwrite"):
            install(self.package, self.xplane)
        self.assertFalse((self.plugin / FILES[0][2]).exists())

    def test_installer_does_not_replace_symlink(self):
        self.package_files()
        (self.plugin / FILES[0][2]).symlink_to(self.package / FILES[0][1])
        with self.assertRaisesRegex(ValueError, "overwrite"):
            install(self.package, self.xplane)

    def test_installed_verification_rejects_stale_copy(self):
        self.package_files()
        install(self.package, self.xplane)
        (self.plugin / FILES[0][2]).write_text("-- changed")
        with self.assertRaisesRegex(ValueError, "stale"):
            verify_installed(self.package, self.plugin)

    def test_aircraft_validator_requires_exact_provider_even_without_native_catalog(self):
        (self.package / "objects").mkdir(parents=True)
        (self.package / "A380X-XP12.acf").write_text("P _obja/0/_v10_att_file_stl a.obj\n")
        obj = self.package / "objects/a.obj"
        def write_ref(ref):
            obj.write_text("I\n800\nOBJ\nPOINT_COUNTS 0 0 0 0\nANIM_begin\n"
                           f"ANIM_rotate 1 0 0 -25 25 -1 1 {ref}\nANIM_end\n")
        write_ref(DATAREFS[0])
        self.assertTrue(any("verified provider" in e for e in validate(self.package)["errors"]))
        self.package_files()
        self.assertFalse(validate(self.package)["errors"])
        catalog = self.base / "DataRefs.txt"
        catalog.write_text("sim/known float n\n")
        self.assertFalse(validate(self.package, catalog)["errors"])
        for ref in HYDRAULIC_DATAREFS:
            write_ref(ref)
            self.assertFalse(validate(self.package, catalog)["errors"])
        for ref in (DATAREFS[0] + "[0]", HYDRAULIC_DATAREFS[0] + "[0]",
                    "a380x/xp12/visual/not_registered", "a380x/xp12/hydraulics/not_registered", "sim/nonexistent"):
            write_ref(ref)
            self.assertTrue(validate(self.package, catalog)["errors"])


if __name__ == "__main__":
    unittest.main()
