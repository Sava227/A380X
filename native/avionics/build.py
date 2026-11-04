#!/usr/bin/env python3
"""Build and test the aircraft-local FBW-derived avionics plugin.

The build uses the official X-Plane SDK and produces a universal macOS plugin
plus a Windows x86-64 plugin.  It never launches or modifies X-Plane.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys as _sys


HERE = Path(__file__).resolve().parent
PLUGIN = HERE / "a380x_avionics.c"
CORE = HERE / "a380x_avionics_core.c"
CORE_HEADER = HERE / "a380x_avionics_core.h"
ADIRS = HERE / "a380x_adirs.c"
ADIRS_TEST = HERE / "test_adirs.c"
ADIRS_ADAPTER_TEST = HERE / "test_adirs_adapter.c"
ADIRS_WINDOW_TEST = HERE / "test_adirs_window.c"
ELEC_TEST = HERE / "test_elec_panel.c"
ELEC_ADAPTER_TEST = HERE / "test_elec_panel_adapter.c"
# A380 electrical network (phase 2): XPLM-free core + flyPad preset tables, adapter a380x_elec_adapter.inc.
ELEC_NETWORK_SOURCES = [HERE / "a380x_elec.c", HERE / "a380x_elec_presets.c"]
# Native Flight Warning System (docs/FWS_ECL.md, docs/FWS_SIGNALS.md): XPLM-free C11 core; tables are
# generated from the pinned FBW EcamMessages by tools/build_fws_tables.py; adapter a380x_fws_adapter.inc.
FWS_SOURCES = [HERE / name for name in ("a380x_fws.c", "a380x_fws_signals.c", "a380x_fws_ecl.c", "a380x_fws_lines.c")]
FWS_TESTS = {"lines": (HERE / "test_fws_lines.c", ["a380x_fws_lines.c"]),
             "ecl": (HERE / "test_fws_ecl.c", ["a380x_fws_lines.c", "a380x_fws_ecl.c"]),
             "signals": (HERE / "test_fws_signals.c", ["a380x_fws_signals.c"]),
             "core": (HERE / "test_fws_core.c", ["a380x_fws.c", "a380x_fws_signals.c", "a380x_fws_ecl.c", "a380x_fws_lines.c"])}
ELEC_NETWORK_TEST = HERE / "test_elec.c"
ELEC_NETWORK_ADAPTER_TEST = HERE / "test_elec_adapter.c"
# Overhead annunciator lamps (docs/OVERHEAD_LAMPS.md): XPLM-free logic a380x_ovhd_lamps.h, generated table
# a380x_ovhd_lamps_table.h (tools/ovhd_lamps_contract.py), X-Plane adapter a380x_ovhd_lamps.inc.
OVHD_TEST = HERE / "test_ovhd_lamps.c"
OVHD_ADAPTER_TEST = HERE / "test_ovhd_lamps_adapter.c"
# L/G GRVTY panel (docs/LG_GRVTY.md): XPLM-free core a380x_lg_grvty.h, X-Plane adapter a380x_lg_grvty.inc
# (tested together with the production center-right adapter against a mock XPLM).
LG_GRVTY_TEST = HERE / "test_lg_grvty.c"
RMP_TEST = HERE / "test_rmp.c"
LG_GRVTY_ADAPTER_TEST = HERE / "test_lg_grvty_adapter.c"
# Thrust / reverse levers, FADEC rating + FLEX, instinctive disconnect (docs/THRUST_LEVERS.md): XPLM-free
# cores a380x_detent_lever.h + a380x_thrust.h, adapters a380x_cockpit_lever.inc + a380x_thrust.inc (tested
# against a mock XPLM linked with the vendored FBW FADEC tables); the detent click is synthesized by
# tools/build_detent_click.py.
THRUST_TEST = HERE / "test_thrust.c"
THRUST_ADAPTER_TEST = HERE / "test_thrust_adapter.c"
DETENT_CLICK = HERE / "sounds/thrust_detent_click.wav"
# FLAPS lever + FBW SFCC + FPPU flap/slat drives (docs/FLAPS_LEVER.md): XPLM-free core a380x_flaps.h, adapter
# a380x_flaps.inc (tested against a mock XPLM; the stock flap commands it intercepts are checked below).
FLAPS_TEST = HERE / "test_flaps.c"
FLAPS_ADAPTER_TEST = HERE / "test_flaps_adapter.c"
# SPEED BRAKE lever + FBW PRIM speed brake / ground spoilers (docs/SPEED_BRAKE.md): XPLM-free core
# a380x_speedbrake.h, adapter a380x_speedbrake.inc (tested against a mock XPLM; the stock speed brake commands it
# intercepts are checked below). It reuses the thrust detent click.
SPEEDBRAKE_TEST = HERE / "test_speedbrake.c"
SPEEDBRAKE_ADAPTER_TEST = HERE / "test_speedbrake_adapter.c"
SPEEDBRAKE_SOURCES = [HERE / "a380x_speedbrake.h", HERE / "a380x_speedbrake.inc"]
DETENT_CLICK_GENERATOR = HERE.parent.parent / "tools/build_detent_click.py"
EFB = HERE / "a380x_efb.c"
EFB_HEADER = HERE / "a380x_efb.h"
TEST = HERE / "test_avionics_core.c"
GUIDANCE = HERE / "a380x_guidance.c"
GUIDANCE_TEST = HERE / "test_guidance.c"
PFD_TEST = HERE / "test_pfd_render.c"
EFB_TEST = HERE / "test_efb_render.c"
ADAPTER_TEST = HERE / "test_fcu_adapter.c"
FCU_MOTION_TEST = HERE / "test_fcu_motion.c"
FCU_LIGHT_TEST = HERE / "test_fcu_lighting.c"
CONTROL_LIFECYCLE_TEST = HERE / "test_control_lifecycle.c"
CLOCK_RENDER_TEST = HERE / "test_clock_render.c"
EFIS_TEST = HERE / "test_efis.c"
EFIS_ND_TEST = HERE / "test_efis_nd.c"
EFIS_AIRPORT_TEST = HERE / "test_efis_airport.c"
EWD_DATA_TEST = HERE / "test_ewd_data.c"
EWD_RENDER_TEST = HERE / "test_ewd_render.c"
EWD_FONT = HERE / "fonts/FBW-Display-EIS-A380.ttf"
EWD_FONT_HEADER = HERE / "a380x_ewd_font.h"
EWD_FONT_GENERATOR = HERE.parent.parent / "tools/build_ewd_font.py"
EWD_FONT_SHA256 = "85a25cece2241ab8939b11869dce762c43f2b65fc4a47d8ec9038688fa2085c4"
FCU_FONTS = {HERE / "fonts/A380X_FCU.ttf": "fab287f184145206289a04085758e1a6f19c8330b89649ba7fc59f072c049490",
             HERE / "fonts/Poppins-SemiBold.ttf": "ae660e6c947afd38446cc19f2acfe2101fd6574fd6f1ae4640d249a3c87470fb"}
FCU_FONT_HEADER = HERE / "a380x_fcu_font.h"
FCU_FONT_GENERATOR = HERE.parent.parent / "tools/build_fcu_font.py"
FCU_RENDER_TEST = HERE / "test_fcu_render.c"
# ISIS 1 (standby flight display): XPLM-free core a380x_isis.inc + X-Plane adapter a380x_isis_adapter.inc.
ISIS_TEST = HERE / "test_isis.c"
CONTROL_MOTION_TEST = HERE / "test_control_motion.c"
ISIS_ADAPTER = HERE / "a380x_isis_adapter.inc"
ISIS_FONT = HERE / "fonts/ISISFontTemporary.ttf"
ISIS_FONT_SHA256 = "e936f34e42474d0d084adba49378f672a743e596a9c2e6694f12fce3014e86cc"
ISIS_FONT_HEADER = HERE / "a380x_isis_font.h"
ISIS_FONT_GENERATOR = HERE.parent.parent / "tools/build_isis_font.py"
EWD_FADEC_SOURCES = [HERE / "a380x_ewd_fadec.cpp", HERE / "vendor/ewd_fadec/Fadec.cpp"]
CXX_SOURCES = [*EWD_FADEC_SOURCES, HERE / "a380x_fbw_laws.cpp", *[
    HERE / "vendor/fbw" / name for name in
    ("A380FgOuterLoops.cpp", "rt_modd.cpp", "look1_binlxpw.cpp")]]
# Native MFD + FMS (docs/mfd-spec/09-port-integration.md section 1.1). XPLM-free C11 core; the
# X-Plane boundary is a380x_mfd_adapter.inc + a380x_fms_adapter.inc (+ a380x_mfd_systems_host.inc).
MFD_CORE_SOURCES = [HERE / name for name in (
    "a380x_mfd_draw.c", "a380x_mfd_ui.c", "a380x_mfd.c", "a380x_mfd_formats.c", "a380x_fms_messages.c",
    "a380x_fms_speeds.c", "a380x_fms.c", "a380x_navdata.c", "a380x_fms_fpln.c", "a380x_mfd_pages.c",
    "a380x_sd.c")]
MFD_PAGE_MODULES = ("data", "init", "fuel", "sec", "perf", "fpln", "fpln_rev", "position", "surv",
                    "atccom", "fcubkup")
MFD_ADAPTER_TEST = HERE / "test_mfd_adapter.c"
SD_ADAPTER_TEST = HERE / "test_sd_adapter.c"
# Live FBW SD page variables (docs/SD_PAGES.md): a380x_sd_data.inc, mocked-dataref test.
SD_DATA_TEST = HERE / "test_sd_data.c"
MFD_TEST_DATA = HERE / "test_data"
MFD_FONT_HEADER = HERE / "a380x_mfd_font.h"
MFD_FONT_GENERATOR = HERE.parent.parent / "tools/build_mfd_font.py"
# (test source, kind): "core" = pure C unit test, "navdata" = gets the fixture directory,
# "render" = writes PPM previews into its own directory. Optional page tests run when present.
MFD_TESTS = (("test_mfd_formats", "core"), ("test_fms_core", "core"), ("test_navdata", "navdata"),
             ("test_mfd_ui", "core"), ("test_mfd_render", "render"), ("test_sd", "render"))
MFD_OPTIONAL_TESTS = (("test_mfd_pages_a", "render"), ("test_mfd_page_perf", "render"),
                      ("test_mfd_page_fpln", "render"), ("test_mfd_pages_h", "render"))
EXTRA_INPUTS = [*FWS_SOURCES, *(test for test, _ in FWS_TESTS.values()), HERE / "test_xpkit_nd_input.c", HERE / "a380x_xpkit_host.m", HERE / "test_xpkit_host.c", ADIRS, ADIRS_TEST, ADIRS_ADAPTER_TEST, ADIRS_WINDOW_TEST, ELEC_TEST, ELEC_ADAPTER_TEST, *ELEC_NETWORK_SOURCES, ELEC_NETWORK_TEST, ELEC_NETWORK_ADAPTER_TEST, OVHD_TEST, OVHD_ADAPTER_TEST, LG_GRVTY_TEST, LG_GRVTY_ADAPTER_TEST, THRUST_TEST, THRUST_ADAPTER_TEST, DETENT_CLICK, FLAPS_TEST, FLAPS_ADAPTER_TEST, SPEEDBRAKE_TEST, SPEEDBRAKE_ADAPTER_TEST, *SPEEDBRAKE_SOURCES, GUIDANCE, GUIDANCE_TEST, PFD_TEST, EFB_TEST, ADAPTER_TEST, CONTROL_LIFECYCLE_TEST, CLOCK_RENDER_TEST, EFIS_TEST, EFIS_ND_TEST, EFIS_AIRPORT_TEST,
    EWD_DATA_TEST, EWD_RENDER_TEST, FCU_MOTION_TEST, FCU_LIGHT_TEST, ISIS_TEST, CONTROL_MOTION_TEST,
    *CXX_SOURCES, *[p for p in HERE.glob("*.h") if not p.name.startswith("._")],
    *[p for p in HERE.glob("*.inc") if not p.name.startswith("._")],
    *MFD_CORE_SOURCES, MFD_ADAPTER_TEST, SD_ADAPTER_TEST, SD_DATA_TEST,
    *[HERE / f"{name}.c" for name, _ in MFD_TESTS],
    *[p for p in MFD_TEST_DATA.rglob("*") if p.is_file() and not p.name.startswith("._")],
    *HERE.glob("sounds/*.pcm"),
    *[p for directory in ("vendor/fbw", "vendor/ewd_fadec", "fonts")
      for p in (HERE / directory).glob("*") if p.is_file() and not p.name.startswith("._")]]
EXPORTS = {"XPluginStart", "XPluginStop", "XPluginEnable", "XPluginDisable",
           "XPluginReceiveMessage"}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=HERE / "dist")
    parser.add_argument("--datarefs", type=Path, help="Optional installed simulator DataRefs.txt for raw-input contract verification")
    parser.add_argument("--clang", default="clang")
    parser.add_argument("--mingw", default="/opt/homebrew/bin/x86_64-w64-mingw32-gcc")
    parser.add_argument("--xpkit", action="store_true", help="Build aircraft-local macOS xp-kit PFD, ND/OANS and MFD renderers")
    parser.add_argument("--xpkit-windows", action="store_true", help="Also build the Windows WebView2 host and seven-view native display integration")
    parser.add_argument("--webview2-sdk", type=Path, default=HERE.parent.parent/'build/sdk/webview2/1.0.3537.50')
    parser.add_argument("--json-header", type=Path, default=HERE.parent.parent/'build/sdk/json/json.hpp')
    parser.add_argument("--xpkit-assets", type=Path, help="Reuse byte-verified installed native display assets for a native-code-only change")
    parser.add_argument("--ecam-source", type=Path, help="Authored cockpit glTF for all physical ECAM page/action/SD-brightness control meshes")
    parser.add_argument("--eng-button", action="store_true", help="Stage ENG-only physical button animation and green indicator for the current cockpit")
    parser.add_argument("--allow-missing-pages", action="store_true",
                        help="Incremental integration build: link only the MFD page modules that exist")
    parser.add_argument("--skip-tests", default="",
                        help="Comma-separated optional MFD page tests to leave out (incremental builds only)")
    parser.add_argument("--skip-pages", default="",
                        help="Comma-separated MFD page modules to leave out (incremental builds only)")
    args = parser.parse_args()
    if args.xpkit_assets and not args.xpkit:
        parser.error('--xpkit-assets requires --xpkit')
    if args.xpkit_windows and not args.xpkit:
        parser.error('--xpkit-windows requires --xpkit so the original display assets are packaged')
    if args.eng_button and not (args.xpkit and args.ecam_source):
        parser.error('--eng-button requires --xpkit and --ecam-source')
    if platform.system() != "Darwin":
        parser.error("This build orchestrator requires macOS for the universal Mac slice")
    sdk, output = args.sdk.resolve(), args.output_dir.absolute()
    if output.exists() or output.is_symlink():
        parser.error("Output directory already exists; select a fresh staging path")
    skipped_pages = {name for name in args.skip_pages.split(",") if name}
    if skipped_pages and not args.allow_missing_pages:
        parser.error("--skip-pages requires --allow-missing-pages")
    mfd_pages = [name for name in MFD_PAGE_MODULES
                 if (HERE / f"a380x_mfd_page_{name}.c").is_file() and name not in skipped_pages]
    missing_pages = [name for name in MFD_PAGE_MODULES if name not in mfd_pages]
    if missing_pages and not args.allow_missing_pages:
        parser.error(f"Missing MFD page modules: {missing_pages} (use --allow-missing-pages for an incremental build)")
    mfd_page_sources = [HERE / f"a380x_mfd_page_{name}.c" for name in mfd_pages]
    mfd_page_flags = [f"-DA380X_MFD_HAVE_PAGE_{name.upper()}=1" for name in mfd_pages if name != "data"]
    MFD_SOURCES = [*MFD_CORE_SOURCES, *mfd_page_sources]
    skipped_tests = {name for name in args.skip_tests.split(",") if name}
    if skipped_tests and not args.allow_missing_pages:
        parser.error("--skip-tests requires --allow-missing-pages")
    mfd_optional_tests = [(name, kind) for name, kind in MFD_OPTIONAL_TESTS
                          if (HERE / f"{name}.c").is_file() and name not in skipped_tests]
    headers = sdk / "CHeaders/XPLM"
    framework = sdk / "Libraries/Mac/XPLM.framework/XPLM"
    import_library = sdk / "Libraries/Win/XPLM_64.lib"
    header_names = ("XPLMDataAccess.h", "XPLMDefs.h", "XPLMDisplay.h",
                    "XPLMGraphics.h", "XPLMPlugin.h", "XPLMProcessing.h",
                    "XPLMUtilities.h", "XPLMMenus.h", "XPLMSound.h", "XPLMNavigation.h", "XPLMScenery.h", "XPLMPlanes.h")
    inputs = [headers / name for name in header_names]
    for path in [*inputs, framework, import_library, PLUGIN, CORE, CORE_HEADER, EFB, EFB_HEADER, TEST,
                 EWD_FONT, EWD_FONT_HEADER, EWD_FONT_GENERATOR, DETENT_CLICK_GENERATOR, ISIS_FONT, ISIS_FONT_HEADER, ISIS_FONT_GENERATOR, *FCU_FONTS, FCU_FONT_HEADER, FCU_FONT_GENERATOR, FCU_RENDER_TEST, MFD_FONT_HEADER, MFD_FONT_GENERATOR, *MFD_SOURCES, *EXTRA_INPUTS]:
        if not path.is_file():
            parser.error(f"Missing build input: {path}")
    clang = shutil.which(args.clang)
    mingw = shutil.which(args.mingw)
    objdump = shutil.which(args.mingw.removesuffix("gcc") + "objdump")
    if not all((clang, mingw, objdump, shutil.which("codesign"), shutil.which("lipo"))):
        parser.error("Missing clang, MinGW gcc/objdump, codesign, or lipo")
    vendor_manifest = json.loads((HERE / "vendor/fbw/manifest.json").read_text())
    for row in vendor_manifest["files"]:
        if digest(HERE / "vendor/fbw" / row["filename"]) != row["sha256"]:
            parser.error(f"Imported FBW source changed: {row['filename']}")
    ewd_fadec_manifest = json.loads((HERE / "vendor/ewd_fadec/manifest.json").read_text())
    if ewd_fadec_manifest["revision"] != vendor_manifest["revision"]:
        parser.error("EWD FADEC and FBW guidance source revisions disagree")
    for row in [*ewd_fadec_manifest["files"], *ewd_fadec_manifest.get("local_files", [])]:
        if "filename" in row and digest(HERE / "vendor/ewd_fadec" / row["filename"]) != row["sha256"]:
            parser.error(f"Imported EWD FADEC source changed: {row['filename']}")
    if digest(EWD_FONT) != EWD_FONT_SHA256:
        parser.error("Imported FBW EIS font differs from the pinned source")
    fcu_header_text = FCU_FONT_HEADER.read_text()
    for fcu_font, fcu_hash in FCU_FONTS.items():
        if digest(fcu_font) != fcu_hash or f"{fcu_font.name} SHA-256: {fcu_hash}" not in fcu_header_text:
            parser.error(f"FCU font or its embedded atlas differs from the pinned source: {fcu_font.name}")
    if (digest(ISIS_FONT) != ISIS_FONT_SHA256
            or f"ISISFontTemporary.ttf SHA-256: {ISIS_FONT_SHA256}" not in ISIS_FONT_HEADER.read_text()[:2048]):
        parser.error("ISIS font or its embedded atlas differs from the pinned FBW source")
    recorded_font_hash = re.search(r"Font SHA-256: ([0-9a-f]{64})", EWD_FONT_HEADER.read_text())
    if not recorded_font_hash or recorded_font_hash.group(1) != EWD_FONT_SHA256:
        parser.error("Embedded EWD font atlas does not identify the pinned source font")
    output.mkdir(parents=True)
    commands: list[dict[str, object]] = []

    def run(command: list[str]) -> str:
        completed = subprocess.run(command, text=True, capture_output=True)
        commands.append({"argv": command, "stdout": completed.stdout,
                         "stderr": completed.stderr, "returncode": completed.returncode})
        if completed.returncode:
            raise RuntimeError(f"Command failed ({completed.returncode}): {command!r}\n"
                               f"{completed.stdout}{completed.stderr}")
        return completed.stdout

    warnings = ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror"]
    test_binary = output / "test_avionics_core"
    run([clang, *warnings, str(CORE), str(TEST), "-lm", "-o", str(test_binary)])
    test_result = run([str(test_binary)]).strip()

    adirs_test_binary = output / "test_adirs"
    run([clang, *warnings, str(ADIRS), str(ADIRS_TEST), "-lm", "-o", str(adirs_test_binary)])
    adirs_result = run([str(adirs_test_binary)]).strip()
    adirs_adapter_binary = output / "test_adirs_adapter"
    adirs_window_binary = output / "test_adirs_window"
    for source, binary in ((ADIRS_ADAPTER_TEST, adirs_adapter_binary), (ADIRS_WINDOW_TEST, adirs_window_binary)):
        run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM300=1", "-DXPLM301=1", "-I", str(headers),
             str(source), str(ADIRS), "-lm", "-o", str(binary)])
    adirs_adapter_result = run([str(adirs_adapter_binary)]).strip()
    adirs_window_result = run([str(adirs_window_binary)]).strip()
    adirs_contracts = run([str(adirs_adapter_binary), "--contracts"]).splitlines()
    elec_binary = output / "test_elec_panel"
    run([clang, *warnings, str(ELEC_TEST), "-lm", "-o", str(elec_binary)])
    elec_result = run([str(elec_binary)]).strip()
    elec_adapter_binary = output / "test_elec_panel_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", f'-DELEC_TEST_ROOT="{HERE}"', "-I", str(headers),
         str(ELEC_ADAPTER_TEST), "-lm", "-o", str(elec_adapter_binary)])
    elec_adapter_result = run([str(elec_adapter_binary)]).strip()
    elec_contracts = run([str(elec_adapter_binary), "--contracts"]).splitlines()
    for source in FWS_SOURCES:   # gcc-only warnings must not first appear at the Windows link
        run([mingw, *warnings, "-c", str(source), "-o", str(output / (source.stem + "_win_check.o"))])
    run([_sys.executable, str(HERE.parent.parent / "tools/build_fws_tables.py"), "--check"])
    fws_test_results = {}
    for name, (test, sources) in FWS_TESTS.items():
        binary = output / test.stem
        run([clang, *warnings, str(test), *[str(HERE / source) for source in sources], "-lm", "-o", str(binary)])
        fws_test_results[name] = run([str(binary)]).strip().splitlines()[-1]
    for source in ELEC_NETWORK_SOURCES:   # gcc-only warnings must not first appear at the Windows link
        run([mingw, *warnings, "-c", str(source), "-o", str(output / (source.stem + "_win_check.o"))])
    elec_network_binary = output / "test_elec"
    run([clang, *warnings, str(ELEC_NETWORK_TEST), *map(str, ELEC_NETWORK_SOURCES), "-lm", "-o", str(elec_network_binary)])
    elec_network_result = run([str(elec_network_binary)]).strip()
    elec_network_adapter_binary = output / "test_elec_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", f'-DELEC_TEST_ROOT="{HERE}"', "-I", str(headers),
         str(ELEC_NETWORK_ADAPTER_TEST), *map(str, ELEC_NETWORK_SOURCES), "-lm", "-o", str(elec_network_adapter_binary)])
    elec_network_adapter_result = run([str(elec_network_adapter_binary)]).strip()
    elec_network_contracts = run([str(elec_network_adapter_binary), "--contracts"]).splitlines()
    ovhd_binary = output / "test_ovhd_lamps"
    run([clang, *warnings, str(OVHD_TEST), "-lm", "-o", str(ovhd_binary)])
    ovhd_result = run([str(ovhd_binary)]).strip()
    ovhd_adapter_binary = output / "test_ovhd_lamps_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", f'-DELEC_TEST_ROOT="{HERE}"', "-I", str(headers),
         str(OVHD_ADAPTER_TEST), *map(str, ELEC_NETWORK_SOURCES), "-lm", "-o", str(ovhd_adapter_binary)])
    ovhd_adapter_result = run([str(ovhd_adapter_binary)]).strip()
    ovhd_contracts = run([str(ovhd_adapter_binary), "--contracts"]).splitlines()
    lg_grvty_binary = output / "test_lg_grvty"
    run([clang, *warnings, str(LG_GRVTY_TEST), "-lm", "-o", str(lg_grvty_binary)])
    lg_grvty_result = run([str(lg_grvty_binary)]).strip().splitlines()[-1]
    rmp_binary = output / "test_rmp"
    run([clang, *warnings, str(RMP_TEST), "-lm", "-o", str(rmp_binary)])
    rmp_result = run([str(rmp_binary)]).strip().splitlines()[-1]
    run([mingw, *warnings, "-c", str(RMP_TEST), "-o", str(output / "test_rmp_win_check.o")])
    # gcc-only warnings must not first appear at the Windows link (the adapter itself is compiled
    # with -Werror into win.xpl by the unity build below).
    run([mingw, *warnings, "-c", str(LG_GRVTY_TEST), "-o", str(output / "test_lg_grvty_win_check.o")])
    lg_grvty_adapter_binary = output / "test_lg_grvty_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", "-I", str(headers), str(LG_GRVTY_ADAPTER_TEST), "-lm",
         "-o", str(lg_grvty_adapter_binary)])
    lg_grvty_adapter_result = run([str(lg_grvty_adapter_binary)]).strip()
    lg_grvty_contracts = run([str(lg_grvty_adapter_binary), "--contracts"]).splitlines()
    lg_grvty_stock_commands = [line[2:] for line in lg_grvty_contracts if line.startswith("X ")]
    thrust_binary = output / "test_thrust"
    run([clang, *warnings, str(THRUST_TEST), "-lm", "-o", str(thrust_binary)])
    thrust_result = run([str(thrust_binary)]).strip().splitlines()[-1]
    # gcc-only warnings must not first appear at the Windows link (the cores are header-only).
    run([mingw, *warnings, "-c", str(THRUST_TEST), "-o", str(output / "test_thrust_win_check.o")])
    detent_click_check = run([_sys.executable, str(DETENT_CLICK_GENERATOR), "--check"]).strip()
    flaps_binary = output / "test_flaps"
    run([clang, *warnings, str(FLAPS_TEST), "-lm", "-o", str(flaps_binary)])
    flaps_result = run([str(flaps_binary)]).strip().splitlines()[-1]
    # gcc-only warnings must not first appear at the Windows link (the core is header-only).
    run([mingw, *warnings, "-c", str(FLAPS_TEST), "-o", str(output / "test_flaps_win_check.o")])
    flaps_adapter_binary = output / "test_flaps_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", "-I", str(headers), str(FLAPS_ADAPTER_TEST), "-lm",
         "-o", str(flaps_adapter_binary)])
    flaps_adapter_result = run([str(flaps_adapter_binary)]).strip()
    flaps_contracts = run([str(flaps_adapter_binary), "--contracts"]).splitlines()
    flaps_stock_commands = [line[2:] for line in flaps_contracts if line.startswith("X ")]
    speedbrake_binary = output / "test_speedbrake"
    run([clang, *warnings, str(SPEEDBRAKE_TEST), "-lm", "-o", str(speedbrake_binary)])
    speedbrake_result = run([str(speedbrake_binary)]).strip().splitlines()[-1]
    # gcc-only warnings must not first appear at the Windows link (the core is header-only).
    run([mingw, *warnings, "-c", str(SPEEDBRAKE_TEST), "-o", str(output / "test_speedbrake_win_check.o")])
    speedbrake_adapter_binary = output / "test_speedbrake_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", f'-DSPEEDBRAKE_TEST_ROOT="{HERE}"', "-I", str(headers),
         str(SPEEDBRAKE_ADAPTER_TEST), "-lm", "-o", str(speedbrake_adapter_binary)])
    speedbrake_adapter_result = run([str(speedbrake_adapter_binary)]).strip()
    speedbrake_contracts = run([str(speedbrake_adapter_binary), "--contracts"]).splitlines()
    speedbrake_stock_commands = [line[2:] for line in speedbrake_contracts if line.startswith("X ")]

    def compile_laws(compiler, flags, suffix):
        objects = []
        for source in CXX_SOURCES:
            obj = output / (source.stem + suffix + ".o")
            run([compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                 "-fvisibility=hidden", *flags, "-c", str(source), "-o", str(obj)])
            objects.append(str(obj))
        return objects

    mac_laws = compile_laws(clang, ["-arch", "arm64", "-arch", "x86_64",
                                    "-mmacosx-version-min=11.0"], "_mac")
    mac_ewd_fadec = [obj for source, obj in zip(CXX_SOURCES, mac_laws) if source in EWD_FADEC_SOURCES]
    ewd_data_binary = output / "test_ewd_data"
    run([clang, *warnings, "-DAPL=1", "-DXPLM200=1", "-I", str(headers),
         str(EWD_DATA_TEST), *mac_ewd_fadec, "-lc++", "-lm", "-o", str(ewd_data_binary)])
    ewd_data_result = run([str(ewd_data_binary)]).strip()
    thrust_adapter_binary = output / "test_thrust_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
         "-DXPLM301=1", "-DXPLM400=1", f'-DTHRUST_TEST_ROOT="{HERE}"', "-I", str(headers),
         str(THRUST_ADAPTER_TEST), *mac_ewd_fadec, "-lc++", "-lm", "-o", str(thrust_adapter_binary)])
    thrust_adapter_result = run([str(thrust_adapter_binary)]).strip()
    thrust_contracts = run([str(thrust_adapter_binary), "--contracts"]).splitlines()
    ewd_render_binary = output / "test_ewd_render"
    run([clang, *warnings, str(EWD_RENDER_TEST), "-lm", "-o", str(ewd_render_binary)])
    ewd_previews = [output / f"ewd-{state}.svg" for state in ("inactive", "live", "warning")]
    ewd_render_result = run([str(ewd_render_binary), *map(str, ewd_previews)]).strip()
    font_data_uri = "data:font/ttf;base64," + base64.b64encode(EWD_FONT.read_bytes()).decode("ascii")
    for preview in ewd_previews:
        svg = preview.read_text()
        if svg.count("EWD_FONT_DATA_URI") != 1:
            raise RuntimeError(f"EWD preview missing its font placeholder: {preview}")
        preview.write_text(svg.replace("EWD_FONT_DATA_URI", font_data_uri))
    guidance_test_binary = output / "test_guidance"
    run([clang, *warnings, str(CORE), str(GUIDANCE), str(GUIDANCE_TEST),
         *mac_laws, "-lc++", "-lm", "-o", str(guidance_test_binary)])
    guidance_result = run([str(guidance_test_binary)]).strip()
    pfd_test_binary = output / "test_pfd_render"
    run([clang, *warnings, str(CORE), str(PFD_TEST), "-lm", "-o", str(pfd_test_binary)])
    pfd_result = run([str(pfd_test_binary), str(output / "pfd-climb.svg"),
                      str(output / "pfd-approach.svg"), str(output / "fcu-selected.svg"),
                      str(output / "fcu-managed.svg"), str(output / "efis-cp.svg"), str(output / "efis-nd.svg")]).strip()
    fcu_render_binary = output / "test_fcu_render"
    run([clang, *warnings, str(FCU_RENDER_TEST), "-lm", "-o", str(fcu_render_binary)])
    fcu_render_result = run([str(fcu_render_binary), str(output)]).strip()
    for preview in (output / "pfd-climb.svg", output / "pfd-approach.svg", output / "nd-fbw-arc.svg",
                    output / "nd-fbw-rose.svg", output / "nd-fbw-plan.svg"):
        preview.write_text(preview.read_text().replace("PFD_FONT_DATA_URI",font_data_uri))
    isis_binary = output / "test_isis"
    isis_preview_dir = output / "isis-previews"
    isis_preview_dir.mkdir()
    run([clang, *warnings, str(ISIS_TEST), "-lm", "-o", str(isis_binary)])
    isis_result = run([str(isis_binary), str(isis_preview_dir)]).strip()
    try:
        from PIL import Image
        for ppm in sorted(isis_preview_dir.glob("isis-*.ppm")):
            Image.open(ppm).save(ppm.with_suffix(".png"), optimize=True)
            ppm.unlink()
    except ImportError:
        pass
    isis_previews = [{"file": path.name, "sha256": digest(path)}
                     for path in sorted(isis_preview_dir.iterdir()) if not path.name.startswith("._")]
    efb_test_binary = output / "test_efb_render"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-I", str(headers),
         str(EFB_TEST), str(CORE), "-lm", "-o", str(efb_test_binary)])
    efb_result = run([str(efb_test_binary), str(output)]).strip()
    adapter_test_binary = output / "test_fcu_adapter"
    run([clang, *warnings, "-DAPL=1", "-DXPLM200=1", "-DXPLM300=1", "-I", str(headers), str(ADAPTER_TEST), str(GUIDANCE), str(CORE),
         *mac_laws, "-lc++", "-lm", "-o", str(adapter_test_binary)])
    adapter_result = run([str(adapter_test_binary)]).strip()
    fcu_mechanical_results = {}
    for source in (FCU_MOTION_TEST, FCU_LIGHT_TEST, CONTROL_MOTION_TEST):
        binary = output / source.stem
        run([clang, *warnings, str(source), "-lm", "-o", str(binary)])
        fcu_mechanical_results[source.stem] = run([str(binary)]).strip()
    control_lifecycle_binary = output / "test_control_lifecycle"
    run([clang, *warnings, "-DAPL=1", "-DXPLM200=1", "-I", str(headers),
         str(CONTROL_LIFECYCLE_TEST), "-lm", "-o", str(control_lifecycle_binary)])
    control_lifecycle_result = run([str(control_lifecycle_binary)]).strip()
    clock_render_binary = output / "test_clock_render"
    run([clang, *warnings, str(CLOCK_RENDER_TEST), "-o", str(clock_render_binary)])
    clock_render_result = run([str(clock_render_binary), str(output / "clock.svg")]).strip()
    efis_test_binary = output / "test_efis"
    run([clang, *warnings, "-DAPL=1", "-DXPLM200=1", "-DXPLM300=1", "-I", str(headers),
         str(EFIS_TEST), "-lm", "-o", str(efis_test_binary)])
    efis_result = run([str(efis_test_binary)]).strip()
    efis_contracts = run([str(efis_test_binary), "--contracts"]).splitlines()
    nd_test_binary = output / "test_efis_nd"
    run([clang, *warnings, str(EFIS_ND_TEST), "-lm", "-o", str(nd_test_binary)])
    nd_result = run([str(nd_test_binary)]).strip()
    airport_test_binary = output / "test_efis_airport"
    run([clang, *warnings, str(EFIS_AIRPORT_TEST), "-lm", "-o", str(airport_test_binary)])
    airport_result = run([str(airport_test_binary)]).strip()
    contracts = run([str(adapter_test_binary), "--contracts"]).splitlines()

    # ---- MFD / FMS core: objects once (host arch, no XPLM include path = proves XPLM-free) ----
    mfd_objects = []
    mfd_object_dir = output / "mfd-objects"
    mfd_object_dir.mkdir()
    for source in MFD_SOURCES:
        obj = mfd_object_dir / (source.stem + ".o")
        run([clang, *warnings, *mfd_page_flags, "-c", str(source), "-o", str(obj)])
        mfd_objects.append(str(obj))
    for source in MFD_SOURCES:   # gcc-only warnings must not first appear at the Windows link
        run([mingw, *warnings, *mfd_page_flags, "-c", str(source), "-o", str(mfd_object_dir / (source.stem + "_win.o"))])
    mfd_previews_root = output / "mfd-previews"
    mfd_test_results = {}
    mfd_previews = []
    for name, kind in [*MFD_TESTS, *mfd_optional_tests]:
        binary = output / name
        run([clang, *warnings, *mfd_page_flags, str(HERE / f"{name}.c"), *mfd_objects, "-lm", "-o", str(binary)])
        argv = [str(binary)]
        if kind == "navdata":
            argv.append(str(MFD_TEST_DATA / "navdata"))
        elif kind == "render":
            preview_dir = mfd_previews_root / name
            preview_dir.mkdir(parents=True)
            argv.append(str(preview_dir))
        lines = run(argv).strip().splitlines()
        mfd_test_results[name] = lines[-1] if lines else ""
        if kind == "render":
            mfd_previews.extend({"file": f"{name}/{p.name}", "sha256": digest(p)}
                                for p in sorted(preview_dir.iterdir()) if p.is_file() and not p.name.startswith("._"))
    mfd_adapter_binary = output / "test_mfd_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1",
         "-DXPLM300=1", "-DXPLM301=1", "-DXPLM400=1", "-DXPLM410=1", *mfd_page_flags, "-I", str(headers),
         str(MFD_ADAPTER_TEST), *mfd_objects, "-lm", "-o", str(mfd_adapter_binary)])
    mfd_adapter_result = run([str(mfd_adapter_binary)]).strip()
    mfd_contracts = run([str(mfd_adapter_binary), "--contracts"]).splitlines()
    sd_adapter_binary = output / "test_sd_adapter"
    run([clang, *warnings, "-Wno-deprecated-declarations", "-DAPL=1", "-DXPLM200=1", "-DXPLM210=1",
         "-DXPLM300=1", "-DXPLM301=1", "-DXPLM400=1", "-DXPLM410=1", *mfd_page_flags, "-I", str(headers),
         str(SD_ADAPTER_TEST), *mfd_objects, "-lm", "-o", str(sd_adapter_binary)])
    sd_adapter_result = run([str(sd_adapter_binary)]).strip()
    sd_contracts = run([str(sd_adapter_binary), "--contracts"]).splitlines()
    sd_data_flags = ["-Wno-deprecated-declarations", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1", "-DXPLM301=1",
                     "-DXPLM400=1", "-DXPLM410=1", "-I", str(headers)]
    sd_data_binary = output / "test_sd_data"
    run([clang, *warnings, "-DAPL=1", *sd_data_flags, str(SD_DATA_TEST), "-lm", "-o", str(sd_data_binary)])
    sd_data_result = run([str(sd_data_binary)]).strip()
    sd_data_contracts = run([str(sd_data_binary), "--contracts"]).splitlines()
    # gcc-only warnings (format-truncation) must not first appear in a Windows xp-kit build
    run([mingw, *warnings, "-DIBM=1", "-DSDD_COMPILE_ONLY", *sd_data_flags, "-c", str(SD_DATA_TEST),
         "-o", str(output / "test_sd_data_win_check.o")])
    fms_adapter_source = "".join((HERE / name).read_text() for name in ("a380x_fms_adapter.inc", "a380x_mfd_systems_host.inc"))
    fms_raw = set(re.findall(r'FMSA_RAW\(\s*"(sim/[^"\n]+)"', fms_adapter_source))
    if not fms_raw:
        raise RuntimeError("No FMS raw-input contracts were discovered")
    fms_dataref_names = {"a380x/xp12/fms/" + name for name in
                         re.findall(r'(?<!define )FMSR(?:_I)?\(\s*([a-z0-9_]+)\s*\)', fms_adapter_source)}
    fms_dataref_names |= {"a380x/xp12/fms/" + name for name in
                          re.findall(r'FMSR_(?:AS|CTL)\(\s*"([a-z0-9_]+)"', fms_adapter_source)}
    surv_dataref_names = {"a380x/xp12/surv/" + name for name in
                          re.findall(r'(?<!define )SURVR\(\s*"([a-z0-9_]+)"', fms_adapter_source)}
    center_right_raw = {
        name
        for filename in ("a380x_clock.inc", "a380x_center_right.inc")
        for name in re.findall(r'XPLMFindDataRef\(\s*"(sim/[^"\n]+)"', (HERE / filename).read_text())
    }
    ewd_input_contracts = re.findall(r'EWD_INPUT\(\s*"([^"\n]+)"', (HERE / "a380x_ewd_data.inc").read_text())
    ewd_raw = {name for name in ewd_input_contracts if name.startswith("sim/")}
    if not ewd_raw:
        raise RuntimeError("No EWD raw-input contracts were discovered")
    isis_source = ISIS_ADAPTER.read_text()
    isis_raw = set(re.findall(r'^\s*"(sim/[^"\n]+)",?$', isis_source, re.M))
    if len(isis_raw) != 24:
        raise RuntimeError("ISIS raw-input contracts were not discovered")
    raw_inputs = sorted({line[2:] for line in [*contracts, *efis_contracts, *adirs_contracts, *elec_contracts,
                                               *elec_network_contracts, *ovhd_contracts] if line.startswith("R ")}
                        | center_right_raw | ewd_raw | fms_raw | isis_raw
                        | {line[2:] for line in lg_grvty_contracts if line.startswith("R ")}
                        | {line[2:] for line in thrust_contracts if line.startswith("R ")}
                        | {line[2:] for line in flaps_contracts if line.startswith("R ")}
                        | {line[2:] for line in speedbrake_contracts if line.startswith("R ")}
                        | {line[2:] for line in sd_contracts if line.startswith("R ")}
                        | {line[2:] for line in sd_data_contracts if line.startswith("R ")})
    if args.datarefs:
        raw_catalog = {line.split()[0] for line in args.datarefs.read_text().splitlines() if line.strip()}
        missing_raw = set(raw_inputs) - raw_catalog
        if missing_raw:
            raise RuntimeError(f"Missing installed raw inputs: {sorted(missing_raw)}")
    installed_commands_verified = None
    if args.datarefs and args.datarefs.with_name("Commands.txt").is_file():
        # Stock commands the L/G GRVTY adapter intercepts or issues (landing_gear_*, emer_on/off).
        commands_catalog = args.datarefs.with_name("Commands.txt")
        command_names = {line.split()[0] for line in commands_catalog.read_text(errors="replace").splitlines()
                         if line.strip()}
        missing_commands = (set(lg_grvty_stock_commands) | set(flaps_stock_commands)
                            | set(speedbrake_stock_commands)) - command_names
        if missing_commands:
            raise RuntimeError(f"Missing installed stock commands: {sorted(missing_commands)}")
        installed_commands_verified = str(commands_catalog)
    windows_laws = compile_laws(mingw, [], "_win")

    # Keep package/manipulator declarations identical to the native registry.
    import sys
    sys.path.insert(0, str(HERE.parent.parent / "tools"))
    from avionics_provider import (KCCU_COMMANDS, KCCU_DATAREFS, MFD_COMMANDS, MFD_DATAREFS, FMS_DATAREFS,
                                   MFD_DEVICES, SURV_DATAREFS, SD_COMMANDS, SD_DATAREFS, SD_DEVICES)
    from isis_cockpit_device import ISIS_COMMANDS, ISIS_DATAREFS, ISIS_DEVICES, ISIS_CHANNELS
    def isis_array(array):
        body = re.search(array + r"\[[A-Z_]*\][^=]*=\s*\{([^}]*)\}", isis_source)
        return tuple(re.findall(r'"([a-z0-9_/.]+)"', body.group(1))) if body else ()
    def isis_names(array):
        return tuple(prefix + name for prefix in isis_array("isis_unit_prefixes") for name in isis_array(array))
    if (isis_names("isis_command_names") != ISIS_COMMANDS or isis_names("isis_ref_names") != ISIS_DATAREFS
            or isis_array("isis_device_ids") != ISIS_DEVICES):
        raise RuntimeError("ISIS native registry disagrees with cockpit provider contract")
    if ({line[2:] for line in sd_contracts if line.startswith("C ")} != set(SD_COMMANDS)
            or {line[2:] for line in sd_contracts if line.startswith("D ")} != set(SD_DATAREFS)
            or {line[2:] for line in sd_contracts if line.startswith("V ")} != set(SD_DEVICES)):
        raise RuntimeError("ECAM SD native registry disagrees with cockpit provider contract")
    sd_geometry = None
    eng_button_geometry = None
    if args.ecam_source:
        from sd_cockpit_device import controls_patch
        source = args.ecam_source.resolve()
        patch = output / "sd-controls.obj"
        patch.write_text(controls_patch(source))
        sd_geometry = {"source": str(source), "source_sha256": digest(source),
                       "controls_patch": patch.name, "controls_sha256": digest(patch)}
        if args.eng_button:
            from ecam_button_geometry import prepare_eng_geometry
            aircraft=HERE.parent.parent/'output/A380X-XP12'
            changes,eng_button_geometry=prepare_eng_geometry(source,aircraft)
            rows=[]
            for relative,encoded in changes.items():
                staged=output/'eng-button'/relative
                staged.parent.mkdir(parents=True,exist_ok=True)
                staged.write_text(encoded)
                rows.append({'path':relative,'file':str(staged.relative_to(output)),
                             'base_sha256':digest(aircraft/relative),'sha256':digest(staged)})
            eng_button_geometry['files']=rows
            eng_button_geometry['textures']=[{'path':eng_button_geometry[key],
                'sha256':digest(aircraft/eng_button_geometry[key])}
                for key in ('selection_atlas','selection_atlas_lit')]
            eng_button_geometry['source_sha256']=digest(source)
            eng_button_geometry['press_seconds']=.10
            eng_button_geometry['return_seconds']=.15
    from avionics_provider import ADIRS_COMMANDS, ADIRS_DATAREFS, CENTER_RIGHT_COMMANDS, CENTER_RIGHT_DATAREFS, CLOCK_COMMANDS, CLOCK_DATAREFS, COCKPIT_DOOR_COMMANDS, COCKPIT_DOOR_DATAREFS, EFIS_COMMANDS, EFIS_DATAREFS, DU_COMMANDS, DU_DATAREFS, FCU_COMMANDS, FCU_DATAREFS, FCU_MOTION_DATAREFS, FCU_LIGHT_DATAREFS, PFD_DATAREFS
    if ({line[2:] for line in adirs_contracts if line.startswith("C ")} | {"a380x/xp12/adirs/window_toggle"} != set(ADIRS_COMMANDS)
            or {line[2:] for line in adirs_contracts if line.startswith("D ")} != set(ADIRS_DATAREFS)):
        raise RuntimeError("ADIRS registry disagrees with cockpit provider contract")
    from avionics_provider import GRVTY_COMMANDS, GRVTY_DATAREFS, THRUST_COMMANDS, THRUST_DATAREFS, FILES as PROVIDER_FILES
    if ([line[2:] for line in thrust_contracts if line.startswith("C ")] != list(THRUST_COMMANDS)
            or [line[2:] for line in thrust_contracts if line.startswith("D ")] != list(THRUST_DATAREFS)):
        raise RuntimeError("Thrust-lever native registry disagrees with tools/avionics_provider.py")
    if not any(source.endswith(DETENT_CLICK.name) for source, _ in PROVIDER_FILES):
        raise RuntimeError("The thrust detent click is not packaged by tools/avionics_provider.py FILES")
    if ([line[2:] for line in lg_grvty_contracts if line.startswith("C ")] != list(GRVTY_COMMANDS)
            or [line[2:] for line in lg_grvty_contracts if line.startswith("D ")] != list(GRVTY_DATAREFS)):
        raise RuntimeError("L/G GRVTY native registry disagrees with tools/avionics_provider.py")
    from avionics_provider import FLAPS_COMMANDS, FLAPS_DATAREFS
    if ([line[2:] for line in flaps_contracts if line.startswith("C ")] != list(FLAPS_COMMANDS)
            or [line[2:] for line in flaps_contracts if line.startswith("D ")] != list(FLAPS_DATAREFS)):
        raise RuntimeError("FLAPS native registry disagrees with tools/avionics_provider.py")
    from avionics_provider import SPEEDBRAKE_COMMANDS, SPEEDBRAKE_DATAREFS
    if ([line[2:] for line in speedbrake_contracts if line.startswith("C ")] != list(SPEEDBRAKE_COMMANDS)
            or [line[2:] for line in speedbrake_contracts if line.startswith("D ")] != list(SPEEDBRAKE_DATAREFS)):
        raise RuntimeError("SPEED BRAKE native registry disagrees with tools/avionics_provider.py")
    from elec_panel_contract import COMMANDS as ELEC_COMMANDS, DATAREFS as ELEC_DATAREFS, SOUND_FILES as ELEC_SOUNDS
    if ({line[2:] for line in elec_contracts if line.startswith("C ")} != set(ELEC_COMMANDS)
            or {line[2:] for line in elec_contracts if line.startswith("D ")} != set(ELEC_DATAREFS)
            or [line[2:] for line in elec_contracts if line.startswith("S ")] != list(ELEC_SOUNDS)):
        raise RuntimeError("ELEC panel native registry or sound set disagrees with tools/elec_panel_contract.py")
    from elec_system_contract import (COMMANDS as ELEC_NETWORK_COMMANDS, DATAREFS as ELEC_NETWORK_DATAREFS,
                                      INTERCEPTED_COMMANDS as ELEC_NETWORK_INTERCEPTS)
    if ([line[2:] for line in elec_network_contracts if line.startswith("C ")] != list(ELEC_NETWORK_COMMANDS)
            or [line[2:] for line in elec_network_contracts if line.startswith("D ")] != list(ELEC_NETWORK_DATAREFS)
            or [line[2:] for line in elec_network_contracts if line.startswith("I ")] != list(ELEC_NETWORK_INTERCEPTS)
            or set(ELEC_NETWORK_DATAREFS) & set(ELEC_DATAREFS)):
        raise RuntimeError("ELEC network native registry disagrees with tools/elec_system_contract.py")
    from ovhd_lamps_contract import COMMANDS as OVHD_COMMANDS, DATAREFS as OVHD_DATAREFS, emit_c_table
    if (HERE / "a380x_ovhd_lamps_table.h").read_text() != emit_c_table():
        raise RuntimeError("a380x_ovhd_lamps_table.h is stale: python3 tools/ovhd_lamps_contract.py --emit-c")
    if ([line[2:] for line in ovhd_contracts if line.startswith("C ")] != list(OVHD_COMMANDS)
            or [line[2:] for line in ovhd_contracts if line.startswith("D ")] != list(OVHD_DATAREFS)
            or set(OVHD_DATAREFS) & (set(ELEC_DATAREFS) | set(ELEC_NETWORK_DATAREFS))):
        raise RuntimeError("Overhead lamp native registry disagrees with tools/ovhd_lamps_contract.py")
    from cockpit_feedback_contract import emit_c_table as feedback_table
    if (HERE / "a380x_cockpit_feedback_table.h").read_text() != feedback_table():
        raise RuntimeError("a380x_cockpit_feedback_table.h is stale: python3 tools/cockpit_feedback_contract.py --emit-c")
    from cockpit_feedback_contract import RMP_KEYS, RMP_KNOBS
    rmp_source = (HERE / "a380x_rmp_adapter.inc").read_text()
    rmp_keys = re.findall(r'"([a-z0-9_]+)"', rmp_source.split("rmp_key_names[] = {", 1)[1].split("};", 1)[0])
    rmp_knobs = re.findall(r'"([a-z0-9_]+)"', rmp_source.split("rmp_knob_names[RMP_KNOB_COUNT] = {", 1)[1].split("};", 1)[0])
    if rmp_keys != [key for _node, key in RMP_KEYS] or rmp_knobs != [ch for _node, ch in RMP_KNOBS]:
        raise RuntimeError("RMP native key / knob names disagree with tools/cockpit_feedback_contract.py")
    adapter = (HERE / "a380x_fcu_adapter.inc").read_text()
    command_names = {"a380x/xp12/fcu/" + name for name in re.findall(r'FCUC\("([^"\n]+)"', adapter)}
    dataref_names = {"a380x/xp12/fcu/" + name for name in re.findall(r'FCUR\("([^"\n]+)"', adapter)}
    registered_commands = {line[2:] for line in contracts if line.startswith("C ")}
    registered_datarefs = {line[2:] for line in contracts if line.startswith("D ")}
    if (len(FCU_COMMANDS) != len(set(FCU_COMMANDS)) or len(FCU_DATAREFS) != len(set(FCU_DATAREFS))
            or command_names != set(FCU_COMMANDS) or dataref_names != set(FCU_DATAREFS)
            or registered_commands != command_names or registered_datarefs != dataref_names):
        raise RuntimeError("FCU native registry disagrees with cockpit provider contract")
    if ({line[2:] for line in contracts if line.startswith("M ")} != set(FCU_MOTION_DATAREFS)
            or {line[2:] for line in contracts if line.startswith("L ")} != set(FCU_LIGHT_DATAREFS)):
        raise RuntimeError("FCU motion/light registry disagrees with cockpit provider contract")
    calibration_source = HERE.parent.parent / 'config/fcu-calibration.ini'
    shutil.copyfile(calibration_source, output / 'fcu-calibration.ini')
    if ({line[2:] for line in contracts if line.startswith("P ")} != set(PFD_DATAREFS)
            or len(PFD_DATAREFS) != len(set(PFD_DATAREFS))):
        raise RuntimeError("PFD native registry disagrees with cockpit provider contract")
    if ({line[2:] for line in efis_contracts if line.startswith("C ")} != set(EFIS_COMMANDS)
            or {line[2:] for line in efis_contracts if line.startswith("D ")} != set(EFIS_DATAREFS)):
        raise RuntimeError("EFIS native registry disagrees with cockpit provider contract")
    clock = (HERE / "a380x_clock.inc").read_text()
    clock_commands = {"a380x/xp12/clock/" + name
                      for name in re.findall(r'CLOCK_CMD\("([^"\n]+)"', clock)}
    clock_datarefs = {"a380x/xp12/clock/" + name
                      for name in re.findall(r'CLOCK_REF\("([^"\n]+)"', clock)}
    if (len(CLOCK_COMMANDS) != len(set(CLOCK_COMMANDS))
            or len(CLOCK_DATAREFS) != len(set(CLOCK_DATAREFS))
            or clock_commands != set(CLOCK_COMMANDS)
            or clock_datarefs != set(CLOCK_DATAREFS)):
        raise RuntimeError("Clock native registry disagrees with cockpit provider contract")
    center_right = (HERE / "a380x_center_right.inc").read_text()
    center_right_commands = {"a380x/xp12/center_right/" + name
        for name in re.findall(r'CR_CMD\("([^"\n]+)"', center_right)}
    center_right_datarefs = {"a380x/xp12/center_right/" + name
        for name in re.findall(r'CR_REF\("([^"\n]+)"', center_right)}
    if (center_right_commands != set(CENTER_RIGHT_COMMANDS)
            or center_right_datarefs != set(CENTER_RIGHT_DATAREFS)):
        raise RuntimeError("Center-right native registry disagrees with cockpit provider contract")
    cockpit_door = (HERE / "a380x_cockpit_door.inc").read_text()
    for command in COCKPIT_DOOR_COMMANDS:
        if cockpit_door.count('XPLMCreateCommand("' + command + '"') != 1:
            raise RuntimeError("Cockpit-door native command registry disagrees with provider contract")
    for dataref in COCKPIT_DOOR_DATAREFS:
        if cockpit_door.count('XPLMRegisterDataAccessor(\n        "' + dataref + '"') != 1:
            raise RuntimeError("Cockpit-door native dataref registry disagrees with provider contract")
    du_source = (HERE / "a380x_du.inc").read_text()
    du_channels = re.findall(r'\{"(captain_[a-z_]+|first_officer_[a-z_]+)", 1\.0f', du_source)
    du_commands = {f"a380x/xp12/du/{channel}_{direction}"
                   for channel in du_channels for direction in ("up", "down")}
    du_datarefs = {f"a380x/xp12/du/{channel}" for channel in du_channels}
    if (len(du_channels) != 8 or len(du_channels) != len(set(du_channels))
            or du_commands != set(DU_COMMANDS) or du_datarefs != set(DU_DATAREFS)):
        raise RuntimeError("DU native registry disagrees with cockpit provider contract")

    if ({line[2:] for line in mfd_contracts if line.startswith("C ")} != set(KCCU_COMMANDS) | set(MFD_COMMANDS)
            or {line[2:] for line in mfd_contracts if line.startswith("D ")} != set(KCCU_DATAREFS) | set(MFD_DATAREFS)
            or {line[2:] for line in mfd_contracts if line.startswith("V ")} != {device for device, _ in MFD_DEVICES}
            or fms_dataref_names != set(FMS_DATAREFS) or len(FMS_DATAREFS) != len(set(FMS_DATAREFS))
            or surv_dataref_names != set(SURV_DATAREFS)):
        raise RuntimeError("MFD native registry disagrees with cockpit provider contract")

    common = [*warnings, "-Wno-deprecated-declarations", "-fvisibility=hidden",
              "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1", "-DXPLM400=1", "-DXPLM410=1",
              *mfd_page_flags, "-I", str(headers)]
    mac = output / "mac.xpl"
    run([clang, *common, *(["-DA380X_XPKIT=1"] if args.xpkit else []), "-DAPL=1", "-arch", "arm64", "-arch", "x86_64",
         "-mmacosx-version-min=11.0", "-dynamiclib", str(PLUGIN), str(CORE), str(EFB), str(GUIDANCE), str(ADIRS),
         *map(str, ELEC_NETWORK_SOURCES), *map(str, FWS_SOURCES),
         *map(str, MFD_SOURCES), *mac_laws, "-lc++",
         "-F", str(sdk / "Libraries/Mac"), "-framework", "XPLM",
         "-framework", "OpenGL", "-Wl,-install_name,A380XAvionics.xpl",
         "-o", str(mac)])
    run(["codesign", "--force", "--sign", "-", str(mac)])
    run(["codesign", "--verify", "--strict", str(mac)])
    architectures = run(["lipo", "-archs", str(mac)]).split()
    if set(architectures) != {"arm64", "x86_64"}:
        raise RuntimeError(f"Unexpected macOS architectures: {architectures}")
    for arch in architectures:
        symbols = run(["nm", "-arch", arch, "-gjU", str(mac)]).split()
        if set(symbol.removeprefix("_") for symbol in symbols) != EXPORTS:
            raise RuntimeError(f"Unexpected exported macOS symbols: {symbols}")

    xpkit_manifest = None
    nd_input_result = None
    if args.xpkit:
        nd_input_binary=output / "test_xpkit_nd_input"
        run([clang,*warnings,"-Wno-unused-function","-Wno-deprecated-declarations","-DAPL=1",
             "-DXPLM200=1","-DXPLM210=1","-DXPLM300=1","-DXPLM400=1","-DXPLM410=1",
             "-I",str(headers),str(HERE / "test_xpkit_nd_input.c"),"-o",str(nd_input_binary)])
        nd_input_result=run([str(nd_input_binary)]).strip()
        from package_xp_kit_cockpit import package, reuse_verified_assets
        if args.xpkit_assets:reuse_verified_assets(args.xpkit_assets,output/'xp-kit')
        else:package(output / "xp-kit")
        host = output / "A380XDisplayHost"
        run([clang, *warnings, "-fobjc-arc", "-arch", "arm64", "-arch", "x86_64",
             "-mmacosx-version-min=11.0", str(HERE / "a380x_xpkit_host.m"),
             "-framework", "Cocoa", "-framework", "WebKit", "-framework", "CoreGraphics", "-o", str(host)])
        run(["codesign", "--force", "--sign", "-", str(host)])
        run(["codesign", "--verify", "--strict", str(host)])
        xpkit_manifest = json.loads((output / "xp-kit/manifest.json").read_text())
        xpkit_manifest["helper_sha256"] = digest(host)
        xpkit_manifest["architectures"] = run(["lipo", "-archs", str(host)]).split()

    windows = output / "win.xpl"
    run([mingw, *common, *(["-DA380X_XPKIT=1"] if args.xpkit_windows else []), "-DIBM=1", "-shared", str(PLUGIN), str(CORE), str(EFB), str(GUIDANCE), str(ADIRS),
         *map(str, ELEC_NETWORK_SOURCES), *map(str, FWS_SOURCES),
         *map(str, MFD_SOURCES), *windows_laws, "-static", "-lstdc++", "-lwinpthread",
         str(import_library), "-lopengl32", "-static-libgcc", "-lm",
         "-Wl,--exclude-all-symbols", "-o", str(windows)])
    windows_headers = run([objdump, "-p", str(windows)])
    windows_format = run([objdump, "-f", str(windows)])
    if "pei-x86-64" not in windows_format:
        raise RuntimeError("Windows plugin is not PE x86-64")
    table = windows_headers.split("[Ordinal/Name Pointer] Table", 1)
    if len(table) != 2:
        raise RuntimeError("Cannot locate Windows export names")
    names = set()
    for line in table[1].splitlines():
        if line.strip().startswith("["):
            names.add(line.split()[-1])
        elif names and line.strip():
            break
    if names != EXPORTS:
        raise RuntimeError(f"Unexpected Windows exports: {sorted(names)}")
    dlls = [line.split(":", 1)[1].strip() for line in windows_headers.splitlines()
            if "DLL Name:" in line]
    allowed = {"xplm_64.dll", "opengl32.dll", "kernel32.dll", "msvcrt.dll",
               "api-ms-win-crt-heap-l1-1-0.dll",
               "api-ms-win-crt-convert-l1-1-0.dll",
               "api-ms-win-crt-environment-l1-1-0.dll",
               "api-ms-win-crt-private-l1-1-0.dll",
               "api-ms-win-crt-runtime-l1-1-0.dll",
               "api-ms-win-crt-stdio-l1-1-0.dll",
               "api-ms-win-crt-string-l1-1-0.dll",
               "api-ms-win-crt-math-l1-1-0.dll",
               "api-ms-win-crt-time-l1-1-0.dll",
               # MFD core (2026-10-07): qsort (navdata CIFP sort) and localeconv (locale-independent
               # number formats). Same Universal CRT (ucrtbase.dll) as the API sets above; present on
               # Windows 10+ and with the UCRT redistributable. No new runtime library.
               "api-ms-win-crt-utility-l1-1-0.dll",
               "api-ms-win-crt-locale-l1-1-0.dll"}
    if not {name.lower() for name in dlls} <= allowed:
        raise RuntimeError(f"Unexpected Windows dependencies: {dlls}")

    windows_manifest = None
    if args.xpkit_windows:
        from build_windows_display_host import build as build_windows_host
        windows_manifest=build_windows_host(args.webview2_sdk,args.json_header,output/'windows-host',mingw,clang)

    report = {
        "format": 1,
        "xp_kit_nd_input_test_result": nd_input_result,
        "source_revision": "2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2",
        "logic_sources": [
            "fbw-a380x/src/systems/instruments/src/PFD/FMADefinitions.ts",
            "fbw-a380x/src/wasm/fbw_a380/src/model/A380FgOuterLoops.cpp",
            "fbw-a380x/src/wasm/fbw_a380/src/model/A380FcuComputer.cpp",
            "fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFg.cpp",
            "fbw-a380x/src/systems/instruments/src/PFD/PFD.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/PFDUtils.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/AttitudeIndicatorHorizon.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/AttitudeIndicatorFixed.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/SpeedIndicator.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/AltitudeIndicator.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/HeadingIndicator.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/VerticalSpeedIndicator.tsx",
            "fbw-a380x/src/systems/instruments/src/PFD/LowerArea.tsx",
            "src/systems/systems-host/CpiomC/FlightWarningSystem/FwsFlightPhases.ts",
            "fbw-a32nx/src/systems/instruments/src/EFB/Service/TODCalculator.ts",
            "fbw-a380x/src/systems/instruments/src/EFB/Assets/Checklists/checklists.json5",
            "fbw-common/src/systems/instruments/src/EFB/Efb.tsx",
            "fbw-common/src/systems/instruments/src/EFB/Assets/Theme.css",
            "fbw-common/src/systems/instruments/src/EFB/ToolBar/ToolBar.tsx",
            "fbw-common/src/systems/instruments/src/EFB/Dashboard/Dashboard.tsx",
            "fbw-a380x/src/systems/instruments/src/EWD/EWD.tsx",
            "fbw-a380x/src/systems/instruments/src/EWD/style.scss",
            "fbw-a380x/src/systems/instruments/src/EWD/elements/EngineGauge.tsx",
            "fbw-a380x/src/systems/instruments/src/EWD/elements/ThrustGauge.tsx",
            "fbw-a380x/src/systems/instruments/src/EWD/elements/N1.tsx",
            "fbw-a380x/src/systems/instruments/src/EWD/elements/EGT.tsx",
            "fbw-a380x/src/systems/instruments/src/EWD/elements/WdMemos.tsx",
            "fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/definitions.scss",
            "fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsMemos.ts",
            "fbw-a380x/src/wasm/fadec_a380x/src/Fadec/EngineControl_A380X.cpp",
            "fbw-a380x/src/systems/instruments/src/MFD/MFD.tsx",
            "fbw-a380x/src/systems/instruments/src/MFD/FMC/FlightManagementComputer.ts",
            "fbw-a380x/src/systems/instruments/src/MFD/FMC/FmcAircraftInterface.ts",
            "fbw-a380x/src/systems/instruments/src/MFD/pages/FMS",
            "fbw-a380x/src/systems/instruments/src/MFD/pages/common",
            "fbw-a380x/src/systems/instruments/src/MFD/shared",
            "fbw-common/src/systems/fmgc/src/flightplanning",
        ],
        "stock_xplane_efis_modes_consumed": True,
        "pfd_design_reference": {
            "product": "FlyByWire A380X",
            "scope": ("Original xp-kit library PFDs via aircraft-local WebKit renderer on macOS; native fallback on other platforms"
                      if args.xpkit else "open-source PFD coordinate geometry, symbology, and display algorithms translated to native OpenGL"),
            "source_revision": "2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2",
            "runtime_dependency": False,
            "imported_files": [],
        },
        "proprietary_reference": {
            "product": "FlightFactor A350",
            "scope": "observation only; not an implementation source",
            "runtime_dependency": False,
            "imported_files": [],
        },
        "display_targets": [
            "captain_pfd", "first_officer_pfd", "captain_nd", "first_officer_nd",
            "ewd", "captain_efb", "first_officer_efb", "fcu", "captain_efis", "first_officer_efis",
            "captain_mfd", "first_officer_mfd",
        ],
        "xplm_api": "XPLM410",
        "minimum_xplane": "12.1.0",
        "mock_test_result": test_result,
        "adirs_test_result": adirs_result,
        "adirs_adapter_test_result": adirs_adapter_result,
        "adirs_window_test_result": adirs_window_result,
        "adirs_commands": list(ADIRS_COMMANDS), "adirs_datarefs": list(ADIRS_DATAREFS),
        "elec_panel_test_result": elec_result,
        "elec_panel_adapter_test_result": elec_adapter_result,
        "elec_network_test_result": elec_network_result,
        "fws_test_results": fws_test_results,
        "elec_network_adapter_test_result": elec_network_adapter_result,
        "elec_network": {"commands": list(ELEC_NETWORK_COMMANDS), "datarefs": list(ELEC_NETWORK_DATAREFS),
                         "intercepted_stock_commands": list(ELEC_NETWORK_INTERCEPTS),
                         "fbw_source": "fbw-a380x/src/wasm/systems/a380_systems/src/electrical",
                         "preset_source": "fbw-a380x/src/base/flybywire-aircraft-a380-842/config/a380x/a380-842/aircraft_preset_procedures.xml",
                         "spec": "docs/ELEC_SYSTEM_SPEC.md", "behaviour": "docs/ELEC_SYSTEM.md"},
        "ovhd_lamps_test_result": ovhd_result,
        "ovhd_lamps_adapter_test_result": ovhd_adapter_result,
        "lg_grvty_test_result": lg_grvty_result,
        "rmp_test_result": rmp_result,
        "lg_grvty_adapter_test_result": lg_grvty_adapter_result,
        "lg_grvty": {"commands": list(GRVTY_COMMANDS), "datarefs": list(GRVTY_DATAREFS),
                     "raw_inputs": [line[2:] for line in lg_grvty_contracts if line.startswith("R ")],
                     "stock_commands": lg_grvty_stock_commands,
                     "installed_commands_verified": installed_commands_verified,
                     "fbw_source": ["fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/"
                                    "attachments/flybywire/Part_Interior_Cockpit/model/behaviour/mip.xml",
                                    "fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs "
                                    "A380GravityExtension, A380GearHydraulicController",
                                    "fbw-common/src/wasm/systems/systems/src/hydraulic/landing_gear.rs",
                                    "fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/"
                                    "FwsAbnormalSensed.ts 320700001"],
                     "xplane_gear_mapping": "gear_handle_down held while >= 1 turn; >= 3 turns: gear_handle_down=1 "
                                            "and sim/flight_controls/landing_gear_emer_on; below 1 turn: "
                                            "gear_handle_down = crew lever, then landing_gear_emer_off",
                     "behaviour": "docs/LG_GRVTY.md", "simulator_tested": False},
        "thrust_test_result": thrust_result,
        "thrust_adapter_test_result": thrust_adapter_result,
        "thrust_levers": {"commands": list(THRUST_COMMANDS), "datarefs": list(THRUST_DATAREFS),
                          "raw_inputs": [line[2:] for line in thrust_contracts if line.startswith("R ")],
                          "fbw_source": ["fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/"
                                         "attachments/flybywire/Part_Interior_Cockpit/model/behaviour/legacy/generated/"
                                         "A32NX_Interior_Engine.xml FBW_ENGINE_Lever_Throttle_SubTemplate2",
                                         "fbw-a380x/src/wasm/fbw_a380/src/FlyByWireInterface.cpp mappingTable3d, updateFadec",
                                         "fbw-a380x/src/wasm/fbw_a380/src/model/A380FadecComputer.cpp (+ _data.cpp)",
                                         "fbw-a380x/src/wasm/fadec_a380x/src/Fadec/EngineControl_A380X.cpp updateThrustLimits",
                                         "fbw-a380x/src/wasm/fadec_a380x/src/Fadec/ThrustLimits_A380X.hpp limitN1 (FLEX)",
                                         "fbw-a380x/src/systems/instruments/src/MFD/FMC/FlightManagementComputer.ts "
                                         "initSimVars ATHR_RESET_DISABLE"],
                          "detent_click": {"file": DETENT_CLICK.name, "sha256": digest(DETENT_CLICK),
                                           "generator": {"file": "tools/build_detent_click.py",
                                                         "sha256": digest(DETENT_CLICK_GENERATOR)},
                                           "check": detent_click_check, "synthesized": True},
                          "engine_command": "FBW FADEC N1 command as the IDLE..TOGA N1 fraction of X-Plane's throttle "
                                            "(override_throttles + ENGN_thro_use) while A/THR is not active",
                          "behaviour": "docs/THRUST_LEVERS.md", "simulator_tested": False},
        "flaps_test_result": flaps_result,
        "flaps_adapter_test_result": flaps_adapter_result,
        "flaps_lever": {"commands": list(FLAPS_COMMANDS), "datarefs": list(FLAPS_DATAREFS),
                        "raw_inputs": [line[2:] for line in flaps_contracts if line.startswith("R ")],
                        "stock_commands": flaps_stock_commands,
                        "installed_commands_verified": installed_commands_verified,
                        "fbw_source": ["fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/sfcc/mod.rs "
                                       "SlatFlapControlComputer (generate_configuration, status/position words)",
                                       "fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/sfcc/flaps_channel.rs, "
                                       "slats_channel.rs (demanded FPPU angles)",
                                       "fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs "
                                       "FLAP/SLAT_FPPU_TO_SURFACE_ANGLE_*, FlapSlatAssembly parameters",
                                       "fbw-common/src/wasm/systems/systems/src/hydraulic/flap_slat.rs FlapSlatAssembly",
                                       "fbw-common/src/wasm/systems/systems/src/hydraulic/command_sensor_unit.rs CSU",
                                       "fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFctl.cpp aileron droop",
                                       "fbw-a380x/src/systems/instruments/src/PFD/LowerArea.tsx SlatsFlapsDisplay",
                                       "fbw-a380x/src/systems/shared/src/PerformanceConstants.ts VfeF1..VfeFF"],
                        "xplane_flap_mapping": "flap_handle_request_ratio = FBW FlapsConf / 6 on the seven-detent "
                                               "ACF (tools/flaps_lever_cockpit.py ACF_CHANGES)",
                        "behaviour": "docs/FLAPS_LEVER.md", "simulator_tested": False},
        "speedbrake_test_result": speedbrake_result,
        "speedbrake_adapter_test_result": speedbrake_adapter_result,
        "speed_brake": {"commands": list(SPEEDBRAKE_COMMANDS), "datarefs": list(SPEEDBRAKE_DATAREFS),
                        "raw_inputs": [line[2:] for line in speedbrake_contracts if line.startswith("R ")],
                        "stock_commands": speedbrake_stock_commands,
                        "installed_commands_verified": installed_commands_verified,
                        "fbw_source": ["fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFctl.cpp (+ _data.cpp): "
                                       "speed_brake_command_deg, inhibition latch, ground_spoilers_armed/out, "
                                       "phased_lift_dumping, spoiler_lift_active, per-panel speed brake gains",
                                       "fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerGeneralLogic.cpp "
                                       "flap_handle_index",
                                       "fbw-a380x/src/wasm/fbw_a380/src/SpoilersHandler.cpp, FlyByWireInterface.cpp "
                                       "(speed_brake_lever_pos = -0.05 when armed)",
                                       "fbw-a380x/src/wasm/fbw_a380/src/fcdc/Fcdc.cpp discrete word 4 bits 27/28",
                                       "Part_Interior_Cockpit/model/behaviour/legacy/generated/A32NX_Interior_Handling.xml "
                                       "FBW_HANDLING_Lever_Spoilers_Template (ARM 0 / RET 1 / 1/2 2 / FULL 3), "
                                       "A380_COCKPIT_LOD00.gltf clip lever_speed_brake"],
                        "detent_click": {"file": DETENT_CLICK.name, "reused": True},
                        "xplane_speedbrake_mapping": "speedbrake_ratio = mean FBW panel order / 45 deg in flight, "
                                                     "/ 50 deg on the ground (never -0.5); acf_autosbrkEQ 0 while driven",
                        "behaviour": "docs/SPEED_BRAKE.md", "simulator_tested": False},
        "ovhd_lamps": {"commands": list(OVHD_COMMANDS), "datarefs": list(OVHD_DATAREFS),
                       "contract": "tools/ovhd_lamps_contract.py", "behaviour": "docs/OVERHEAD_LAMPS.md",
                       "fbw_source": "Part_Interior_Cockpit/model/A380_Cockpit_Behavior.xml SEQ_POWERED, SEQ1/SEQ2 codes"},
        "elec_panel": {"commands": list(ELEC_COMMANDS), "datarefs": list(ELEC_DATAREFS), "sounds": list(ELEC_SOUNDS),
                       "sound_provenance": json.loads((HERE / "sounds/elec_sounds_provenance.json").read_text())},
        "guidance_test_result": guidance_result,
        "pfd_render_test_result": pfd_result,
        "fcu_render_test_result": fcu_render_result,
        "efb_render_test_result": efb_result,
        "fcu_adapter_test_result": adapter_result,
        "fcu_mechanical_test_results": fcu_mechanical_results,
        "control_lifecycle_test_result": control_lifecycle_result,
        "clock_render_test_result": clock_render_result,
        "efis_test_result": efis_result,
        "efis_nd_test_result": nd_result,
        "efis_airport_test_result": airport_result,
        "ewd_data_test_result": ewd_data_result,
        "ewd_render_test_result": ewd_render_result,
        "sd_render_test_result": mfd_test_results["test_sd"],
        "sd_adapter_test_result": sd_adapter_result,
        "sd_data_test_result": sd_data_result,
        "ecam_sd": {"cockpit_devices": list(SD_DEVICES), "commands": list(SD_COMMANDS),
                    "datarefs": list(SD_DATAREFS), "pages": ["ENGINE","APU","BLEED","COND","PRESS","DOOR","ELEC AC","ELEC DC","FUEL","WHEEL","HYD","F/CTL","CB","CRUISE","STATUS","VIDEO"],
                    "native_only": True,"full_fws":False,"unavailable_inputs":"XX / amber; no preview fixtures", "geometry": sd_geometry,
                    "eng_button":eng_button_geometry},
        "isis": {"cockpit_devices": list(ISIS_DEVICES), "commands": list(ISIS_COMMANDS),
                 "datarefs": list(ISIS_DATAREFS), "lighting_channels": ISIS_CHANNELS, "texels": [1024, 1024],
                 "fbw_source": "fbw-a380x/src/systems/instruments/src/ISISlegacy",
                 "units": ["ISIS 1 (SFD at load)", "ISIS 2 (SND at load)"],
                 "raw_inputs": sorted(isis_raw), "test_result": isis_result, "previews": isis_previews,
                 "font": {"upstream_path": "fbw-a380x/src/base/flybywire-aircraft-a380-842/html_ui/Fonts/fbw-a380x/ISISFontTemporary.ttf",
                          "sha256": digest(ISIS_FONT),
                          "embedded_atlas": {"file": ISIS_FONT_HEADER.name, "sha256": digest(ISIS_FONT_HEADER)},
                          "generator": {"file": "tools/build_isis_font.py", "sha256": digest(ISIS_FONT_GENERATOR)}}},
        "ewd_input_contracts": ewd_input_contracts,
        "ewd_render_previews": [{"file": path.name, "sha256": digest(path)} for path in ewd_previews],
        "ewd_font": {
            "upstream_path": "fbw-a380x/src/fonts/FBW-Display-EIS-A380.ttf",
            "source_revision": vendor_manifest["revision"],
            "sha256": digest(EWD_FONT),
            "embedded_atlas": {"file": EWD_FONT_HEADER.name, "sha256": digest(EWD_FONT_HEADER)},
            "generator": {"file": "tools/build_ewd_font.py", "sha256": digest(EWD_FONT_GENERATOR)},
        },
        "efis_commands": list(EFIS_COMMANDS), "efis_datarefs": list(EFIS_DATAREFS),
        "mfd_ui_test_result": mfd_test_results["test_mfd_ui"],
        "mfd_formats_test_result": mfd_test_results["test_mfd_formats"],
        "fms_core_test_result": mfd_test_results["test_fms_core"],
        "navdata_test_result": mfd_test_results["test_navdata"],
        "mfd_render_test_result": mfd_test_results["test_mfd_render"],
        "mfd_page_test_results": {name: mfd_test_results[name] for name, _ in mfd_optional_tests},
        "mfd_adapter_test_result": mfd_adapter_result,
        "mfd_render_previews": mfd_previews,
        "mfd_page_modules": mfd_pages,
        "mfd_missing_page_modules": missing_pages,
        "mfd_font": {
            "embedded_atlas": {"file": MFD_FONT_HEADER.name, "sha256": digest(MFD_FONT_HEADER)},
            "generator": {"file": "tools/build_mfd_font.py", "sha256": digest(MFD_FONT_GENERATOR)},
            "source_sha256": re.findall(r"SHA-256: ([0-9a-f]{64})", MFD_FONT_HEADER.read_text()[:8192]),
        },
        "kccu_commands": list(KCCU_COMMANDS), "kccu_datarefs": list(KCCU_DATAREFS),
        "mfd_commands": list(MFD_COMMANDS), "mfd_datarefs": list(MFD_DATAREFS),
        "fms_datarefs": list(FMS_DATAREFS), "surv_datarefs": list(SURV_DATAREFS),
        "cockpit_devices": [{"id": device, "name": name, "width": 768, "height": 1024}
                            for device, name in MFD_DEVICES],
        "raw_inputs": raw_inputs,
        "installed_datarefs_verified": str(args.datarefs) if args.datarefs else None,
        "fcu_commands": list(FCU_COMMANDS), "fcu_datarefs": list(FCU_DATAREFS),
        "fcu_motion_datarefs": list(FCU_MOTION_DATAREFS), "fcu_light_datarefs": list(FCU_LIGHT_DATAREFS),
        "fcu_calibration": {"file":"fcu-calibration.ini", "sha256":digest(output/'fcu-calibration.ini'),
                            "factory_ratios":"linear source-TLA mapping assumption; verify hardware detents"},
        "pfd_datarefs": list(PFD_DATAREFS),
        "imported_fbw_laws": json.loads((HERE / "vendor/fbw/manifest.json").read_text()),
        "imported_ewd_fadec": ewd_fadec_manifest,
        "simulator_tested": False,
        "xp_kit_native_pfd": xpkit_manifest,
        "xp_kit_native_windows": windows_manifest,
        "sdk_root": str(sdk),
        "sdk_inputs": [{"path": str(path.relative_to(sdk)), "sha256": digest(path)}
                       for path in [*inputs, framework, import_library]],
        "sources": [{"name": str(path.resolve().relative_to(HERE)), "sha256": digest(path)}
                    for path in (PLUGIN, CORE, CORE_HEADER, EFB, EFB_HEADER, TEST, Path(__file__), *EXTRA_INPUTS,
                                 *mfd_page_sources, *[HERE / f"{name}.c" for name, _ in mfd_optional_tests])],
        "binaries": [
            {"file": mac.name, "sha256": digest(mac), "bytes": mac.stat().st_size,
             "architectures": architectures},
            {"file": windows.name, "sha256": digest(windows), "bytes": windows.stat().st_size,
             "architectures": ["x86_64"], "dependencies": dlls},
        ],
        "commands": commands,
    }
    (output / "build-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(test_result)
    print(f"Staged A380X avionics plugin: {output}")


if __name__ == "__main__":
    main()
