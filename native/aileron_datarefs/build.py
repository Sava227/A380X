#!/usr/bin/env python3
"""Compile/test a staged, never-installed plugin using an external official SDK.

Requires macOS clang, lipo, codesign, and MinGW-w64 for the Windows cross-build.
The output directory must not already exist; no package or simulator is touched.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess


HERE = Path(__file__).resolve().parent
PLUGIN = HERE / "aileron_datarefs.c"
# Native exterior surface driver: C port of the two FlyWithLua visual drivers (docs/HYDRAULIC_SURFACES.md).
CORE = HERE / "a380x_surface_motion.c"
CORE_HEADER = HERE / "a380x_surface_motion.h"
TEST = HERE / "test_aileron_datarefs.c"
GOLDEN_TEST = HERE / "test_surface_golden.c"
# Lua-vs-C golden reference: the real FlyWithLua drivers and modules under a FlyWithLua host model.
GOLDEN_LUA = HERE.parent.parent / "tests/surface_golden_lua.lua"
FLYWITHLUA = HERE.parent.parent / "FlyWithLua"
LUA_SOURCES = [FLYWITHLUA / "Scripts/A380X_XP12_Ailerons.lua", FLYWITHLUA / "Modules/a380x_aileron_dance.lua",
               FLYWITHLUA / "Scripts/A380X_XP12_Hydraulics.lua", FLYWITHLUA / "Modules/a380x_hydraulic_surfaces.lua"]
EXPORTS = {"XPluginStart", "XPluginStop", "XPluginEnable", "XPluginDisable",
           "XPluginReceiveMessage"}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, required=True,
                        help="Official SDK root containing CHeaders and Libraries")
    parser.add_argument("--output-dir", type=Path, default=HERE / "dist")
    parser.add_argument("--clang", default="clang")
    parser.add_argument("--mingw", default="/opt/homebrew/bin/x86_64-w64-mingw32-gcc")
    parser.add_argument("--lua", default="lua", help="Lua 5.3+ interpreter for the Lua-vs-C golden test")
    args = parser.parse_args()
    if platform.system() != "Darwin":
        parser.error("This build orchestrator requires a macOS host for universal Mac output")
    sdk, output = args.sdk.resolve(), args.output_dir.absolute()
    if output.exists() or output.is_symlink():
        parser.error("Output directory already exists; select a new staging directory")
    headers = sdk / "CHeaders/XPLM"
    framework = sdk / "Libraries/Mac/XPLM.framework/XPLM"
    import_library = sdk / "Libraries/Win/XPLM_64.lib"
    needed = [headers / name for name in
              ("XPLMDataAccess.h", "XPLMDefs.h", "XPLMPlugin.h", "XPLMProcessing.h", "XPLMUtilities.h")]
    for path in [*needed, framework, import_library]:
        if not path.is_file():
            parser.error(f"Missing official SDK input: {path}")
    mingw = shutil.which(args.mingw)
    clang = shutil.which(args.clang)
    objdump = shutil.which(args.mingw.removesuffix("gcc") + "objdump")
    lua = shutil.which(args.lua)
    if not all((mingw, clang, objdump, lua, shutil.which("codesign"), shutil.which("lipo"))):
        parser.error("Missing clang, MinGW gcc/objdump, Lua, codesign, or lipo")

    output.mkdir(parents=True, exist_ok=False)
    commands: list[dict] = []

    def run(command: list[str]) -> str:
        completed = subprocess.run(command, text=True, capture_output=True)
        commands.append({"argv": command, "stdout": completed.stdout,
                         "stderr": completed.stderr})
        if completed.returncode:
            raise RuntimeError(f"Command failed ({completed.returncode}): {command!r}\n"
                               f"{completed.stdout}{completed.stderr}")
        return completed.stdout

    # Define only API generations needed for aircraft/fat-plugin packaging;
    # none of the new SDK 4.3 flight-init APIs are used or imported. XPLM210 = XPLMCreateFlightLoop.
    # -ffp-contract=off: no fused multiply-add, so the driver's arithmetic is the tested Lua arithmetic.
    common = ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
              "-fvisibility=hidden", "-DXPLM200=1", "-DXPLM210=1", "-DXPLM300=1",
              "-I", str(headers)]
    test_binary = output / "test_aileron_datarefs"
    run([clang, *common, "-DAPL=1", str(PLUGIN), str(CORE), str(TEST), "-o", str(test_binary)])
    test_result = run([str(test_binary)]).strip()
    # Lua-vs-C golden test: the real FlyWithLua drivers write the reference, the real plugin replays it.
    golden = output / "surface-golden.txt"
    lua_result = run([lua, str(GOLDEN_LUA), str(FLYWITHLUA), str(golden)]).strip().splitlines()[-1]
    golden_binary = output / "test_surface_golden"
    run([clang, *common, "-DAPL=1", str(PLUGIN), str(CORE), str(GOLDEN_TEST), "-o", str(golden_binary)])
    golden_result = run([str(golden_binary), str(golden), "0"]).strip()
    lua_version = run([lua, "-v"]).strip()
    # gcc-only warnings must not first appear at the Windows link (the core is XPLM-free).
    run([mingw, *common, "-c", str(CORE), "-o", str(output / "a380x_surface_motion_win_check.o")])

    mac = output / "mac.xpl"
    run([clang, *common, "-DAPL=1", "-arch", "arm64", "-arch", "x86_64",
         "-mmacosx-version-min=11.0", "-dynamiclib", str(PLUGIN), str(CORE),
         "-F", str(sdk / "Libraries/Mac"), "-framework", "XPLM",
         "-Wl,-install_name,A380XAileronDatarefs.xpl", "-o", str(mac)])
    # Ad-hoc sign both slices for local Apple Silicon loading. Not notarization.
    run(["codesign", "--force", "--sign", "-", str(mac)])
    run(["codesign", "--verify", "--strict", str(mac)])
    architectures = run(["lipo", "-archs", str(mac)]).split()
    if set(architectures) != {"arm64", "x86_64"}:
        raise RuntimeError(f"Unexpected macOS architectures: {architectures}")
    for arch in architectures:
        symbols = run(["nm", "-arch", arch, "-gjU", str(mac)]).split()
        if set(symbol.removeprefix("_") for symbol in symbols) != EXPORTS:
            raise RuntimeError(f"Unexpected exported macOS symbols: {symbols}")
    mac_libraries = run(["otool", "-L", str(mac)])

    windows = output / "win.xpl"
    run([mingw, *common, "-DIBM=1", "-shared", str(PLUGIN), str(CORE), str(import_library),
         "-static-libgcc", "-Wl,--exclude-all-symbols", "-o", str(windows)])
    windows_headers = run([objdump, "-p", str(windows)])
    windows_format = run([objdump, "-f", str(windows)])
    if "pei-x86-64" not in windows_format:
        raise RuntimeError("Windows plugin is not PE x86-64")
    # The PE name-pointer table is an exact export list (ignore other tables).
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
    # Homebrew MinGW targets the Windows Universal CRT (part of Windows 10+).
    # No libgcc, libstdc++, winpthread, or other third-party DLL is permitted.
    allowed_dlls = {"xplm_64.dll", "kernel32.dll", "msvcrt.dll",
                    "api-ms-win-crt-heap-l1-1-0.dll", "api-ms-win-crt-math-l1-1-0.dll",
                    "api-ms-win-crt-private-l1-1-0.dll",
                    "api-ms-win-crt-runtime-l1-1-0.dll",
                    "api-ms-win-crt-stdio-l1-1-0.dll",
                    "api-ms-win-crt-string-l1-1-0.dll"}
    if not {name.lower() for name in dlls} <= allowed_dlls:
        raise RuntimeError(f"Unexpected external Windows runtime dependencies: {dlls}")

    report = {
        "format": 1, "staged_only": True, "simulator_tested": False,
        "visual_only": True, "mock_test_result": test_result,
        "native_driver": {
            "flag_dataref": "a380x/xp12/hydraulics/native_driver", "flag_type": "Int", "flag_writable": False,
            "flight_loop": "XPLMCreateFlightLoop, after flight model, every frame (from XPluginEnable)",
            "ports": ["FlyWithLua/Scripts/A380X_XP12_Ailerons.lua", "FlyWithLua/Modules/a380x_aileron_dance.lua",
                      "FlyWithLua/Scripts/A380X_XP12_Hydraulics.lua",
                      "FlyWithLua/Modules/a380x_hydraulic_surfaces.lua"],
            "writes_simulator_datarefs": False,
            "golden_reference": {"interpreter": lua_version, "generator": lua_result, "result": golden_result,
                                 "tolerance": 0,
                                 "lua_sources": [{"path": str(path.relative_to(HERE.parent.parent)),
                                                  "sha256": digest(path)} for path in LUA_SOURCES]},
        },
        "registration_phase": "XPluginStart", "dataref_count": 22,
        "type": "Float", "writable": True,
        "ranges": [{"suffix": "_ratio", "count": 14, "minimum": -1, "maximum": 1},
                   {"suffix": "_deg", "count": 8, "minimum": 0, "maximum": 65}],
        "nonfinite_write_policy": "ignore; retain last finite cached value",
        "sdk_source": "https://developer.x-plane.com/sdk/plugin-sdk-downloads/",
        "sdk_root": str(sdk),
        "sdk_inputs": [{"path": str(path.relative_to(sdk)), "sha256": digest(path)}
                       for path in [*needed, framework, import_library]],
        "sources": [{"name": path.name, "sha256": digest(path)}
                    for path in (PLUGIN, CORE, CORE_HEADER, TEST, GOLDEN_TEST, GOLDEN_LUA, Path(__file__))],
        "binaries": [
            {"file": mac.name, "sha256": digest(mac), "bytes": mac.stat().st_size,
             "architectures": architectures, "signing": "ad-hoc, verified; not notarized",
             "dependencies": mac_libraries},
            {"file": windows.name, "sha256": digest(windows),
             "bytes": windows.stat().st_size, "architectures": ["x86_64"],
             "dependencies": dlls, "minimum_os": "Windows 10 (Universal CRT)",
             "runtime_tested": False}],
        "commands": commands,
    }
    report_path = output / "build-report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(test_result)
    print(golden_result)
    print(f"Staged Mac universal + Windows x64 plugin: {output}")
    print(f"Build evidence: {report_path}")


if __name__ == "__main__":
    main()
