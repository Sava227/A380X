"""Exact, auditable control-surface provider contract (not a namespace exemption)."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil

PROVIDER_ID = "a380x-aileron-dance-v1"
MANIFEST = Path("animation-providers/aileron-dance-v1.json")
KEYS = tuple(f"{side}_{panel}_ratio" for side in ("l", "r")
             for panel in ("inboard", "midboard", "outboard"))
DATAREFS = tuple("a380x/xp12/visual/aileron_" + key for key in KEYS)
HYDRAULIC_DATAREFS = tuple("a380x/xp12/hydraulics/" + name for name in (
    *("aileron_" + key for key in KEYS), "elevator_ratio", "rudder_ratio",
    *(f"spoiler_{side}_{panel}_deg" for panel in ("inboard", "inner", "middle", "outer")
      for side in ("l", "r")),
))
ALL_DATAREFS = DATAREFS + HYDRAULIC_DATAREFS
DATAREF_CONTRACT = tuple(
    {"name": name, "type": "Float", "minimum": 0 if name.endswith("_deg") else -1,
     "maximum": 65 if name.endswith("_deg") else 1}
    for name in ALL_DATAREFS
)
# Project source, packaged source, global FlyWithLua destination.
FILES = (
    ("FlyWithLua/Scripts/A380X_XP12_Ailerons.lua",
     "plugins/FlyWithLua/Scripts (required)/A380X_XP12_Ailerons.lua",
     "Scripts/A380X_XP12_Ailerons.lua"),
    ("FlyWithLua/Modules/a380x_aileron_dance.lua",
     "plugins/FlyWithLua/Modules/a380x_aileron_dance.lua",
     "Modules/a380x_aileron_dance.lua"),
    ("FlyWithLua/Modules/a380x_hydraulic_surfaces.lua",
     "plugins/FlyWithLua/Modules/a380x_hydraulic_surfaces.lua",
     "Modules/a380x_hydraulic_surfaces.lua"),
    ("FlyWithLua/Scripts/A380X_XP12_Hydraulics.lua",
     "plugins/FlyWithLua/Scripts (required)/A380X_XP12_Hydraulics.lua",
     "Scripts/A380X_XP12_Hydraulics.lua"),
)
BOOTSTRAP_FILES = (
    ("native/aileron_datarefs/dist/mac.xpl", "plugins/A380XAileronDatarefs/mac_x64/A380XAileronDatarefs.xpl"),
    ("native/aileron_datarefs/dist/win.xpl", "plugins/A380XAileronDatarefs/win_x64/A380XAileronDatarefs.xpl"),
)
# The bootstrap plugin also DRIVES the 22 refs: a flight-loop C port of the two FlyWithLua drivers
# and their modules (native/aileron_datarefs/a380x_surface_motion.c), Lua-vs-C golden tested. While it
# runs it publishes NATIVE_DRIVER_FLAG = 1 and the global FlyWithLua drivers stand down.
NATIVE_DRIVER_FLAG = "a380x/xp12/hydraulics/native_driver"
NATIVE_DRIVER = {
    "plugin": "A380XAileronDatarefs", "flag_dataref": NATIVE_DRIVER_FLAG, "flag_type": "Int",
    "flight_loop": True, "writes_simulator_datarefs": False,
    "ports": [source for source, _, _ in FILES],
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def package_provider(root: Path, project: Path) -> dict:
    sources = [project / source for source, _, _ in FILES]
    bootstrap_sources = [project / source for source, _ in BOOTSTRAP_FILES]
    if not all(path.is_file() for path in sources + bootstrap_sources):
        raise ValueError("Missing required aileron/hydraulic visual driver/core or early-registration plugin")
    files = []
    for source, (_, packaged, installed) in zip(sources, FILES):
        target = root / packaged
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        files.append({"path": packaged, "install_path": installed, "sha256": digest(target)})
    bootstrap = []
    for source, (_, packaged) in zip(bootstrap_sources, BOOTSTRAP_FILES):
        target = root / packaged
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        bootstrap.append({"path": packaged, "sha256": digest(target)})
    document = {"format": 1, "provider": PROVIDER_ID, "runtime": "FlyWithLua NG 2.8.12 or later",
                "visual_only": True, "requires_global_installation": True,
                "datarefs": [dict(row) for row in DATAREF_CONTRACT], "files": files,
                "bootstrap": {"registration": "XPluginStart", "files": bootstrap},
                "native_driver": dict(NATIVE_DRIVER)}
    (root / MANIFEST).parent.mkdir(parents=True, exist_ok=True)
    (root / MANIFEST).write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    (root / "INSTALL_AILERON_VISUALS.txt").write_text(INSTALL_TEXT, encoding="utf-8")
    return document


INSTALL_TEXT = (
    "A380X aileron and hydraulic surface visual animations\n\n"
    "The included plugins/A380XAileronDatarefs plugin registers the 22 animation\n"
    "datarefs before X-Plane loads the objects and drives them itself from a flight\n"
    "loop: a native C port of the FlyWithLua drivers and modules listed below. No\n"
    "FlyWithLua installation is needed. While the native driver runs it publishes\n"
    "a380x/xp12/hydraulics/native_driver = 1; the FlyWithLua drivers then stand down,\n"
    "so there is never a second writer. Keep the plugin folder in this aircraft.\n\n"
    "Only custom visual values are written; flight physics and the simulator's\n"
    "flight controls are never changed. The six legacy and eight hydraulic ratios\n"
    "use [-1, 1]; the eight hydraulic spoiler angles use [0, 65] degrees.\n\n"
    "The packaged FlyWithLua files (plugins/FlyWithLua/) are the reference the native\n"
    "driver is tested against, frame by frame. They are only needed for an aircraft\n"
    "folder named A380X-XP12 whose plugin predates the native driver: install\n"
    "FlyWithLua NG (tested API 2.8.12), then, with X-Plane closed, copy\n"
    "plugins/FlyWithLua/Modules/*.lua to X-Plane 12/Resources/plugins/FlyWithLua/Modules/\n"
    "and plugins/FlyWithLua/Scripts (required)/*.lua to .../FlyWithLua/Scripts/.\n"
    "Back up existing files first. Do not enable the optional FBW control-law script.\n"
    "Offline tested (mock XPLM, Lua 5.4 golden comparison), not a simulator flight test.\n")


def validate_provider(root: Path) -> tuple[set[str], list[str]]:
    manifest = root / MANIFEST
    if not manifest.exists():
        return set(), []
    try:
        document = json.loads(manifest.read_text(encoding="utf-8"))
        if (document.get("format") != 1 or document.get("provider") != PROVIDER_ID
                or document.get("visual_only") is not True
                or document.get("requires_global_installation") is not True):
            raise ValueError("unsupported provider identity/contract")
        if document.get("datarefs") != list(DATAREF_CONTRACT):
            raise ValueError("expected the exact 22 scalar visual datarefs and their ranges")
        rows = document.get("files")
        if not isinstance(rows, list) or len(rows) != len(FILES):
            raise ValueError("expected both visual drivers and both core modules")
        for row, (_, packaged, installed) in zip(rows, FILES):
            if row.get("path") != packaged or row.get("install_path") != installed:
                raise ValueError("unexpected provider file path")
            path = root / packaged
            if (not path.resolve().is_relative_to(root.resolve()) or not path.is_file()
                    or digest(path) != row.get("sha256")):
                raise ValueError(f"missing, escaped or changed provider file: {packaged}")
        native = document.get("native_driver")
        if native is not None and native != NATIVE_DRIVER:
            raise ValueError("unexpected native driver contract")
        bootstrap = document.get("bootstrap", {})
        rows = bootstrap.get("files")
        if bootstrap.get("registration") != "XPluginStart" or not isinstance(rows, list) or len(rows) != 2:
            raise ValueError("missing early-registration aircraft plugin contract")
        for row, (_, packaged) in zip(rows, BOOTSTRAP_FILES):
            path = root / packaged
            if (row.get("path") != packaged or not path.resolve().is_relative_to(root.resolve())
                    or not path.is_file() or digest(path) != row.get("sha256")):
                raise ValueError(f"missing, escaped or changed bootstrap file: {packaged}")
        # The runtime test exercises registration; this manifest is a static
        # dependency proof only, never evidence that Lua loaded in X-Plane.
        return set(ALL_DATAREFS), []
    except (OSError, ValueError, TypeError, AttributeError) as error:
        return set(), [f"Invalid aileron animation provider: {error}"]


def verify_installed(root: Path, flywithlua: Path) -> dict:
    refs, errors = validate_provider(root)
    if errors or not refs:
        raise ValueError("; ".join(errors) or "Aileron provider is not packaged")
    files = []
    for _, packaged, installed in FILES:
        target = flywithlua / installed
        if not target.is_file() or digest(target) != digest(root / packaged):
            raise ValueError(f"Aileron runtime not installed or stale: {target}")
        files.append({"path": str(target), "sha256": digest(target)})
    return {"provider": PROVIDER_ID, "files_match_package": True,
            "runtime_execution_verified": False, "files": files}
