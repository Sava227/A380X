#!/usr/bin/env python3
"""Mute X-Plane's built-in (legacy) alert sounds for the A380 (docs/COCKPIT_AURALS.md).

X-Plane plays its own alert samples (Resources/sounds/alert: stall horn, gear horn, fire bell, AP
disconnect, altitude alert, GPWS "pull up" / "sink rate" / "glideslope" / "minimums", radio-altitude
callouts, TCAS, windshear, seat-belt chime, ...) for every aircraft, on top of the aircraft's own
sounds. An aircraft replaces one by shipping a file of the same name in <aircraft>/sounds/alert/;
like other X-Plane 12 airliners, this writes a 10 ms silent WAV for each alert, so only the A380's own
FWS aurals (plugins/A380XAvionics/sounds/cockpit) remain. Radio audio is kept: marker beacons and
Morse idents (dash_/dot_*.wav) are not alerts. Engine, airframe and system sounds are not touched.

    python3 silence_xplane_alerts.py <aircraft folder>            write the silent overrides
    python3 silence_xplane_alerts.py <aircraft folder> --check    report, write nothing
    python3 silence_xplane_alerts.py <aircraft folder> --remove   delete them again (only our files)
"""
from __future__ import annotations

import argparse
import struct
import subprocess
import sys
from pathlib import Path

# X-Plane 12 alert samples (Resources/sounds/alert, plus autopilot_alt.wav that X-Plane also looks up).
ALERTS = (
    "10ft", "20ft", "30ft", "40ft", "50ft", "100ft", "200ft", "300ft", "400ft", "500ft", "1000ft",
    "alternator_off", "altitude_alert", "autopilot_alt", "autopilot_disco", "autopilot_fail",
    "fire_bell", "gear_warn_1", "gear_warn_2", "glideslope", "lo_rotor", "mini", "pull", "radar_lock",
    "seatbelt", "sink", "stall", "tcas", "te_climb", "te_descend", "transonic", "wshr",
)
RATE = 44100
FRAMES = 441   # 10 ms


def silent_wav() -> bytes:
    data = b"\0\0" * FRAMES
    fmt = struct.pack("<HHIIHH", 1, 1, RATE, RATE * 2, 2, 16)   # PCM, mono, 16-bit
    return (b"RIFF" + struct.pack("<I", 4 + 8 + len(fmt) + 8 + len(data)) + b"WAVE"
            + b"fmt " + struct.pack("<I", len(fmt)) + fmt + b"data" + struct.pack("<I", len(data)) + data)


def xplane_running() -> bool:
    return subprocess.run(["pgrep", "-x", "X-Plane"], capture_output=True).returncode == 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("aircraft", type=Path)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check", action="store_true")
    group.add_argument("--remove", action="store_true")
    args = parser.parse_args()
    if not any(args.aircraft.glob("*.acf")):
        sys.exit(f"not an aircraft folder (no .acf): {args.aircraft}")
    folder = args.aircraft / "sounds" / "alert"
    wav = silent_wav()
    state = {name: (folder / f"{name}.wav") for name in ALERTS}
    ours = [n for n, p in state.items() if p.is_file() and p.read_bytes() == wav]
    foreign = [n for n, p in state.items() if p.exists() and n not in ours]
    if args.check:
        print(f"{len(ours)}/{len(ALERTS)} muted in {folder}" + (f"; other files kept: {foreign}" if foreign else ""))
        return 0
    if xplane_running():
        sys.exit("X-Plane is running: close it first (X-Plane reads these files when the aircraft loads)")
    if args.remove:
        for name in ours:
            state[name].unlink()
        for d in (folder, folder.parent):
            if d.is_dir() and not any(d.iterdir()):
                d.rmdir()
        print(f"removed {len(ours)} silent overrides" + (f"; other files kept: {foreign}" if foreign else ""))
        return 0
    folder.mkdir(parents=True, exist_ok=True)
    written = 0
    for name, path in state.items():
        if name in foreign:
            continue   # never overwrite a sound someone else put there
        if name not in ours:
            path.write_bytes(wav)
            written += 1
    print(f"muted {len(ALERTS) - len(foreign)}/{len(ALERTS)} X-Plane alerts in {folder} ({written} written)"
          + (f"; other files kept: {foreign}" if foreign else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
