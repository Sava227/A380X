# A380X FBW-derived avionics plugin

The macOS `--xpkit` build runs the original FlyByWire PFD and ND/OANC libraries
**inside the actual 3D cockpit**. An aircraft-local WebKit helper renders seven
independent views. Native simulator/ADIRS/FCU/FMS data travels over shared memory;
PFDs retain their panel-atlas regions, and both NDs have higher-resolution cockpit
devices with yellow cursor, click, drag and zoom input. No browser tab, localhost
server or Web API is required by the avionics. Both MFDs use full-resolution
WebKit textures of the existing native FlyByWire-derived page ports; their shared
FMS, input and validation remain native. EWD and FCU drawing remain native.
The AFS CP LCDs use FBW's own FCU fonts (`A380X_FCU.ttf` values, Poppins-SemiBold
legends) from an embedded atlas (`tools/build_fcu_font.py`), each LCD at 1024 x 212
texels, with FBW's display strings (e.g. V/S `+12oo`, `~----`) and ghosted inactive
MACH/TRK/FPA legends; `test_fcu_render.c` writes pixel previews.
The EFIS CP LCDs redraw FBW's own dot-matrix ND-filter legends dot-for-dot
(`tools/build_efis_legends.py` -> `a380x_efis_legends.h`; VOR/pointer legends follow
VOR1/VOR2) on 1536 x 104 texel rows, and the 384 x 244 baro window uses the FBW
Digital font (`29.92`, `1013`, `Std`), QNH/QFE legends and the STD preset shown
once the knob is turned.
See [PFD integration](xp-kit/NATIVE.md), [ND/OANS integration](xp-kit-nd/NATIVE.md)
and [MFD integration and limits](xp-kit-mfd/README.md).
The earlier [browser trial](xp-kit/README.md) is still available for diagnostics.

For Windows x64, add `--xpkit-windows` to the `--xpkit` build. The Windows
`A380XDisplayHost.exe` uses Microsoft Edge WebView2, the same seven display
assets and a C/C++ ABI-checked local mapping. Native systems and all aircraft
writes remain in the plugin. Its isolated Python runtime supplies the OANS
scenery reader without a system Python installation. See
[Windows packaging, dependencies and acceptance checklist](../../docs/WINDOWS_AVIONICS.txt).
These binaries are cross-built developer artifacts: Windows X-Plane runtime
and WebView2 capture are not yet verified. Existing unported systems stay
partial/unavailable; the platform port does not complete the full MSFS aircraft.

This aircraft-local X-Plane 12 plugin is the runtime boundary for the avionics
port. It translates the pinned FlyByWire A380 FMA priority and flight-phase
logic into simulator-independent C, reads only raw X-Plane sensor, engine,
electrical, and failure inputs, and draws both PFD and ND pairs, the center EWD, plus the flyPad
into the aircraft panel texture. Both authored tablets map the flyPad pixels
and expose page, action, and power touch zones. X-Plane lists the runtime as
`A380X FBW Avionics + flyPad` in Plugin Admin. It reads the side-specific
X-Plane EFIS mode and range selector positions as cockpit inputs; it does not
consume X-Plane's stock FMS map, flight-director, VNAV, or autopilot mode status.

Without `--xpkit`, or on a Windows build without `--xpkit-windows`, the PFD is a native OpenGL translation of the pinned open-source FlyByWire A380
PFD coordinate geometry and algorithms, including its attitude mask, tapes,
FMA columns, heading ribbon, nonlinear pitch and vertical-speed mappings, and
lower display area. The installed FlightFactor A350 was observation-only; no
FlightFactor code, secured script, binary, texture, font, private dataref, or
other packaged file is used.

The xp-kit navigation display uses the aircraft's own active FMS plan and the
physical EFIS selectors. Its straight-leg geometry is partial: complete lateral
turn transitions, VNAV, traffic encounter classification, terrain and weather
images are not supplied. OANS reads local scenery, not a certified AMDB, and does
not command BTV braking. The non-xp-kit/stale-host fallback remains the existing
partial native ND, with its explicit unavailable-function indications.

The center EWD uses the authored `SCREEN_DU_EWD` face and a full 768 x 1024
region of the 2048 x 2048 cockpit atlas. Its four engine columns translate
the pinned FBW THR, N1 and EGT geometry, colors and embedded EIS font. Live
N1 is normalized to THR using the imported A380X FADEC idle/thrust-limit
tables and current environmental/bleed inputs; gauges respond to engine
state, reverse, unavailable inputs and over-limit values. The lower ECAM
area shows supported live engine/gear alerts and APU, seat-belt, parking-brake,
external-power and anti-ice memos. Full FWS procedures, electronic checklists,
and automatic thrust-rating selection remain unported. Optional producer
contracts enable rating, throttle-target and autothrust cues without inventing
their values. See [EWD development](../../docs/EWD_DEVELOPMENT.md) for the exact
boundary, differences and validation. The xp-kit build supplies the separate
System Display through the native ECAM state and the ported page renderer.

Both physical Display Unit Control Panels now command aircraft-local brightness
channels. The PFD and ND knobs dim each side's rendered screen independently,
including a fully dark zero setting. The ND outer WX/TERR and MFD channels are
registered for their respective future display layers; OIT, PFD/ND exchange,
and RECONF remain unavailable. See `../../docs/COCKPIT_CONTROLS.md` for the
knob and dataref mapping.

The two MFDs are plugin cockpit devices (`a380x.xp12.mfd.captain` /
`a380x.xp12.mfd.first_officer`, 768 x 1024) driven by the native FMS, navigation
database and flight-plan core (`a380x_mfd*.c`, `a380x_fms*.c`, `a380x_navdata.c`).
The device API needs **X-Plane 12.1 or newer** (12.4.0+ for reliable 3D hover and
drag); it is resolved with `XPLMFindSymbol`, so the plugin still loads on 12.0 and
the MFDs then open only as plain windows. See `../../docs/MFD_DEVELOPMENT.md`.

Build from a clean output directory:

```sh
python3 native/avionics/build.py \
  --sdk /path/to/XPSDK430/SDK \
  --xpkit \
  --output-dir /path/to/empty/staging-directory
```

The build compiles and runs the pure core tests before producing universal
macOS and Windows x86-64 plugin binaries.

The FCU now uses the unmodified pinned FlyByWire A380 `A380FgOuterLoops` C++
laws in `vendor/fbw`, plus a C command/mode adapter and X-Plane actuator
boundary. One shared state drives the physical FCU, PFD bugs, FMA and FD.
The two continuous purple LCD faces pair SPD/HDG and ALT/V/S at 512×128
texels each; all four targets retain independent native input and guidance.
Source hashes and the GPL license are retained with the import. This does not
import the complete PRIM/FMS/FADEC network. LAND/FLARE/ROLL OUT, RETARD and
SRS GA follow the FBW PRIM FG mode logic (`a380x_autoland.inc`,
`../../docs/AUTOLAND.md`). See `../../docs/AUTOPILOT_DEVELOPMENT.md` for the
explicit boundary and controls.

The build also runs numerical guidance, autoland mode-logic and closed-loop
bench (`test_autoland.c`, `test_autoland_sim.c`, report `autoland-bench.txt`),
actual PFD/FCU rendering, and XPLM boundary tests and checks native
command/dataref declarations against the cockpit package contract. Generated
SVG previews are offline render tests.

The three-unit ADIRS now drives the authored overhead controls and selected
PFD/ND/guidance validity. See [ADIRS operation and model boundary](../../docs/ADIRS.md)
for alignment, independent IR/ADR switching, ATT heading entry, source 3 and
airborne NAV lockout.

## Overhead annunciator lamps

`a380x_ovhd_lamps.h` (XPLM-free) lights the 217 legends of the 127 overhead pushbuttons
outside the ELEC and ADIRS panels: FlyByWire's SEQ1/SEQ2 condition (native APU ECB or the
X-Plane dataref the pushbutton drives) or ANN LT TEST, gated by FBW's per-pushbutton supply bus
from the native network. The table `a380x_ovhd_lamps_table.h` is generated from
`tools/ovhd_lamps_contract.py` (`--emit-c`; `build.py` fails if it is stale).
`a380x_ovhd_lamps.inc` publishes `a380x/xp12/ovhd/lamp/<id>_upper|_lower`, the ANN LT switch
(shared with `a380x/xp12/elec/ann_lt`, also lighting the ADIRS legends in TEST) and the opt-in
`settings/lamp_supply`. Tests: `test_ovhd_lamps.c`, `test_ovhd_lamps_adapter.c` (mock XPLM with
the production ELEC panel and network adapters). See [OVERHEAD_LAMPS.md](../../docs/OVERHEAD_LAMPS.md).

## L/G GRVTY (landing-gear gravity extension)

`a380x_lg_grvty.h` (XPLM-free) translates FlyByWire's `mip.xml` cover/guard/switch interlocks
and `A380GravityExtension` (handle +50 deg/s DOWN, -200 deg/s RESET, 0..1080 deg, turns =
floor(angle / 360); >= 1 turn hydraulics isolated, >= 2 door and >= 3 gear uplocks released).
`a380x_lg_grvty.inc` publishes `a380x/xp12/gear/grvty_*`, holds X-Plane's gear handle while the
hydraulics are isolated (the cockpit lever, `center_right/gear_toggle` and the stock
`sim/flight_controls/landing_gear_*` lever commands then move only the crew lever), selects the
gear down with `landing_gear_emer_on` at 3 turns and hands the handle back to the lever below one
turn before `landing_gear_emer_off`. The FWS reads the switch (320700001). Tests:
`test_lg_grvty.c`, `test_lg_grvty_adapter.c` (mock XPLM with the production center-right adapter).
See [LG_GRVTY.md](../../docs/LG_GRVTY.md).

## ISIS 1 / 2 (standby flight and navigation displays)

`a380x_isis.inc` is a native translation of FlyByWire's `ISISlegacy` (attitude with pitch
and roll scales, speed and altitude tapes, altitude drums, Mach, baro QNH/STD/inHg, LS
scales, BUGS page, ATT RST 10 s flag, 90 s power-up self-test, 10 s power ride-through).
It is XPLM/GL-free: it builds a display list of anti-aliased triangles for a 1024×1024
texel cockpit device (`a380x.xp12.isis.1`, lighting channel 21). `a380x_isis_adapter.inc`
binds it to X-Plane (standby pitot/static datarefs, standby baro, `a380x/xp12/isis/`
commands and datarefs, pop-up on screen click) and replays the list with OpenGL, only
when it changes (≤30 Hz). `test_isis.c` tests the logic and rasterises the identical list
into PNG previews (`docs/isis-previews/`). The FBW `Ecam` ISIS font is embedded as
`a380x_isis_font.h` (`tools/build_isis_font.py`). Physical buttons/knob and the device
quads are appended to the installed cockpit objects by `tools/isis_cockpit_device.py`
(via `tools/install_xp_kit_cockpit.py`). See [ISIS](../../docs/ISIS.md).

Two units: `a380x.xp12.isis.1` (SFD at load) and `a380x.xp12.isis.2` (SND at load, channel 22);
MODE swaps the roles. Key/knob feedback for the EFIS and ISIS controls comes from
`a380x_control_motion.h` (momentary ~80 ms eased travel, push/pull shafts, endless knobs;
`test_control_motion.c`); the panel OBJs are animated by `tools/patch_controls_animations.py`.
