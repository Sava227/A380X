# Hydraulic surface settling — 2026-10-06

The exterior flight-control animation now reads actual X-Plane hydraulic
pressure. With the aircraft on the ground and pressure lost, all six ailerons
and all four elevator panels slowly settle down. The two rudder panels slowly
move downstream with the local wind. Spoilers hold their last supported
position and ignore roll/speedbrake commands until control power returns.
A cold, unpressurized load starts spoilers closed, even with a raised lever.

This is a pressure-aware exterior animation adapter. It does not replace
X-Plane's hydraulic or aerodynamic solver, alter pilot inputs, or write native
flight-control deflections. It is not a complete port of the A380 Green/Yellow
networks, electric actuator backup, individual actuator failures or RAT logic.
The installed aircraft's existing flight physics are retained.

## Pressure, motion and flight behavior

The adapter reads all three native `sim/cockpit2/hydraulics/indicators/`
`hydraulic_pressure_1`, `_2`, `_3` channels. They use Plane Maker pressure units,
not normalized ratios. This package configures nominal engine pressure of
3000 PSI. Available pressure is the maximum channel; 450 PSI releases the
surfaces and 900 PSI restores support, with hysteresis between the thresholds.
This models common support availability, not per-actuator source distribution.
Engine count is deliberately not a pressure proxy: running engines with failed
pumps do not prevent settling, and electric pumps can maintain pressure with
the engines stopped. The pressure thresholds and settling rates are development
approximations for this ACF, not certified A380 values.

Ground settling uses damped, rate-limited motion. Both wings settle trailing-edge
down despite their opposite animation signs. Elevators settle down separately
from stabilizer trim. Rates are limited to 0.045 clip ratio/second for ailerons,
0.04 for elevators and 0.05 for rudders. Most droop develops over tens of
seconds. Calm wind holds the rudder's previous free position. Wind direction
is relative to true aircraft heading: with the nose north, wind from the east
pushes the rudder trailing edge left. Stronger wind produces greater deflection;
there is no arbitrary oscillation. Ground airflow from 10 through 35 m/s
progressively unloads gravity droop. Pressure restoration returns surfaces
smoothly to their control commands, including spoilers.

Airborne loss of engine thrust does not activate parked droop. The primary
surface animations continue to follow native controls and the existing three-panel
aileron response; unsupported spoilers continue to hold their last positions. This preserves the intended visible failure behavior without
claiming the adapter simulates electrical emergency power. Airbus describes
RAT supply to EHA/EBHA during total engine flameout in
[The A380 Flight Control Electrohydrostatic Actuators](https://icas.org/icas_archive/ICAS2006/PAPERS/048.PDF),
PDF page 7. The fixed vertical fin is unchanged, and the horizontal stabilizer
continues to follow its existing trim input; only elevators and rudder panels
have passive motion.

The relaxed clip pose is derived from pinned FlyByWire revision
`2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2`,
`fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs` (aileron/elevator
factories and slow damping comment), and the corresponding WASM surface
aspects. Left ailerons use +20/30, right ailerons -20/30 and elevators -20/30.
These are source animation ratios; the exported mesh's geometric angle can
differ from the actuator-command angle. The source spoiler actuator restricts
upward movement without pressure; the user's requested parked behavior holds
all spoiler positions, including a spoiler left raised before shutdown.

## Runtime and build

The existing `A380XAileronDatarefs` aircraft plugin now registers 22 scalar
Float datarefs in `XPluginStart`: six legacy aileron-command outputs and sixteen
final surface-animation outputs under `a380x/xp12/hydraulics/`. The legacy
dance driver remains active. `A380X_XP12_Hydraulics.lua` reads those six values,
native pitch/yaw/spoiler channels, pressure, ground state and weather, then
runs the independent `a380x_hydraulic_surfaces.lua` state machine.

All four Lua files are packaged and required in global FlyWithLua. The provider
manifest lists exact names, types, ranges and hashes. The native bootstrap
clamps ratios to -1..1 and spoiler degrees to 0..65; all values reject NaN and
infinity. Missing references/types disable startup rather than creating late
OBJ datarefs. Pauses, invalid data and discontinuous frame periods hold the
last valid pose. The adapter is guarded to this A388 package and never writes
to native controls or overrides.

`config/animation_datarefs.json` persists the bindings for future exports.
The guarded rebinder changes animation-header dataref tokens only: mesh data,
keyframes, trim, flaps, gear and cockpit control mappings are retained.
`tools/install_hydraulic_surfaces.py` installs the staged update with original
hash verification, backups and rollback, while X-Plane is closed.

## Verification

`tests/hydraulic_surfaces_test.lua` exercises real state transitions, wind,
pressure recovery, airborne failure, frame timing and pause handling.
`tests/hydraulic_visual_mock_test.lua` exercises the installed wrapper contract,
including native-write prohibition. Native SDK mocks cover all 22 registration,
collision and rollback positions, plus scalar ranges. The geometry staging
report checks 510 token-only edits across 20 OBJ files. The independent
`tools/audit_hydraulic_surfaces.py` evaluates 38 affected draw cases and 2,830
unrelated draw comparisons, confirming downward droop and both rudder wind
directions while retaining neutral geometry.

These tests establish offline behavior and packaging. A simulator observation
is separate evidence and must not be inferred from a compiled plugin or mock.

## Installed — 2026-10-07

The update is installed in the aircraft package and global FlyWithLua runtime.
The native Mac universal and Windows x64 bootstrap binaries are included. The
installer verified all hashes and preserved the ACF. Originals and installation
evidence are in `build/hydraulics-backup-20261007`; independent geometry evidence
is in `build/hydraulics-candidate-v2/motion-report.json`. Five engine objects
were independently cleaned during preparation and are preserved unchanged.

A full X-Plane restart loads the new 22-reference bootstrap. Native simulator
observation has not yet been performed for this update.

Future builds also translate legacy cached exterior bindings during scoped
packaging. This was checked against all 20 installed objects byte-for-byte;
cached source exports and their material directives are unchanged. Package
validation reports zero errors.

## Native driver (A380XAileronDatarefs) — 2026-11-05

The FlyWithLua drivers only run for an aircraft folder named exactly `A380X-XP12` (their
`AIRCRAFT_PATH` gate), so a copy in any other folder (for example `A380X-XP12 Latest`) had frozen
elevators, rudder, ailerons and spoilers: the early-registration plugin owned the 22 datarefs but
nothing wrote them. The plugin now drives them itself.

* `native/aileron_datarefs/a380x_surface_motion.c` is a C port of both FlyWithLua drivers and both
  modules (`a380x_aileron_dance.lua`, `a380x_hydraulic_surfaces.lua`), in their FlyWithLua order
  (aileron dance, then the hydraulic settling that reads the published aileron values back). It runs
  from a flight loop (after the flight model, every frame) created in `XPluginEnable`, whatever the
  folder is called and without FlyWithLua.
* Visual only: it reads the same simulator datarefs as the Lua drivers and writes only its own 22
  Floats. No flight control, override or flight-model dataref is written (the mock test links no
  `XPLMSetData*` at all).
* Pause holds the pose (both cores); a frame period of 0, < 0, > 1 s or non-finite resets the aileron
  core and holds the hydraulic one, exactly as in Lua. `XPLM_MSG_PLANE_LOADED` (user aircraft) and
  `XPLM_MSG_AIRPORT_LOADED` re-seed both cores on the next frame, like FlyWithLua's script reload.
  Replay follows the replayed controls (nothing is written to the simulator, the frame-period guards
  cover discontinuities). Disable clears the 22 values and stops the loop; Stop unregisters them.
* `a380x/xp12/hydraulics/native_driver` (Int, read-only) is 1 while the native driver runs. The global
  FlyWithLua drivers look it up at load and read it every frame: while it is 1 they stand down without
  writing (`tests/native_driver_standdown_test.lua`), so there are never two writers. Their path gate is
  unchanged for the development aircraft, whose older plugin has no flag.
* Equivalence: `tests/surface_golden_lua.lua` runs the real, unmodified FlyWithLua scripts and modules
  under a FlyWithLua host model on 8 scripted scenarios (ground and air full deflections, roll doublets
  with spoilers, pressure loss and recovery through the 450/900 PSI hysteresis, cold aircraft with
  wind gusts / heading changes / the 0.25 m/s threshold, unpowered ground roll and lift-off, pause /
  invalid inputs / frame-period anomalies, random stress with mixed frame rates, reloads mid-flight).
  `test_surface_golden.c` replays the same inputs through the real plugin (mock XPLM, real flight
  loop and accessors) and requires every one of the 22 published values to be identical, frame by
  frame (10 470 frames, 230 340 values: bit-exact). Both are compiled with `-ffp-contract=off`.
  The reference interpreter is Lua 5.4; FlyWithLua NG uses LuaJIT, whose float `%` is
  `a - floor(a/b)*b` rather than 5.4's fmod form: an ulp-level difference in the wind angle that does
  not survive the float32 dataref (a mutation of that branch is not observable in the golden run).
* `native/aileron_datarefs/build.py` runs the mock test, the golden comparison and the MinGW check;
  its build report records the golden result and the sha256 of the four Lua sources it was run against.
