# Control-surface animation repair — 2026-10-04

This records offline geometry/animation evidence, not native X-Plane or flight
acceptance. No flight-model tuning, control-law enablement or global simulator
preference change is part of this repair.

## Horizontal stabilizer and four elevator panels — v3 correction

The all-moving stabilizer uses its actual degree-valued X-Plane output. The four
elevator visuals use the live pilot pitch command as a runtime fallback:

| FBW animation | X-Plane dataref | Authored input range |
| --- | --- | ---: |
| Left outer elevator | `sim/joystick/yoke_pitch_ratio` | −1 to +1 |
| Left inner elevator | `sim/joystick/yoke_pitch_ratio` | −1 to +1 |
| Right outer elevator | `sim/joystick/yoke_pitch_ratio` | −1 to +1 |
| Right inner elevator | `sim/joystick/yoke_pitch_ratio` | −1 to +1 |
| All-moving stabilizer | `sim/flightmodel2/controls/stabilizer_deflection_degrees` | −10.218274 to +7.513818 degrees |

The October 4 runtime probe found the indexed `flightmodel2/wing` mirrors and
all four legacy scalar horizontal-tail degree outputs at zero in the loaded
A380 while the control-surface override and optional FBW bridge were disabled;
the user's preceding control test produced no visible elevator movement. The
visual elevators therefore follow the raw, live pitch input instead of relying
on those surface-solver outputs. This is a visual fallback, not aerodynamic
feedback: the four separate meshes currently share one input, not independent
FBW actuator commands. Positive pitch input raises their trailing edges.

The user's 16:11 screenshot exposed an error in the previous repair: tiny end
pieces moved while the main elevators stayed flat. The two optimized tail
skins use **one-based** positive-weight joint indices 1..11. Reading them as
standard zero-based glTF attached parts to neighbouring joints. The previous
right-palette permutation compensated for symptoms but did not fix this cause.
Those older symmetric/bounds-only checks did not establish correct ownership.

`repair_fbw_tail_bindshape.py` v3 retains the verified bind-shape transform,
subtracts one from positive-weight joint lanes, and preserves both original
palettes and inverse binds. Exact mesh/family/range guards reject unexpected
source or stale v2 intermediates. Both preparation entrypoints protect these
lanes from legacy clamping before the source-specific validator sees them.
Rebuild from the original source, not a previously repaired glTF.

`--rigid-fbw-tail-panels` consolidates only verified same-family weights. The
complete inner panels (Cube.118/.101, 620 triangles each) and outer panels
(Cube.119/.100, 572 each) follow their respective elevator hinges. Six fixed
stabilizer/hardware meshes, 11,972 triangles, follow trim but not pitch. The
source-based approximation error is at most 2.410 micrometres over all 21
authored elevator keys. No fuselage-wide static-pose bake is used.

Native stabilizer degrees are **positive leading-edge up**, unlike aircraft
nose-up trim terminology (installed DataRefs.txt, stabilizer_deflection_degrees).
The mapping now uses 21 actual signed angles measured from the source trim
quaternions relative to the midpoint. The clip is asymmetric and was never a
symmetric ±10-degree interval. Export evaluation at native −10/+2 produces
10.00000171°/2.00000006° rotation, with the correct leading-edge direction.
No ACF trim limits or flight-model settings were changed.

Independent v3 export checks cover all seven fuselage OBJ files / 445,150
triangles. Triangle-to-UV associations are exact; maximum neutral corner
difference from the previous export is 0.0524 mm. Posed non-tail geometry,
including rudder and gear doors, is unchanged. Previously connected pieces
separated by up to 1.067 m under pitch; the corrected coincident boundaries
stay within 0.00312 mm across pitch, trim, and combined endpoint samples.

The verified candidate remains at `xplane-obj/fuselage-tail-joints-v3-candidate`
under the external build root. It is now installed in the SD-card aircraft and
promoted to `xplane-obj/fuselage-corrected`. The guarded update replaced only
seven fuselage OBJ files in each location; original files and the hash-verified
install report are retained in `tail-v3-install-20261004`. The ACF is unchanged.
No native X-Plane movement test has been performed for this revision.
Reports in `/Volumes/MacGames/A380X-XPlane12-build/reports/`:

- `tail-v3-joint-family-audit.json`: raw pinned-source indices and motion audit.
- `fuselage-tail-joints-v3-export.json`: ten-panel preparation/export provenance.
- `tail-v3-independent-parity.json`: full neutral and unrelated-control parity.
- `tail-v3-independent-partition-motion.json`: complete-panel movement, seams,
  mirrored motion, trim angles, and sign.
- `tail-v3-neutral-preview.png` and `tail-v3-pitch-up-preview.png`: offline
  textured geometry previews, not simulator screenshots.
- `tail-v3-installed-package-validation.json`: installed package, zero errors;
  existing aileron runtime-verification and missing-custom-FMOD-bank warnings.

Recheck in X-Plane from behind: pitch moves the full trailing-edge elevators;
trim moves the whole horizontal stabilizer, carrying the elevators with it.
Live elastic tail flex, FBW hydraulics, and native Windows testing remain open.

## Root cause: optimized wing joint indices

The pinned source's two 63-joint wing skins use one-based weighted joint indices
in ASOBO 4.6 BLEND4-v2 and BLEND1-v2 vertex layouts. Their positive-weight lanes
span 1..63, never zero; standard glTF expects 0..62. Previously, the importer
only clamped invalid index 63 to 62. That allowed loading, but left the other
vertices bound to neighbouring, incorrect bones.

For example, the left inboard flap indices 54/58/62 pointed to spoiler,
multifunction and neutral bones. Subtracting one attaches them to the intended
FLAP4/3/2 bones. Right aileron meshes Cylinder.255/.256/.257 similarly become
AILERON14/13/12 rather than neighbouring slat/aileron bones. This is supported
by source membership and evaluated geometry, not just names.

`blender/repair_fbw_wing_joints.py` is explicitly opt-in and source-scoped.
It checks version, named palettes, joint count, layout, accessor ranges and
weighted index evidence before appending new JOINTS accessors. Zero-weight
padding becomes zero. Source buffers, vertices, weights, UVs and animation
curves remain unchanged. Both normalization entrypoints run it before legacy
clamping. Rebuild from upstream; do not repair already-clamped intermediates.

The repaired subset has 63 affected primitives / 208,231 vertices and requires
zero legacy joint-index clamps. This does not establish the same convention for
all MSFS assets; no other aircraft part is changed by this adapter.

## Complete main flap panels

After the index repair, 8,476 of 13,524 main flap triangles still had blended
weights and were frozen by the generic rigid-only OBJ8 conversion. Splitting
them from the rigid parts made holes when flaps extended.

`--rigid-fbw-flap-panels` instead preserves each of the six complete, baked-neutral
skins and attaches it to a representative source flap joint: FLAP3 for inboard,
FLAP7 for middle and FLAP10 for outboard, on each side. Source names, polygon
counts, exact allowed influence sets, prior static-pose bake and corrected
palette are required. No vertices, UVs or neutral geometry are edited by this
step. All 13,524 panel triangles now follow the flap dataref.

This is a **bounded rigid approximation**, not exact MSFS skinning. An independent
raw-source calculation checks every authored flap key (41 poses on each panel),
comparing weighted skinning against the representative joint's motion applied
to the same weighted-neutral geometry:

| Panel | Left maximum position error | Right maximum position error |
| --- | ---: | ---: |
| Inboard | 31.643 mm | 31.648 mm |
| Middle | 23.802 mm | 22.433 mm |
| Outboard | 19.565 mm | 20.702 mm |

Thus the checked poses stay within 32 mm of the source. This is not a bound on
unmeasured continuous-time interpolation. Neutral mathematical error is below
1e-12 m. Worst area-weighted geometric face-normal RMS is about 0.475 degrees.
Large angular outliers occur on tiny/sliver triangles: over-5-degree errors
cover at most 0.009529% of a panel's area; over-90-degree outliers cover at most
0.000010573%. These are geometric normals, not a PBR/shading validation.

The actual exported OBJ animation paths were independently checked against that
raw weighted source at all 41 poses, matching used vertices by UV and neutral
position. Worst export-to-source error is 31.64870 mm; neutral error is at most
32.587 micrometres. Comparing the palette-only and complete-panel exports across
all 739,929 triangles also preserves neutral geometry within 30.419 micrometres
and preserves every triangle's UV coordinates, independent of index/draw order.

## Aileron behavior

All six authored clips retain their original curves and existing roll mapping.
Each has 201 quaternion samples; fraction 0.5 matches the rest quaternion, and
both endpoints are 25 degrees from neutral. No right-side sign inversion was
added. The corrected source evaluation confirms right roll moves left trailing
edges down and right trailing edges up, with the opposite result for left roll.
Each side has 1,660 main aileron triangles (604 outboard, 532 middle, 524 inboard).

This is still a native-roll-ratio visual mapping. It does not implement the FBW
independent droop, antidroop, high-speed outboard scheduling or hydraulic effects.
The subsequent spoiler repair is described below. The motion-only repair is not a
claim that physical surface deflections match a validated A380 control law.

## Complete spoiler skins and native differential control

The sixteen spoiler meshes contain 45,076 triangles across 32 material variants.
Previously 11,816 blended triangles stayed frozen while the remaining skins
opened. `--rigid-fbw-spoiler-panels` consolidates each moving panel onto an audited
representative source bone, restoring those triangles. All 45,030 moving triangles
follow their panels; 46 neutral-only triangles on left spoiler 2 are detached
fixed hardware and deliberately retain their original weights. No faces span
the moving/fixed boundary. Exact object names, materials, triangle counts,
influences, corrected palette and prior static-pose bake are required before
any weights change. Neutral vertices, UVs and topology are not edited.

The representative bone suffixes for panels 1 through 8 are 1, 1, 1, 2, 2, 2,
1, 1 on both sides. The source's `WING_SPOILER55_1` spelling is intentional.
These are rigid approximations, not exact blended skinning. Independent source
evaluation checks all 41 authored keys; the mathematical approximation is within
17.481 mm over the full 65-degree clip and 13.747 mm through 50 degrees. The
tiny fixed-hardware exception is excluded from moving-panel consolidation.

Five left `WING_MFTF_{3,5,7,9,11}.L` targets were incorrectly owned by the
higher-priority flap clip. That clip has constant translation and rotation
matching node rest, whereas spoiler 1 supplies real travel. A read-only glTF
verifier proves this exact condition before omitting only those redundant flap
tracks. The source rest rotation remains intact and the spoiler clip now owns
the linkage motion on both sides. This is not a generic additive composition
of independent flap/spoiler transforms.

Eight existing native output channels drive sixteen visible panels:

| FBW panels | Left / right X-Plane wing datarefs | Source travel cap |
| --- | --- | ---: |
| 1–2 | `speedbrake1_deg[0]` / `[1]` | 35 degrees |
| 3 | `spoiler1_deg[0]` / `[1]` | 50 degrees |
| 4–6 | `spoiler1_deg[2]` / `[3]` | 50 degrees |
| 7–8 | `spoiler2_deg[4]` / `[5]` | 50 degrees |

All names have prefix `sim/flightmodel2/wing/`. Panels 1–2 stay speedbrake-only;
the others use outputs that already combine roll and speedbrake. No extra
additive speedbrake transform, Lua dependency, custom dataref, control override
or flight-model change is introduced. Shared native segments are a documented
spatial/function approximation, not sixteen independent actuators. Panel 4
straddles a native segment boundary. Actual deflection follows the existing ACF:
panels 7–8 currently reach 45 degrees, and full in-flight native speedbrake maps
panels 1–2 to 31.5 degrees, not FBW's clean-wing 20-degree schedule. FBW's
flap-dependent gains, hydraulics and exact ground-spoiler trigger logic remain
unimplemented.
Since docs/SPEED_BRAKE.md the plugin writes FBW's speed brake / ground spoiler
order (flap-dependent gains, FBW ground-spoiler logic) into X-Plane's handle as the
mean panel order; these native outputs then follow it.

The local DataRefs description says positive spoiler deflection is trailing-edge
down. Direct offline evaluation of shipped Laminar 737 and A330 geometry instead
shows all 24 tested spoiler/speedbrake meshes rising at +10 degrees and lowering
at −10 degrees. This supports positive-open mapping; it is not a live dataref
recording. Native source files remain unchanged.

The old symmetric ratio mapping could extrapolate beyond the source clip.
`animation_sampling.py` now supports explicit endpoint guards: duplicate endpoint
poses at distinct input values flatten extrapolation below zero and above the
configured maximum. Degree inputs map onto a cropped 35/65 or 50/65 portion of
the authored clip. Other unclamped animation rules retain their previous mapping.

The actual exported OBJ files were independently matched to the raw source by
UV and neutral position across all 32 moving pieces and the fixed hardware.
All authored keys within each crop plus its exact endpoint were checked (23
samples for panels 1–2, 32 for panels 3–8). Maximum error is 13.7604 mm; neutral
matching error is below 0.023 mm. Every wing triangle keeps its UV association,
and the full 739,929-triangle neutral comparison changes no point by more than
0.024 mm. Explicit native-input tests from −1000 through +1000 confirm flat
limits; right-roll-only leaves left spoilers and both inboard groups stationary.
Flap deployment does not alter the spoiler transforms. These are geometry/input
tests, not proof that the simulator generates the desired outputs in flight.

## Evidence and rebuild

The external build root is `/Volumes/MacGames/A380X-XPlane12-build`:

- `reports/wings-controls-normalize.json`: one-based palette decoding audit.
- `reports/wings-controls-panels-candidate.json`: six panel assignments and export.
- `reports/wing-controls-source-approximation-audit.json`: all41 raw-source pose
  comparisons and main aileron trailing-edge checks, with source glTF SHA256.
- `animation-audit/audit_wing_control_approximation.py`: repeatable source audit.
- `reports/wing-controls-panels-export-vs-source.json`: actual exported flap
  paths versus all 41 authored source poses; `wing-controls-panels-neutral-parity.json`
  checks neutral geometry/UV correspondence across all 11 wing objects.
- `reports/wing-final-candidate-audit.json`: all 13,524 main-panel triangles now
  move; none of the formerly frozen 8,476 main-panel triangles remain static.
- `reports/wing-controls-neutral-preview.png`, `wing-full-flap-panels-preview.png`
  and `wing-right-roll-preview.png`: offline OBJ previews, not simulator screenshots.
- `reports/flap-panel-regressions.log` and `static-skin-pose-controls-regressions.log`:
  small Blender regressions.
- `reports/wings-spoilers-candidate.json`: whole spoiler skins, restored MFTF
  ownership and exported native datarefs.
- `reports/spoilers-export-vs-source.json`: actual exported spoiler paths versus
  weighted source poses, including fixed hardware and clamped endpoints.
- `reports/wing-spoilers-neutral-parity.json`: full neutral geometry/UV check.
- `reports/wing-spoiler-native-matrix-candidate.json`: native-group isolation,
  combined flap/spoiler inputs and under/over-range clamps.
- `reports/native-spoiler-sign.json`: unchanged Laminar 737/A330 sign evidence.
- `reports/wing-spoilers-deployed-preview.png`: offline geometry preview only.
- `reports/wing-spoilers-left-deployed-preview.png`: close front-side preview
  showing the raised panel faces (the rear overview obscures them).
- `reports/wing-spoilers-package-validation.json`: installed package, zero errors;
  existing missing-custom-FMOD-bank warning only. The ACF hash is unchanged.

The checked spoiler candidate is now installed through the SD-card package.
Its source export directory was promoted to `xplane-obj/wings-corrected`; the
previous complete flap/aileron export remains at
`xplane-obj/wings-before-spoiler-repair-20261004`. Candidate paths in audit reports
describe the pre-promotion location. Validation covered 143 pure-Python tests,
10 spoiler-panel Blender regressions, and re-runs of the five flap-panel and
five static-skin-pose Blender regressions. No native runtime test was attempted.

`tools/export_exterior_mac.sh` enables the separate source-scoped wing and tail
joint/panel repairs. The final wing group retains all 739,929 triangles.
`tools/audit_control_surface_motion.py` measures OBJ8 travel
at explicit dataref values; its results are diagnostics, not aerodynamic limits.

For a native recheck, start clean with failures explicitly off, inspect flaps
retracted/intermediate/full and roll left/neutral/right, including combined inputs.
Check spoilers retracted/intermediate/full, roll-only left and right, combined
roll/speedbrake, full flaps with speedbrakes, and armed touchdown. Verify the actual
loaded aircraft is fresh. Offline native-output tests do not establish correct
live ground-spoiler triggering or A380 flight-control behavior. Live wing flex and
Windows runtime remain untested.

## Wing flex (plugin side ready; needs a wing re-export)

FBW's eight wing-flex clips (`left|right_inboard_flex`, `_inner_midboard_flex`, `_outer_midboard_flex`,
`_outboard_flex`) were exported frozen at their zero-flex frame. The avionics plugin now runs a port of FBW's
wing flex model (`native/avionics/a380x_wing_flex.h`: 5 nodes per wing, spring-dampers, lift from weight x
load factor blended out by weight on wheels, fuel on the inner nodes) and publishes FBW's bend angles as
`a380x/xp12/wing/flex_<left|right>_<inboard|inboard_mid|outboard_mid|outboard>` (degrees).
`config/animation_datarefs.json` maps each clip to its angle with A380_WINGS.xml's `angle x gain + offset`
(0..100), so a wing re-export (`tools/export_exterior_mac.sh`) picks them up; `tools/test_wing_flex_mapping.py`
pins the ranges. The installed wing objects were not re-exported (the converter can only move single-bone faces, so about 58 % of the skin would stay frozen and tear, and the slat skin repair would be lost): `tools/wing_flex/wing_flex_obj.py` adds the bend to them as 13 rigid segments per wing cut along FBW's bone axes; see [WING_FLEX_OBJ.md](WING_FLEX_OBJ.md). Typical output: about -1.4 deg total
droop parked with full fuel, +1.6 deg in 1 g cruise, more under g.

## Engine pylon wobble (plugin side ready; needs the engine OBJ rotation)

`native/avionics/a380x_eng_wobble.h` and `.inc` publish:
- `a380x/xp12/eng/wobble_pitch_deg[4]`: + intake up.
- `a380x/xp12/eng/wobble_yaw_deg[4]`: + intake right.

Both are in degrees, for engines 1..4 from left to right. They give each engine's nod and sway on its pylon, relative to the wing.

**Model:** one lightly damped spring per axis and engine.
- Pitch is about 2.5 Hz and yaw about 1.8 Hz, with damping ratio 0.06.
- It is driven by the acceleration at the pylon attachment:
  - body load factor;
  - the wing flex node the pylon hangs from (inboard node 1, outboard node 2);
  - roll acceleration × span;
  - side load factor, for the sway.
- The drive is high-passed (τ 1 s), so steady loads leave the engines centred. A 0.4 g, 0.15 s bump nods the intake about 0.2° down, and it rings out in about 3 s (`test_eng_wobble.c`).

**Still to do, in the wing objects** (`tools/wing_flex/wing_flex_obj.py`):
- Each engine, nacelle core, fan and pylon group needs two rotations about that pylon's wing attachment point, nested inside the wing flex bone rotation the engine already rides on (`WING_FLEX_6` inboard, `WING_FLEX_11` outboard):
  - `ANIM_rotate_begin <lateral axis> a380x/xp12/eng/wobble_pitch_deg[i]`, keys -1.5 → -1.5 and 1.5 → 1.5;
  - `ANIM_rotate_begin <vertical axis> a380x/xp12/eng/wobble_yaw_deg[i]`, with the same keys.
- The signs must make + pitch raise the intake and + yaw point it right, in the OBJ frame (nose +z, left +x).
