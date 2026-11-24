# FLAPS lever, slat/flap control computers and flap/slat surfaces

The A380 FLAPS lever with its pull-out detents and gates, FlyByWire's slat/flap control computer logic
(CONF 1 / 1+F, automatic retraction, cruise baulk, alpha/speed lock, flap load relief), FBW's flap and
slat drives on FPPU (synchro) angles, and their indications. Native code: `native/avionics/a380x_flaps.h`
(XPLM-free core) and `a380x_flaps.inc` (X-Plane adapter), unity-included from `a380x_avionics.c` after
the thrust levers. Geometry, exterior bindings and the ACF schedule: `tools/flaps_lever_cockpit.py`,
installed by `tools/install_flaps_lever.py`. Nothing here has been flown in X-Plane yet.

## FlyByWire sources (pinned revision 2baa2b35 of `upstream-aircraft`)

| Behaviour | Source |
| --- | --- |
| SFCC configuration logic and thresholds, status / actual-position words | `fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/sfcc/mod.rs` (`SlatFlapControlComputer`, its tests) |
| Demanded FPPU angle per configuration | `sfcc/flaps_channel.rs`, `sfcc/slats_channel.rs` |
| FPPU -> surface angle tables, drive speeds, gear ratios, hydraulic sides (flaps green + yellow, slats green), 5250 psi | `a380_systems/src/hydraulic/mod.rs` (`FLAP/SLAT_FPPU_TO_SURFACE_ANGLE_*`, `FlapSlatAssembly::new`, `update_with_*`) |
| Drive motion: 0.3 s speed filter, pressure factor, 6.69 / 0.18 deg thresholds, 18 % low speed, 0.976 deceleration | `fbw-common/src/wasm/systems/systems/src/hydraulic/flap_slat.rs` (`FlapSlatAssembly`) |
| CSU handle index 0..4 (`FLAPS_HANDLE_INDEX`), previous/current detent | `fbw-common/.../hydraulic/command_sensor_unit.rs` |
| Aileron droop 5 deg when the flaps are not retracted (bit 19 = 0), 1 deg/s | `fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFctl.cpp` + `_data.cpp` (`Constant2_Value`, `BitfromLabel_bit_e`, `RateLimiterVariableTs2`) |
| VFE 263 / 222 / 220 / 196 / 182 kt | `fbw-a380x/src/systems/shared/src/PerformanceConstants.ts`, `OperatingSpeeds.tsx` |
| PFD slat/flap display (target text, pointer laws, target markers, F RELIEF, A LOCK) | `fbw-a380x/src/systems/instruments/src/PFD/LowerArea.tsx` (`SlatsFlapsDisplay`) |
| Lever clip (5 keys, 0 / 13.07 / 26.21 / 40.80 / 53.43 deg) and sleeve clip `LOCK_FLAPS` (7.8 mm) | `Part_Interior_Cockpit/model/A380_COCKPIT_LOD00.gltf`; behaviour `pedestal.xml` + `A32NX_Interior_Handling.xml` (`flaps_lever` = `FLAPS_HANDLE_PERCENT`, `lock_flaps` while the lever is held) |
| Exterior clips keyed on the FPPU ratio | `Part_Exterior_Wings/model/A380_WINGS.xml` (`[lr]_flap_percent_key` = `L:A32NX_*_FLAPS_ANIMATION_POSITION`) |

## The reference table against FBW

| Lever | Configuration | VFE (kt) | Slats | Flaps | FPPU flaps / slats | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 0 | VMO/MMO | 0 | 0 | 0 / 0 | |
| 1 (in flight) | 1 | 263 | 20 | 0 | 0 / 286.48 | slats only |
| 1 (ground, < 205 kt) | 1+F | 222 | 20 | 8 | 215.68 / 286.48 | aileron droop 5 deg |
| 2 | 2 | 220 | 20 | 17 | 259.28 / 286.48 | aileron droop 5 deg |
| (3, load relief) | 2S | 220 | 23 | 17 | 259.28 / 327.39 | only reached by relief at lever 3 |
| 3 | 3 | 196 | 23 | 26 | 297.59 / 327.39 | aileron droop 5 deg |
| FULL | FULL | 182 | 23 | 33 | 338.99 / 327.39 | aileron droop 5 deg |

The user's table matches FBW except two points where FBW wins: FULL flaps are exactly 33 deg, and FBW's
PRIM droops the ailerons 5 deg whenever the flaps are not retracted (1+F, 2, 2S, 3, FULL), not only in
1+F. FBW has a seventh configuration, 2S (slats 23 / flaps 17), used only by the load relief at lever 3.

## SFCC logic (FBW `generate_configuration`, arm for arm)

The CSU handle index is the last detent the lever was read in (the lever out of detent keeps it, as FBW's
discrete handle never leaves a detent), so `(previous, current)` pairs are FBW's.

- **Lever 1.** On the ground, or below 205 kt, CONF 1+F. Selected from 0 in flight at or above 205 kt:
  CONF 1 (slats only). Held at 1: above 212 kt CONF 1 (automatic retraction, `1+F -> 1`); below 205 kt
  1+F again (re-extension); 205..212 kt hysteresis keeps the configuration. From 2 / 3 / FULL to 1: 1+F at
  or below 212 kt, 1 above.
- **Cruise baulk.** 0 -> 1 above 265.5 kt or above 22,000 ft pressure altitude: CONF 0 is kept (status bit
  25, PFD "1"), until both conditions clear. Overspeeding CONF 1 is not baulked.
- **Alpha/speed lock.** Lever to 0 in flight below 155 kt or above 9.5 deg AoA: CONF 1+F is held (bit 24,
  PFD "A LOCK"); released above 161 kt **or** below 9.2 deg AoA (FBW's OR, see deviations).
- **Flap load relief.** FULL above 184.5 kt -> CONF 3, back to FULL below 179.5 kt; 3 above 198.5 kt -> 2S,
  back below 193.5 kt; 2 (selected from 1 or held) above 222.5 kt -> 1+F, back below 217.5 kt (bit 22,
  PFD "F RELIEF"). These are the "flaps go up automatically" functions, with re-extension.

FBW uses the simulator IAS for these thresholds and the ADR AoA; the port uses
`sim/cockpit2/gauges/indicators/airspeed_kts_pilot`, `sim/flightmodel/position/alpha` and
`sim/flightmodel2/position/pressure_altitude`.

## Surfaces (FBW FPPU drives)

The SFCC demands an FPPU angle per configuration; each drive moves its synchro at
`0.07 / 0.08 rad/s x 180/pi x 314.98/140` = 9.024 (flaps) / 10.313 (slats) FPPU deg/s at full pressure,
half per hydraulic side (flaps green + yellow, slats green), through FBW's pressure factor and 0.3 s speed
filter, at full speed outside 6.69 deg of the target, then decelerating to the 18 % low speed, stopping
within 0.18 deg. Surface angles come from FBW's FPPU tables; the first 122.2 deg of flap travel (36 %) is
Fowler translation at 0 deg of rotation, which is why the exterior must be driven by the FPPU ratio.

| Movement (full pressure) | FBW nominal | Core (tested) |
| --- | --- | --- |
| Slats 0 -> 20 deg | 27.8 s | 27.5..30.5 s |
| Flaps 0 -> 8 deg (1+F) | 23.9 s | 23.5..26.5 s |
| Flaps 8 -> 33 deg | 13.7 s | 13..16 s |
| Flaps full retraction | 37.6 s | 37.5..40.5 s |
| One hydraulic side | twice as long | 47.5..51 s for 0 -> 1+F |

X-Plane hydraulics: `hydraulic_pressure_1` = green, `_2` = yellow (as the FWS adapter), scaled from this
ACF's 3000 psi engine pumps to FBW's 5250 psi. No pressure: the surfaces stop (the SFCC still commands).

## Lever and detent model

The real lever is pulled out of its detent before it moves; gates at 1 and 3 stop a single action from
passing them; it never rests between positions. The port:

- **X-Plane mechanism.** The manipulator is X-Plane's lift-detent click-hold-drag, as Laminar's A330 flap
  lever: `ATTR_manip_drag_rotate` about the lever's own pivot and axis (0..53.43 deg = lever 0..4,
  `ATTR_manip_keyframe` at the authored 1 / 2 / 3 angles) with a lift of the sleeve travel (7.84 mm) on
  `lift_input` 0..1, and `ATTR_axis_detent_range` pits (height 0) at 0, 1, 2, 3, 4 joined by travel
  ranges of height 1. The pilot drags the handle outward (pulls it out), then along its arc; X-Plane
  itself refuses rotation in a pit and refuses to drop the handle between pits. This models the real
  pull-out directly, which the thrust levers (no lift on Airbus thrust levers) could not use.
- **Native enforcement** (`a380x_flaps.h`, also when X-Plane's own detent behaviour differs). A latched
  lever ignores rotation input; lift >= 0.5 takes it out of its detent and starts an action with the gate
  window `0 -> [0,1]`, `1 -> [0,3]`, `2 -> [1,3]`, `3 -> [1,FULL]`, `FULL -> [3,FULL]`; the hand is
  clamped to the window (the gate). Dropping the lift within 0.25 positions of a position seats it there
  (to pass a gate: drop into it and pull again, also within one mouse hold); releasing the mouse (no
  manipulator write for 0.4 s) springs it into the nearest position of the window. The CSU reads a
  detent within 0.08 positions (about 1 deg). The input datarefs report the hand / lift while dragging
  and the lever otherwise, so the next click starts where the lever is shown.
- **Animation.** `lever_anim` follows the hand while dragged and travels at 6 positions/s after a command,
  hardware step or spring; `lock_anim` (the sleeve) is out while the lever is out of its detent (0.2 s).
  No sound (the only plugin sound remains the thrust detent click).
- **Commands.** `a380x/xp12/flaps/lever_down` / `lever_up` move one position (a complete action, so the
  gates are respected). X-Plane's `sim/flight_controls/flaps_down` / `flaps_up` (one position),
  `flaps_down_full` / `flaps_up_full` (FULL / 0) and `flaps_detent_1..8` (position 1..4, 5..8 = FULL) are
  intercepted and consumed while the plugin runs, so keyboard and joystick buttons step the five lever
  positions and X-Plane's own seven-detent handle never moves under them.
- **Hardware axis.** A change of `sim/cockpit2/controls/flap_handle_request_ratio` that the plugin did not
  write (flap axis, another plugin, the flyPad "flaps up" preset) is adopted as lever position
  `ratio x 4` (0.6 positions of hysteresis against axis noise) and the configuration is written back.

## X-Plane flight model

The plugin writes the SFCC configuration into X-Plane's own flap handle: `flap_handle_request_ratio =
FlapsConf / 6` (0 CONF 0, 1/6 CONF 1, 2/6 1+F, 3/6 2, 4/6 2S, 5/6 3, 1 FULL), only when it changes. The
ACF's seven detents are those configurations, so X-Plane's flight model flies FBW's slat and flap angles
(`xp_layout_ok` = 1). With an unpatched ACF (`xp_layout_ok` 0) the handle position whose interpolated
flap angle is the configuration's angle is written instead (CONF 1: the first detent).

ACF changes (`tools/flaps_lever_cockpit.py` `ACF_CHANGES`, both `output/A380X-XP12/A380X-XP12.acf` and
`acf/A380X-XP12-current.acf`, which stay byte-identical):

| Key | Before | After | Why |
| --- | --- | --- | --- |
| `acf/_flap_detents` | 6 | 6 (unchanged) | seven configurations |
| `acf/_flap1_dn/0..6`, `acf/_flap2_dn/0..6` | 0, 1, 5, 10, 20, 25, 40 | 0, 0, 8, 17, 17, 26, 33 | FBW surface angles per FlapsConf (the wings use flap set 2; set 1 kept identical) |
| `acf/_slat1_dn/0..6` | 0, 1, 1, 1, 1, 1, 1 | 0, .8696, .8696, .8696, 1, 1, 1 | slats 0 / 20 / 23 deg (ratios of 23) |
| `acf/_slat2_dn/0..6` | 0, 0, 1, 1, 1, 1, 1 | same as slat 1 | all slats move together (FBW) |
| `acf/_slat1_dn_max_deg`, `acf/_slat2_dn_max_deg` | 10 | 23 | A380 slats 23 deg |
| `acf/_flap_ext_time`, `acf/_flap_ret_time` | 45 | 37.567 | FBW full flap travel at full pressure |
| `acf/_slat_ext1_time`, `_ext2_`, `_ret1_`, `_ret2_` | 0.1 | 31.747 | FBW full slat travel |
| `acf/_slat_with_stall_EQ` | 1 | 0 | no X-Plane auto-slats (the SFCC owns slat logic) |
| `acf/_flap_extend_protect` | 1 | 0 | X-Plane must not veto SFCC configurations (relief, 2S) |
| `acf/_flap_retract_protect` | 0 | 0 (unchanged) | |
| `acf/_Vfe1_kts`, `acf/_Vfem_kts` | 250, 180 | 263, 182 | FBW VfeF1 / VfeFF for X-Plane's own overspeed reference |

The flap/slat aerodynamic coefficients (`_flap*_cl/cd/cm`, `_slat*_inc`, wing assignments) are unchanged.

2026-11-24: the A388 donor flight model (docs/A388_FLIGHTMODEL_ADAPTATION.md, 14 Nov) brought its own flap
tables (`_flap_detents` 4, 45 s flaps, 7 s slats), so `xp_layout_ok` is 0 and the fallback request above is
used. Only the travel times were set back to FBW's (37.567 s flaps, 31.747 s slats, the `ACF_CHANGES` values);
the donor's detents, angles and coefficients stay. Re-running `adapt_a388_flightmodel.py` copies the
donor's `flap*` / `slat*` families again and undoes this.
X-Plane moves its surfaces with its own timing toward the commanded configuration; the visible surfaces and
every indication follow the FBW drives instead (see the decisions below).

## Cockpit geometry and exterior animations

| OBJ | Change |
| --- | --- |
| `objects/interior/cockpit/cockpit_a380_cockpit_pedestal_misc01_4k.obj` | The Blender export already animates `LEVER_FLAPS` (TRIS 22722/1026), `LEVER_FLAPS_LOCK_Cylinder.001` (23748/312) and `LOCK_FLAPS` (100464/3816) on X-Plane's flap handle, pivot-local vertices posed at clip angle 26.17 deg. That block is re-bound to `a380x/xp12/flaps/lever_anim` (keys 0..4, same five angles) and `LOCK_FLAPS` gets the nested sleeve translation on `lock_anim` (the clip vector rotated into the exported pose, 7.84 mm). Every range is identified against the glTF (1e-5 m); VT/IDX/UV and drawn ranges unchanged; the thrust-lever wraps untouched. Marker `# A380X FLAPS LEVER V1 MOTION LOCK_FLAPS`. |
| `objects/interior/cockpit-controls/a380x_cockpit_controls.obj` | Appended after `# A380X FLAPS LEVER V1`: one draw-disabled box around the handle (`LEVER_FLAPS` + `LOCK_FLAPS`), animated with the lever, carrying the lift-detent manipulator above. |
| `objects/exterior/wings/a380x_{pylons,wing1,wing1_2,wing2,wing2_2,wingfence}.obj` | Only animation-header tokens: `sim/flightmodel2/controls/flap1_deploy_ratio` -> `a380x/xp12/flaps/flap_anim_ratio` (110 per object), `slat1_deploy_ratio` -> `slat_anim_ratio` (104). Keyframes (FBW FPPU ratio 0..1) unchanged. |

`config/animation_datarefs.json` now maps `flaps_lever` to `lever_anim` (0..4), `lock_flaps` to
`lock_anim`, and the exterior flap/slat clips to the FPPU ratios, so a future export binds the same
datarefs. Thirteen other cockpit OBJs contain empty per-material copies of the exported lever block (no
TRIS) on X-Plane's flap handle; they draw nothing and are left alone.

## Displays, warnings and other consumers

- **PFD.** The cockpit PFD is FBW's own PFD (xp-kit); the plugin now sends `L:A32NX_SFCC_1/2_
  SLAT_FLAP_SYSTEM_STATUS_WORD`, `_SLAT_FLAP_ACTUAL_POSITION_WORD`, `_SLAT_ACTUAL_POSITION_WORD`,
  `_FLAP_ACTUAL_POSITION_WORD` and `L:A32NX_FLAPS_HANDLE_INDEX`, `FLAPS_CONF_INDEX`, `*_SLATS_ANGLE`,
  `*_FLAPS_ANGLE`, so its lower area shows 0 / 1 / 1+F / 2 / 3 / FULL, the S/F pointers on FPPU angles,
  cyan targets while moving, F RELIEF and A LOCK. The native fallback PFD (`a380x_pfd_configuration.inc`)
  draws the same from the same data (`pfd_guidance.sfcc_valid`). VMAX uses the more restrictive of the
  commanded configuration's VFE (2S at the flaps-17 limit) and the actual surfaces (enclosing position
  while moving); VFE next follows the displayed configuration (FBW OperatingSpeeds).
- **EWD.** The A380 has no slat/flap indication on the EWD (it is on the PFD); nothing to change.
- **FWS.** `flaps_handle_index` (CSU), `flaps_conf_index` (FlapsConf) and the surface angles come from the
  module (T.O CONFIG, FLAPS LEVER NOT ZERO, landing memo, flap/slat alerts).
- **FMS speeds** take the lever from the PFD snapshot; the center-right gear-not-down lamp uses the lever
  (>= 2); flyPad presets still step the lever with `flaps_down` and reset it through the handle.

## Datarefs and commands (`a380x/xp12/flaps/`)

Writable: `lever_input` (0..4, manipulator), `lift_input` (0..1, manipulator lift). Read-only integers:
`handle_index`, `lever_latched`, `conf_index` (FBW FlapsConf), `config_display` (0 "0" .. 5 "FULL"),
`flap_relief`, `alpha_lock`, `cruise_baulk`, `xp_layout_ok`. Read-only floats: `lever_anim`, `lock_anim`,
`flap_anim_ratio`, `slat_anim_ratio`, `flap_angle_deg`, `slat_angle_deg`, `flap_fppu_deg`,
`slat_fppu_deg`, `flap_target_fppu_deg`, `slat_target_fppu_deg`, `aileron_droop_deg`, `vfe_knots`,
`vfe_next_knots`. Commands: `lever_down`, `lever_up`. The list is `FLAPS_COMMANDS` / `FLAPS_DATAREFS` in
`tools/avionics_provider.py`; `build.py` fails when the native registry differs.

## Tests, build and installation

`native/avionics/build.py` runs `test_flaps.c` (166 checks: pull-out, gates, spring, no intermediate
positions, commands and hardware hysteresis, SFCC 1 / 1+F / ARS / baulk / alpha lock / relief incl. FBW's
own unit-test scenarios, out-of-detent handle hold, drive timing and hydraulics, words, PFD targets, VFE,
droop, X-Plane mapping), its MinGW `-Werror` compile check, and `test_flaps_adapter.c` (mock XPLM:
registry, first sync, lift-detent drags with gates, commands and the twelve intercepted stock commands,
hardware adoption without fighting, CONF 1 / relief / ARS into X-Plane's handle, FPPU surfaces with and
without pressure, pause, replay, disable, reload, legacy ACF, shutdown). The raw inputs are checked
against `DataRefs.txt` and the intercepted commands against `Commands.txt`; results are in
`build-report.json` (`flaps_lever`). `test_pfd_render.c` covers the native SFCC display.
`tools/test_flaps_lever_cockpit.py` covers the ACF patch, the exterior rebind, the click target, the real
pedestal rebind and the installer.

Install with X-Plane closed, in this order: geometry + ACF (`tools/install_flaps_lever.py --aircraft
output/A380X-XP12 --geometry-stage <stage>`; backs up to `build/backups/flaps-lever-<timestamp>`), then the
plugin (`tools/install_xp_kit_cockpit.py <build>`; it writes the new commands and datarefs into the
provider manifest, which the new animation datarefs need), then `tools/validate_aircraft.py`.

## Unverified in the simulator

- X-Plane's lift-detent feel: the lift direction for this lever (dragging outward along the lever is
  expected, as on the Laminar A330), the 7.8 mm lift distance, whether X-Plane keeps the lift on release
  between pits (the plugin then springs the lever after 0.4 s) and whether it writes during a still hold.
- X-Plane's flap/slat motion with the new detent table and times, and that it accepts every written
  handle value (extension protection is off); the flight-model effect of 33 deg FULL and 23 deg slats.
- That a hardware flap axis writes the handle only when moved (an axis written every frame would hold the
  lever where the axis is, which is acceptable, but X-Plane may fly that frame's raw axis value).
- The visible surfaces at 1+F / 2 / 3 / FULL (the exterior clips are FBW's, now on FBW's FPPU ratio).
- The xp-kit PFD's reaction to the new SFCC words (the bridge passes `words` and `numbers` through).

## Deviations from FBW and decisions for the user

- **Alpha/speed lock release is FBW's OR** (IAS > 161 kt or AoA < 9.2 deg): at a normal AoA the speed lock
  holds one cycle. Airbus logic needs both (AoA low and speed high). Kept as FBW; switch to AND if wanted.
- **Two timings.** X-Plane's flight model flies its surfaces toward the commanded configuration with its
  own ACF timing (37.6 / 31.7 s full travel, linear), while the visible surfaces, the PFD and the FWS follow
  FBW's FPPU drives (translation before rotation, per-segment rates, hydraulics). Both end at the same
  angles; in transit they can differ by several seconds (e.g. 0 -> 1+F: about 13 s in X-Plane, 24 s FBW).
  Without hydraulic pressure only the FBW surfaces stop.
- Settled drives snap to the demand inside FBW's 0.18 deg target band; the 0.976 deceleration is
  referenced to 30 Hz; the low-speed drive never points away from the target.
- Aileron droop is computed (`aileron_droop_deg`) but not applied: the X-Plane ailerons and the FlyWithLua
  aileron visual driver are unchanged (FBW droops 5 deg in every flap configuration).
- Commands and hardware are complete actions without gates (the hardware lever has its own); `flaps_up_full`
  / `flaps_down_full` jump to 0 / FULL.
- X-Plane's own flap overspeed reference interpolates between 263 and 182 kt over its detents, so its
  optional overspeed warning can disagree with FBW's VFE in 2S / relief; the FWS and PFD use FBW's.
- `tools/pfd_runtime_check.py` and `tools/sound_port_map.py` still name X-Plane's flap datarefs, which now
  carry the configuration (handle) and X-Plane's own surface timing.
