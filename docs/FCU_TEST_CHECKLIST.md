# FCU installation and verification checklist

This checklist accompanies [FCU_FUNCTIONAL_SPEC.md](FCU_FUNCTIONAL_SPEC.md). Record the candidate build hash, aircraft ACF hash, X-Plane version, platform, graphics settings, camera/FOV, flight state and input method with each live result. The supplied training extracts are the primary reference for the supported AP/FD/A/THR principles; missing ALT/HDG/V/S manufacturer pages remain a documented verification gap.

## Evidence status

| Check | Evidence available | Scope |
|---|---|---|
| Guidance regressions | `build/fcu-manual-audit-20261020/guidance-tests.txt` | Strict C build and imported-law bench tests passed, including new AP/FD/A/THR cases and altitude sensor replays. |
| SDK input/authority catalog | `build/fcu-runtime-audit-20261020/runtime-audit.md`, `source-catalog-audit.json` | Source/catalog inspection; final SDK adapter tests and live authority checks must be attached separately. |
| Panel / geometry / registry | `build/fcu-manual-audit-20261020/review-python-tests.txt` | 47 focused panel, animation, provider and control tests passed at review. Retain the final candidate's outputs and geometry patch report as well. |
| Mechanical travel | `native/avionics/test_fcu_motion.c` | Pure C motion checks; verify the installed candidate separately. |
| Rendering / texture | `tools/test_fcu_panel.py`, renderer/build previews | Source or isolated previews are not cockpit screenshots. |
| Installation, cockpit screenshots and FPS | Record final candidate's installation report and live-run artifacts here. | **Pending until actual X-Plane evidence is saved.** Do not copy ND/helper measurements into FCU flight-performance claims. |

Final offline status, 20 October 2026: candidate `build/fcu-overhaul-20261020/native-v3` is installed with backup `build/backups/fcu-overhaul-20261020-121141`. The complete native build passed; the final focused Python run passed **77 tests** (`python-tests-final.log`). Static validation passed with zero errors (`aircraft-validation-final.json`). Installation/hashes are recorded in `installation.json`; all native build source hashes still match the installed candidate. The supplied ACF and improved ND assets are preserved. The old static report `aircraft-validation.json` contains two validator assumptions corrected by the new saved-ACF regression tests: lexical property order mistaken for numeric attachment order, and reliance on a stripped comment instead of the empty hidden native panel group.

The user explicitly requested that X-Plane stay closed during the final continuation. No live cockpit click, smooth-mouse, flight, night-lighting, before/after screenshot or FPS result is claimed. The `fcu-before-matched.png` / `fcu-after-matched.png` files are offline renderer previews only. Live checklist items below remain unchecked deliberately.

Checked items below must identify real evidence. A screenshot made by a test renderer or SVG export must be labelled as such. Do not fabricate before/after screenshots, FPS or completed flight tests.

## Build and install

- [x] Preserve the current user ACF and installed ND. Hashes before/after are in `installation.json`.
- [x] Back up plugin, atlas, controls, retained knob/decal OBJs and affected materials. The first factory INI was added; rollback manifest records that no previous INI existed.
- [x] Build the complete candidate with tests enabled and byte-verified installed display assets; external `TMPDIR` used.
- [x] Run and retain the focused Python suite; 77 final checks passed.
- [x] Retain passing native guidance, FCU adapter, motion, lighting and PFD/FCU tests in `native-v3/build-report.json`.
- [x] Verify retained geometry/UV/texture preservation and unrelated controls in `geometry/staging-report.json`.
- [x] Verify both shared FCU glass quads (SPD/HDG and ALT/V/S) match the 1024×128 atlas and all existing display/touch devices remain mapped; final static aircraft validation passed.
- [x] Validate the signed native candidate and reviewed assets in `install-preflight.json`.
- [x] Install the combined native/geometry/material/panel set while X-Plane is closed with `tools/install_fcu_overhaul.py`; backup/install report saved.
- [ ] Put the reviewed `fcu-calibration.ini` beside the **active** ACF. Verify four pilot-handle values and actual hardware detents; a successful INI parse does not prove physical calibration.
- [x] Confirm installed plugin/asset hashes, current ACF preservation and unchanged ND asset hashes.
- [ ] Start X-Plane only on a later user-authorized test run and verify a single aircraft-local plugin/helper set, no missing datarefs and no duplicate autopilot provider.

## Every physical control

Run this once with the 3D mouse controls and again with configured hardware bindings. API command injection verifies command routing; it does not prove that the physical cockpit click surface is reachable.

| Control | Interaction to perform | Expected result |
|---|---|---|
| Each of the 11 buttons | Short click, hold, release, rapid separated clicks; drag pointer off before release. | Visible independent travel and return; one logical toggle per Begin; no repeated toggle while held; no stuck depression or accidental neighbor activation. |
| Each of the four knobs | Rotate both directions at outer edge; use upper-center push and lower-center pull. | Correct value/direction and axial sign, aligned hit region during motion, independent rotation/axial animation. |
| ALT 100/1000 selector | Toggle both positions, then rotate ALT. | Correct visible selector and increments; selected altitude changes by 100 or 1000 ft. |
| V/S/FPA push | Push in V/S and FPA. | No mode change in the retained A380 adapter; tooltip must not promise level-off. |
| Disconnect buttons/bindings | Click both outer thrust-handle disconnect buttons; use AP and A/THR hardware disconnect. | Shared authoritative disconnect, immediate logical/FMA change and release of the relevant actuator leases. |
| EFIS LS | Operate captain and F/O separately. | Correct side display indication; no unintended AP/approach engagement. |

Observe physical motion through `fcu/*_pressed`, `*_knob_rotation`, `*_knob_axial` and `altitude_selector`; observe logical state through engagement/selection outputs. Pressing an inhibited AP button must still animate without falsely lighting its engagement bar.

## Ground preparation and lighting

- [ ] Power-up selects both FDs ON, with no initial takeoff guidance bars. FD OFF/ON affects both channels; cycling electrical power restores ON.
- [ ] AP1/AP2 reject engagement on the ground and show no false AP engagement indication. Press/release still works.
- [ ] At stable ground IDLE, A/THR push can select armed standby. Active regulation remains off.
- [ ] TOGA entry automatically arms A/THR; MCT/FLEX entry does so only with validated FMS FLEX. MCT without FLEX must not falsely arm a takeoff mode.
- [ ] Select A/THR OFF while keeping the handles at the takeoff detent. Confirm it stays off until a fresh eligible detent entry or explicit arm command.
- [ ] With an invalid/missing INI, duplicate/unknown keys, bad ordering or a nonfinite handle sample, confirm automatic detents and normal active regulation are unavailable while unrelated FCU controls work.
- [ ] Sweep LCD, integral legends and each flood channel independently. Check daylight, night, zero brightness, partial brightness and unpowered states. LCD zero blanks the glass; engaged lamps follow their separate `fcu/*_light` outputs, retaining the specified 0.25 minimum at integral zero while powered. Power loss makes both glass and engagement lamps dark.
- [ ] A/THR bar is visible while armed, including ground TOGA; its physical depression is independent of the lamp. Check AP1/AP2, FD, LOC, ALT and APPR against their actual logical state.

## AP, selected guidance and sensor independence

Use a repeatable saved airborne test state. Retain mode traces, actual control outputs and flight data; a synthetic sensor replay alone cannot establish aircraft handling quality.

- [ ] Check AP rejection just before five seconds airborne and acceptance at/after five seconds with valid inputs. Pause must not advance the gate.
- [ ] Outside approach, engage AP1 then AP2, and reverse the order. Only the latest selected channel remains engaged. Fault the requested channel and confirm a healthy opposite channel cannot authorize the failed one.
- [ ] With approach selected and valid front-course ILS, check dual AP selection, LOC*/LOC and GS*/GS. Loss of valid approach data must produce the supported reversion/disconnection. Do not infer LAND/FLARE/ROLLOUT or autoland certification.
- [ ] From stabilized 10,000 ft, select 12,000 ft and pull ALT; verify OP CLB, appropriate thrust indication, ALT*, ALT and level capture. Repeat to 8,000 ft with OP DES. Record overshoot, settling, IAS, bank, pitch and throttle traces from X-Plane.
- [ ] Repeat a selected V/S climb/descent and TRK/FPA conversion. Change ALT after capture without pulling/pushing; it must update the window without unexpectedly leaving the existing hold.
- [ ] Turn and pull HDG/TRK through north; confirm direction, wraparound, reference conversion and the actual flight response. Test TRUE/MAG and SPD/MACH without an unintended mode toggle.
- [ ] Set a selected speed below VLS and above VMAX. FCU retains the crew's selection; PFD effective target respects valid limits. Test managed speed with a healthy provider and verify an unavailable request is clearly rejected without one.
- [ ] With A/THR active and healthy ADR speed, invalidate IR heading/navigation/VSI. AP/FD must respond to their sensor loss while A/THR speed regulation stays independent. Loss of ADR speed validity must stop active A/THR control.

## A/THR and authority checks

- [ ] Transition TOGA/FLEX armed to airborne CL active. Confirm no physical pilot-handle or sidestick position changes are written by guidance.
- [ ] At a lower active lever setting, verify commanded engine output stays within the configured proportional ceiling. Repeat with asymmetric handles and confirm an IDLE handle cannot receive another engine's CL command. Qualified alpha-floor/TOGA LK is the intentional handle-ceiling exception. Report the 0.88 CL command as a port approximation, not measured N1 or a complete FADEC limit.
- [ ] Move all handles into IDLE and select both inner reversers in a controlled ground test. Confirm normal A/THR disconnection. One inner reverse alone must not be mistaken for the both-inner criterion.
- [ ] Without a qualified FE provider, confirm raw AoA/PFD envelope values cannot trigger alpha floor. With a controlled qualified provider, verify A FLOOR→TOGA LK and explicit disconnect suppression until the request clears. Clearly label provider injection as a diagnostic test.
- [ ] Test supported stock AP1/AP2/A/THR hardware on/toggle/off commands. Engagement must enter the custom FCU path; stock off/disconnect must remain effective. Unsupported AP3/retard must not create an alternative controller.
- [ ] In a controlled conflict test, activate a foreign stock servo/A/THR state or foreign override. Verify custom engagement drops and only this plugin's leases are released. Stock feedback must never become guidance targets.
- [ ] Verify pause/replay, power loss, plugin disable, aircraft reload and unload release owned pitch/roll/throttle overrides. Check that no control is left latched and unrelated plugins' overrides are preserved.

## Real screenshots, FPS and regression record

- [ ] Capture real before/after FCU screenshots at the same pilot-seat camera/FOV, display settings, lighting and aircraft state. Include a close view and a normal cockpit view; repeat at night. Record paths and the corresponding candidate hashes.
- [ ] Compare labels, signs, decimals, segment edges, dots, dashes, control alignment, metal/plastic roughness and glass readability. Check oblique viewing, supported monitor scales and both pilot seats. No shimmer, double legends, clipped digits or moving hit-surface gaps.
- [ ] Measure actual X-Plane frame periods in matched idle, cursor-sweep, knob-rotation and repeated-click phases. Report duration, sample count, mean/median/p95 frame time and reciprocal FPS with method and input source. Separate physical OS mouse tests from API-injected coordinates or commands.
- [ ] Confirm cursor motion does not rebuild airport geometry or display textures unnecessarily; panning/zoom and page changes remain responsive. Recheck ND 0.05/0.1 NM close maps, stand/taxiway labels and original map-panel buttons.
- [ ] Confirm PFD/FMA, MFD FCU backup, ND, ECAM, ADIRS buttons and current aircraft flight model still work. Attach errors, missing assets/datarefs, unresolved tests and restoration details instead of marking an incomplete run passed.

Final acceptance requires real installation and cockpit verification in addition to passing bench checks. Unsupported SRS takeoff, the FMS go-around phase, real LAND 3 monitoring redundancy, full LNAV/VNAV, complete FE/FADEC and full redundant computer behavior remain listed in the functional specification.

## Autoland, RETARD and go-around (AUTOLAND.md)

Watch `a380x/xp12/fcu/{land_active,flare_active,rollout_active,retard_active,go_around_active,land_capability,yaw_servo_active}` with the FMA.

- [ ] On the ground, confirm the radio altimeter reads about 0 ft with the main gear on the runway (FLARE and ROLL OUT are RA-based).
- [ ] Calm ILS with AP1+AP2 and A/THR: LAND near 400 ft, FLARE about 40-55 ft, FMA IDLE (RETARD) at 50 ft, RETARD aural at 10 ft, touchdown sink and point, ROLL OUT, the AP holds the centreline. Set IDLE: A/THR disconnects. Disconnect the AP at taxi speed: rudder lease released, no AUTOLAND light.
- [ ] Repeat with a 10-15 kt crosswind: decrab below 30 ft (heading toward the runway course), bank at touchdown, roll-out deviation. Tune ALIGN_KP/KI/KR and the ROLL OUT pedal scale in `a380x_autoland.inc` if needed; check the sign of `sim/flightmodel/position/beta` (nose right of the path should be positive).
- [ ] Leave the levers at CL after touchdown: A/THR stays at idle (FMA IDLE) until IDLE or reverse is selected.
- [ ] Pause, replay, AP disconnect and physical rudder-pedal input during ROLL OUT each release `override_joystick_heading`.
- [ ] TOGA at 100-200 ft: SRS / GA TRK, MAN TOGA, AP stays engaged, height loss; levers to CL: THR CLB; ALT* at the FCU altitude.
- [ ] FD-only approach: LAND/FLARE FD guidance, no RETARD, no yaw lease; manual RETARD reminder at 20 ft.
- [ ] Landing capability: APPR armed with AP1 only and A/THR shows LAND 3 SINGLE, AP1+AP2 LAND 3 DUAL. Above 200 ft, disconnect AP2, fail NAV 2, the copilot radio altimeter or hydraulic system 1/2: downgrade after about 1 s with a triple click. Below 200 ft in LAND: no change and no click. Check that `sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1/2` read at least 900 in flight. Check `NO DH` / `RADIO` / `BARO` in D3 for each PERF APPR minimum.


## Shared LCD follow-up

Latest candidate/evidence: `build/fcu-shared-displays-20261020/`. Native build checks pass, and the focused Python suite passes 85 tests including the two-face migration, cockpit controls, provider contracts and aircraft-validator regressions. The offline PNG is a production-renderer preview, not an X-Plane screenshot.

- [x] Generate two continuous cockpit display surfaces and preserve unrelated mesh/input payloads.
- [x] Keep SPD/HDG and ALT/V/S fields independent under the shared purple backgrounds.
- [x] Verify all existing command, motion, lighting, power and native rendering checks.
- [ ] At the next authorized simulator session, inspect both windows at normal and close cockpit views, operate each rotary/push/pull and confirm nighttime dimming and mouse performance.
