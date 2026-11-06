# Autoland: LAND / FLARE / ROLL OUT, A/THR RETARD and go-around

The native guidance (`native/avionics/a380x_guidance.c`) now continues an ILS
approach below 400 ft RA into LAND, FLARE and ROLL OUT, retards the A/THR in an
autoland, and supports a TOGA go-around into SRS GA / GA TRK. The code is
`a380x_autoland.h` / `a380x_autoland.inc` (unity include of the guidance core),
the law selection in `a380x_fbw_laws.cpp`, the yaw lease in
`a380x_fcu_adapter.inc` and the localizer-distance lookup in
`a380x_autoland_adapter.inc`.

**Status: offline only.** Unit tests and a closed-loop bench on a simplified
airframe pass. X-Plane was not run. The X-Plane A380 flight model, its rudder
and nosewheel authority and the port's yoke/pedal servo gains need in-sim
tuning before this can be relied on (see the last section).

## What was ported, from where

| Part | Source | Form |
| --- | --- | --- |
| LAND / FLARE / ROLL OUT / ALIGN / RETARD / GA transitions and thresholds | FlyByWire `fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFg.cpp` + `A380PrimComputerFg_data.cpp`, revision `2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2` (GPL-3.0) | Hand port; each constant names its Simulink parameter |
| Lateral law LOC TRACK (incl. its align beta command) and ROLL OUT; vertical law FLARE (incl. touchdown derotation) and SRS | Vendored, **unmodified** `A380FgOuterLoops` (`vendor/fbw`) | Selected via `lateral_law` 5/6, `vertical_law` 7/8 |
| PFD FMA bits | `PrimFgPublisher.ts`, `FcdcPublisher.ts`, `FMA.tsx` (same revision) | Bit layout only |
| Flare height, RETARD reminder | Airbus Safety First, "A focus on the landing flare": flare around 40 ft for the A380 (manual), the 20 ft "RETARD" callout is a reminder, levers at idle by touchdown | Cross-check |
| RETARD aural | FBW A380 `LegacyGpws.ts`: 20 ft without AP, 10 ft with AP | Matches the existing `a380x_aurals` thresholds; unchanged |

Not ported: the three-PRIM redundancy and monitoring, the A380 lateral normal
law (rudder/sideslip loop), BSCU nosewheel steering, FMS go-around phase,
acceleration altitude and thrust-reduction altitude, the LAND 3 capability
logic.

## Mode sequence

| Mode | Engages | Ends |
| --- | --- | --- |
| LAND (FMA `LAND`, B1+C1) | APPR selected, LOC TRK and G/S TRK, AP or FD on, valid RA below 400 ft for 1.2 s (`CompareToConstant_const_b`, `ConfirmNode1_timeDelay_ll`) | Go-around; on the ground after 10 s with no AP and APPR pushed or both FD off (`ConfirmNode2_timeDelay_i`); power loss |
| FLARE (`FLARE`) | LAND and the FgOuterLoops flare condition: below 80 ft when 14 x RA <= abs(H dot) (about 50 ft at 700 fpm), or at 42 ft; latched after 0.36 s | With LAND |
| ALIGN (decrab, no FMA word) | LAND below 30 ft RA (`CompareToConstant1_const_c`) | ROLL OUT |
| ROLL OUT (`ROLL OUT`) | LAND, on ground below 50 ft RA for 0.36 s | With LAND (the pilot disconnects the AP) |
| RETARD (A/THR FMA `IDLE`) | A/THR active, LAND, AP engaged, RA <= 50 ft (`CompareToConstant_const_d4`) | A/THR off (levers to IDLE, reverse, disconnect), GA, alpha floor, LAND reset |
| SRS GA + GA TRK | Rising edge of >= 2 levers at TLA >= 36.7 deg, flap lever >= 1, not 30 s on ground, guidance valid; airborne or in LAND | Another vertical and lateral mode selected |
| AP disengage on ground | Any lever at TLA >= 36.7 deg or a go-around while on the ground (FBW `APEngagedLogic` groundGa); the FD keeps SRS/GA TRK | - |

In LAND the LOC and APPR pushbuttons, HDG/ALT/V/S push/pull and ALT are
ineffective, as in the FBW PRIM. A lost localizer or glideslope does not revert
the modes (the memorized course is kept, the AUTOLAND light flashes); a lost
radio altimeter before FLARE disconnects the AP. Without LAND engaged (no APPR,
no RA, LOC/G/S not in track) the previous rule stays: the AP disconnects below
100 ft in G/S.

The FBW PRIM uses the same FLARE vertical law on the ground for the
derotation; the port bounds the yoke to -0.3..+0.15 in ROLL OUT so the nose
gear is held down without full forward elevator.

## FMA and lights

FBW PFD (xp-kit): word 1 bit 23 LAND and bit 27 LAND 2, word 2 bit 28 LAND
armed (APPR before LAND), word 3 bit 24 FLARE, word 4 bits 25 ALIGN / 26 ROLL
OUT, the G/S and LOC bits cleared in LAND, word 3 bit 16 PITCH GA (SRS),
ATS word bit 19 RETARD, A/THR FMA `IDLE`, and `L:A32NX_FCDC_{1,2}_FG_DISCRETE_WORD_1`
bits 18 LAND / 19 GA / 24 LAND 2 for the D1D2 capability cell. Native PFD:
`LAND`, `FLARE`, `ROLL OUT` centred over B1/C1 (border hidden), `SRS`,
`GA TRK`, `THR IDLE`, and `LAND 2` / `APPR 1` in D1/D2.

Capability: `LAND 2` when APPR or LAND is active, both ILS receivers are valid
on the same course, the RA is valid, attitude/air data are valid and an AP is
engaged; otherwise `APPR 1`. `LAND 3 SINGLE/DUAL` are never annunciated: this is
one guidance computer with one radio-altimeter source, without the PRIM
monitoring that makes LAND 3 fail-passive or fail-operational.

AUTOLAND light (`a380x/xp12/glareshield/land_mode`, now read-only): LAND with an
AP engaged in it, airborne, until ROLL OUT. The existing causes apply below
200 ft: both AP off, LOC > 1/4 dot above 15 ft, G/S > 1 dot above 100 ft,
signal loss, RA disagreement > 15 ft.

## A/THR on landing

Autoland: RETARD at 50 ft, the engine command ramps to idle at 0.12/s (the
FBW `RETARD_Value` is -10 N1 %/s; 0.12/s is the existing slew of the port's
normalized command). RETARD stays active through touchdown and the roll-out,
holding idle even if the levers are still at CL, until the crew sets IDLE
(A/THR disconnects, existing logic) or selects reverse on the inner engines
(disconnects). The RETARD aural (10 ft with AP) prompts that action.

Manual landing: the FBW PRIM has no RETARD mode without AP. A/THR keeps SPEED;
the 20 ft aural reminds the crew to set IDLE, which disconnects A/THR. Reverse
is always the crew's.

## Go-around

TOGA (see table) engages SRS GA and GA TRK, clears LAND/FLARE/RETARD and APPR,
arms A/THR if it was off (MAN TOGA while the levers are in TOGA) and restarts
the A/THR regulator from lever thrust. The SRS target is the higher of the
current speed target and the IAS at engagement, limited to VLS + 25 kt. GA TRK
holds the track at engagement. With the levers back at CL, A/THR is active in
`THR CLB` (`THR LVR` below CL) while SRS is engaged. SRS arms ALT* toward a
higher FCU altitude. AP1+AP2 stay engaged in GA; leaving GA reduces to one AP
(existing rule). There is no FMS acceleration altitude: the crew selects the
next vertical mode.

## Yaw / rollout channel

`sim/operation/override/override_joystick_heading` + `sim/joystick/yoke_heading_ratio`
are leased only in ALIGN and ROLL OUT with an AP engaged, with the pitch/roll
rules: released on pause, replay, plugin disable/unload, AP disconnect, foreign
native AP; a foreign heading override at that moment disconnects the AP; losing
the lease disconnects the AP; the physical rudder axis (assignment 3) beyond
40 % travel is a pilot override (`yoke_heading_ratio` is ignored as an override
source while it is our own output). X-Plane's nosewheel steering follows the
pedals (ACF: nose gear 90 deg low speed / 7 deg high speed); FBW's separate
`Nosewheel_c` output is computed but not applied.

- ROLL OUT: the FBW law's autopilot `Beta_c_deg` is, on the ground, a rudder
  position command (the A380 lateral normal law passes it through as zeta);
  pedal = -Beta_c / 30 deg. The law blends from the rudder position at
  engagement over 3 s (FBW).
- ALIGN: port law. The rudder aligns the heading with the memorized
  localizer course (gains 0.12 pedal/deg, 0.02 /deg s, 0.45 /deg/s yaw rate).
  The FBW align beta command (1.5 x (beta - heading error)) is constant in a
  steady decrab and, tracked by a direct sideslip servo, settles at 150 % of the
  drift; the FBW lateral law that integrates it is not ported. The FBW LOC TRACK
  roll law still applies its own wing-down term from that command.

## Other guidance changes

- FBW glide path angle is negative: the GS TRACK law saturates the stored angle
  to [-4, -1] deg, so the previous +3 deg became -1 deg and the aircraft drifted
  above the beam (bench: about 2.5 dots high at 1000 ft). The bridge now passes
  -3 deg. This also affects G/S tracking outside LAND.
- Sideslip is passed to FBW with FBW's sign (positive nose-left of the path,
  derived from the FBW lateral normal law's beta loop and the align command;
  X-Plane's `position/beta` is positive nose-right). Verify in-sim.
- The port's AP speed guard is VLS - 10 kt in approach/GA modes (standing in for
  the alpha-prot + 1 deg disconnect) and not applied in LAND; elsewhere VLS as before.
  (In the FBW PRIM the VLS/VMAX/attitude terms only inhibit AP engagement.)
- LAND/ROLL OUT convert the angular localizer deviation with the distance to
  the localizer antenna (X-Plane navaid database by frequency) instead of the
  ILS DME, which X-Plane usually places at the glide antenna. FBW clamps this
  distance to >= 2 nm.
- FLARE and SRS GA add attitude-error gain (0.3) and an integrator (0.2/deg s)
  to the port yoke servo, which otherwise lags the FBW attitude increments by
  about a degree (bench: 490 fpm touchdowns without it).
- New read-only outputs `a380x/xp12/fcu/{land_active, flare_active,
  rollout_active, retard_active, go_around_active, land_capability,
  yaw_servo_active}` for in-sim monitoring.

## Verification

- `test_autoland.c`: LAND confirm timing and gating (RA, APPR, FD-only),
  FCU inhibits, no reversion on LOC loss, 100 ft rule retained without LAND,
  RETARD at 50 ft only with AP, FLARE latch, ROLL OUT at 0.36 s, ground A/THR
  idle hold and disconnect by IDLE or reverse, ground reset, GA engagement /
  inhibits / thrust modes / exits, LAND 2 gating, yaw release on pause.
- `test_fcu_adapter.c` `test_yaw_lease`: yaw override lease, pause/replay,
  AP disconnect, lease theft, foreign owner, physical pedal override, unload.
- `test_autoland_sim.c` (report `autoland-bench.txt` in the build output):
  the real guidance and FBW laws fly LFBO 14R (X-Plane 12 default navdata: LOC
  4091 m beyond the threshold, GS antenna 325 m past and 122 m left, TCH 55.8 ft,
  DME at the glide antenna) from 11.9 nm, 2000 ft, 900 m left of the
  centreline, with AP1+AP2 and A/THR, crew IDLE at the 10 ft RETARD call,
  reverse at touchdown + 2.5 s, autobrake 2 m/s2, AP off at 30 kt. The airframe is a
  bench model (386 t, CL0 0.95, CLalpha 5/rad, ground effect, oleo/tyre model,
  log wind profile, first-order turbulence), not X-Plane's.

| Case | LAND / FLARE / RETARD | Touchdown | Sink | Crab / bank | Roll-out max |
| --- | --- | --- | --- | --- | --- |
| Calm, 386 t | 386 / 50 / 50 ft, flare 5.4 s | 372 m, 0.8 m off | 270 fpm | 0.0 / 0.0 deg | 0.8 m |
| 15 kt from left, 5 kt head, light turbulence | 387 / 49 / 50 ft, 5.7 s | 408 m, 1.0 m | 209 fpm | -1.8 / -0.6 deg | 1.8 m |
| 25 kt from right, 10 kt head, moderate turbulence | 387 / 56 / 50 ft, 8.3 s | 449 m, 0.8 m | 337 fpm | 1.3 / 1.5 deg | 6.0 m |
| 300 t, 10 kt tailwind, IDLE 1 s after touchdown | 384 / 54 / 50 ft, 6.8 s | 473 m, 0.4 m | 92 fpm | 0.1 / 0.2 deg | 1.7 m |
| TOGA at 150 ft | SRS + GA TRK at 150 ft | min RA 123 ft (27 ft loss) | - | - | ALT* at 1850 ft, peak 3206 ft for FCU 3000 ft |

Crosswind at 20 ft; the wind is 1.8 times stronger at 1000 ft. In the 300 t
case RETARD holds idle on the ground with the levers at CL until the crew sets
IDLE (A/THR off 1.0 s after touchdown). The go-around level-off overshoot is
the existing ALT* capture and A/THR speed regulation, not this module.

## Needs in-sim tuning / not done

- Everything above was verified offline only. The bench airframe is not the
  X-Plane A380; flare height, sink rate, decrab and roll-out depend on X-Plane's
  pitch, roll, rudder and nosewheel response.
- Port gains: FLARE/SRS yoke gains, ALIGN pedal gains, ROLL OUT pedal scale
  (30 deg), ground pitch bound, RETARD ramp. The base pitch servo has an
  almost zero integrator (pre-existing), which also limits ALT* and SRS.
- X-Plane specifics to check: radio altimeter reading 0 at main-gear touchdown
  (FLARE and ROLL OUT use RA), the sign of `position/beta`, localizer/glide
  dots per degree (the bridge assumes 1 deg/dot LOC, 0.28 deg/dot G/S), pedal
  steering blend with ground speed.
- Not implemented: LAND 3 capability, FMS go-around phase and acceleration
  altitude, SRS takeoff, capability-downgrade triple click (the aurals module
  can use `land_capability`), BSCU nosewheel command, GA from the ground roll
  after more than 30 s.
