# A380 FCU and imported FlyByWire guidance

The aircraft-local `A380XAvionics` plugin owns the physical FCU, both PFDs,
flight directors, selected guidance, AP1/AP2 and A/THR. The previous
`A380X_XP12_Autopilot.lua` remains a historical source-only comparison and must
not be installed alongside this controller.

## Imported implementation

`native/avionics/vendor/fbw/` contains the **unmodified A380FgOuterLoops** C++
implementation and its support files from FlyByWire aircraft revision
`2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2`. The manifest records original paths
and SHA256 hashes, and the GPL license is retained. `a380x_fbw_laws.cpp` bridges
raw aircraft and ILS measurements into these original flight-guidance laws.

The FCU mode state machine in `a380x_guidance.c` adapts the pinned A380 FCU/PRIM
interfaces and [FlyByWire's A380 AFS documentation](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/glareshield/afs/)
to X-Plane. The complete PRIM/FMS/FADEC computer network is **not** imported.
The final attitude-to-yoke servo and speed-to-throttle regulator are explicitly
port-specific and require aircraft flight-test tuning. They do not delegate
selected guidance to X-Plane's native autopilot or consume its mode flags.

## Controls

All cockpit commands use `a380x/xp12/fcu/`. They are also available for keyboard
and hardware binding. Each rotary knob has outer rotation zones, an upper
center PUSH zone and lower center PULL zone with distinct hover labels.

| Control | Behavior |
| --- | --- |
| SPD/MACH | Changes display/selection units while preserving the equivalent target |
| Speed rotation / pull | Selects 100–399 kt or Mach .01–.99 / uses selected speed |
| Heading rotation / pull | Selects 000–359° / HDG or TRK guidance |
| Altitude rotation / pull | Selects altitude in 100/1000-ft increments / OP CLB or OP DES |
| V/S rotation / pull | Preselects ±6000 fpm (50 fpm/detent) / uses V/S |
| FPA rotation / pull | Preselects ±9.9° (.1°/detent) / uses FPA |
| V/S/FPA push | No effect, as specified for the A380 |
| ALT pushbutton | Levels off at the current altitude |
| HDG/TRK | Changes paired HDG–V/S / TRK–FPA selections |
| TRUE/MAG | Changes north reference while preserving the physical heading target |
| METER | Shows current and selected metric altitudes on both PFDs |
| AP1 / AP2 | Engages/disengages each channel, subject to airborne/sensor/attitude gates |
| FD | Toggles both flight directors |
| A/THR | Arms/activates or disconnects the port's thrust regulator |
| LOC / APPR | Arms localizer / localizer and glideslope capture from raw receivers |
| LS on each EFIS | Toggles that side's landing scales and receiver information |
| Sidestick priority | Disconnects the custom AP |
| Thrust instinctive buttons | Disconnect A/THR |

Managed speed, lateral and vertical requests require valid FMS target inputs,
produced by the native FMGC module (see *Managed guidance* below) from the
native FMS flight plan. A push without those inputs (no flight plan, FMS
invalid, discontinuity) preserves selected guidance and briefly displays
`FMS GUIDANCE N/A`; the stock GPS/VNAV is never used. A managed preselection
window times out after 45 s.

Seven authored button annunciator faces reflect AP1/AP2, FD, A/THR armed, LOC, ALT and APPR state with power gating. Mechanical knob travel is not animated.

FCU numbers, dashes, decimal signs and managed dots are drawn by the native
plugin into four cockpit texture regions. There are no stock LED instruments
reading unrelated native autopilot targets. The PFD mode annunciations,
selected bugs, flight directors and AP/FD/A/THR status use the same live state.

## Simulator boundary

Raw pilot/copilot attitude, air data, magnetic variation, body rates and
accelerations, pressure altitude, mass and independent NAV1/NAV2 signals feed
the controller. AP2 alone selects the first-officer input channel. A VOR signal
cannot satisfy localizer capture. Signal loss cancels capture/reverts guidance;
invalid sensors disconnect the AP. VMAX uses the lower of the aircraft IAS and Mach limits, with a compressible air-data conversion. Simultaneous AP1+AP2
is allowed with approach selected, in LAND and in go-around.

**No engagement limits (user decision, 2026-11-09).** The A380 manual allows AP engagement only from
5 s after liftoff, with pitch between −11 and 22.5°, bank below 40° and speed within VLS..VMAX. The AP
then disconnects beyond −13/25° pitch, 45° bank, below VLS or above VMAX. The port applies none of
these limits:
* Pressed in flight, the AP engages whenever it has the flight data it needs (power, ADR air data,
  IR attitude/heading). Only the loss of that data disconnects it.
* Pressed on the ground, the press is kept (`AP1 armed: it engages by itself after liftoff`). The
  AP engages once airborne above 100 ft RA (5 s airborne without a radio altitude), with the stick
  and rudder released. A second press, or the AP disconnect button, cancels it.

Below 400 ft RA the
approach continues into LAND, FLARE and ROLL OUT with A/THR RETARD, and TOGA
engages SRS GA / GA TRK: see [AUTOLAND.md](AUTOLAND.md).

After ground startup, the FD remains in standby through liftoff until an explicit supported vertical selection or AP engagement. Engaging AP from standby captures the current airborne altitude, never the previous runway altitude.

The adapter owns pitch/roll joystick overrides only while AP guidance is
valid, the yaw (rudder pedal, and through it nosewheel steering) override only
in LAND ALIGN and ROLL OUT, and throttle override only while A/THR is active. It refuses competing
owners, and releases its own overrides on pause, replay, disable and unload.
AP and A/THR disconnects release their respective control channels independently.
Physical joystick movement and the standard native servo-disconnect command
can disconnect the custom AP. The A380 experimental FlyWithLua surface-control
law remains a separate, unvalidated controller; keep it disabled during AP
flight validation.

## Managed guidance (FMGC: LNAV, VNAV, managed speed)

`native/avionics/a380x_fmgc_guidance.[ch]` (pure C11, no XPLM) turns the ACTIVE
flight plan of `a380x_fms_fpln` and the native FMS outputs into the managed
inputs of `A380XGuidanceInputs`. `a380x_fms_adapter.inc` runs it every frame
after the FMS (it also owns leg sequencing, replacing `a380x_fpln_update`) and
passes the result through `fms_feed` to `a380x_fcu_adapter.inc`. The managed
FG modes live in `a380x_guidance_managed.inc`, included by `a380x_guidance.c`.

**Provenance.** The AP/FD laws that fly the guidance are the unmodified FBW
`A380FgOuterLoops` (HPATH lateral law 3, VPATH vertical law 9, SPD/MACH law 3,
V/S law 4, SRS law 8). Their inputs follow the FBW FG interface: cross-track
error, track-angle error and bank feed-forward (`fms_xtk_nmi`, `fms_tae_deg`,
`fms_phi_deg`; heading legs fly zero XTK and a heading error as FBW's
`ControlLaw.HEADING`), profile altitude and vertical speed
(`fms_H_c_profile_ft`, `fms_H_dot_c_profile_ft_min`). Sign convention, verified
against the law and the FBW PFD's XTK readout: XTK > 0 means the path lies to the
right of the aircraft. The managed-profile constraint logic (`managedProfile`,
FAI.ts:1459-1603, spec 07 sections 5.7/6.3) moved unchanged from the FMS adapter
into the module so the offline flight shares it; descent decelerations use
FBW's `decelDist`; the approach managed speed is max(VAPP, maneuvering speed of
the flap lever) as in A32NX `FMCMainDisplay.updateManagedSpeed`. The path
geometry, sequencing, descent profile and mode logic are **original C written
for this port**, not a translation of the FBW TypeScript guidance/VNAV
(`fbw-common/src/systems/fmgc/src/guidance`), whose path vectors, transition
classes and performance predictor are not imported.

**LNAV.** Leg types: TF/IF/PI (line, or direct without a start), CF (inbound
course line through the fix), DF (pursuit while turning, then a frozen great
circle), RF/AF (arcs, bank feed-forward atan(GS²/gR)), FA/FC/FD/FM (outbound
course line from the fix), CA/CD/CI/CR (track), VA/VD/VI/VM/VR (heading, managed
heading output), HA/HF/HM (racetrack: inbound, 180° turns at the hold radius,
outbound for the coded time/distance; IMMEDIATE EXIT returns direct to the fix;
entries below). Fly-by transitions: circular arcs of a 20° design-bank
radius at ground speed, anticipation R·tan(Δ/2) (Δ ≤ 135°); fly-over fixes and
hold entries sequence at the fix. Terminations: altitude (CA/FA/VA/HA), DME
crossing (CD/FD/VD), radial crossing (CR/VR), next-leg intercept (CI/VI), along
distance (FC), never automatic for FM/VM. Sequencing writes the TO leg through
`a380x_fpln_sequence_to` (ND and MFD follow it); outside NAV, fixes sequence
abeam within 7 NM. A TO waypoint followed by a discontinuity: NAV is lost when it
is passed (HDG on the current heading, mode reversion; CLB → OP CLB, DES → V/S).
NAV arms on the ground (route valid, FD on, HDG not preselected) and engages at
30 ft RA; in flight HDG push engages NAV inside its capture zone
(|XTK| ≤ R(1−cos intercept) converging, or on the path), otherwise arms it. DIR
TO, F-PLN edits and new plans reset the leg state.

**Holds.** The entry is chosen over the fix from the arrival track relative to
the inbound course (ICAO PANS-OPS / Airbus sectors, right-hand pattern; mirrored
for left turns): direct −110°..+70°, teardrop +70°..+140°, parallel otherwise,
with the ±5° flexibility resolved towards direct at both direct boundaries and
towards teardrop at +140° (`a380x_fmgc_hold_entry`). Teardrop: outbound from the
fix offset α towards the holding side, tan(α/2) = R/L (20–60°; the 180°+α turn
then rolls out on the inbound course), turn, inbound. Parallel: outbound along
the inbound course line on the non-holding side for the leg length, a 225° turn
towards the holding side, direct to the fix, then the racetrack. Hold speed is
FBW `getHoldingSpeed` (spec 07 §5.8): max(FCU, hold-constraint) altitude, green
dot (+25 kt engine out; GD/S/F/VAPP in the approach phase), the hold speed
constraint, ICAO caps 230/240/265 kt and M0.83 above FL340, the climb/descent
speed limit below its altitude, VLS floor; it is the managed target in the hold
and from decelDist = clamp((CAS − hold) × 0.15, 3, 20) NM before the fix of a
next-leg hold (`updateHoldingSpeed`), shown on the MFD HOLD line. In a hold the
profile takes the hold exit at the fix (distance to the fix).

**Pseudo waypoints (ND + MFD).** One list (`A380XFmgcOutputs.nd_pwp`, nearest
first) feeds the MFD F-PLN lines and both NDs, so they share the distances:
(T/C) / LEVEL climb level-off (FCU, cruise level or a climb constraint; distance
from the filtered V/S with the climb rate falling linearly to zero at 43000 ft),
START of climb at the constraint waypoint (CLB armed in ALT CST), (SPDLIM) speed
change at the climb speed-limit altitude, (T/D), descent (SPDLIM), (DECEL),
descent LEVEL (on the profile in DES, from the V/S in OP DES / V/S), START of
descent after an ALT CST level-off with DES armed, INTCPT where the present V/S
meets the profile. Symbols are FBW's ND `typePwp` set (level-off, start of
climb/descent, DECEL, speed-change dot, intercept); colours magenta in the
managed vertical mode (DECEL and speed changes: managed speed), cyan selected or
armed, T/D white. With NAV they sit on the flight-plan path; in HDG/TRK only the
level-off and intercept arrows are drawn, along the track line. MFD idents are
FBW's A380 labels `(T/C)`, `(T/D)`, `(DECEL)`, `(SPDLIM)`. Web ND: the plugin sends
`nav.pseudo` (`a380x_fmgc_nd_pwp_json`), and `xp-kit-nd/fmgc-pwp-layer.js` appends
the symbols to the native bridge's FBW `symbols` bus message (re-sent at most at
4 Hz when only they move); the bundle's own PseudoWaypointLayer draws them. The
layer is inserted after the native bridge by `tools/patch_xp_kit_nd.py` (now an
ordered list: TERRAIN, then pseudo waypoints; known page hash, exact anchor,
idempotent, manifest hashes per layer) and inlined by a full `package_nd`. Native
ND (both layouts): `a380x_nd_pwp.inc`, FBW's symbol paths in FBW pixels.

**VNAV.** Managed CLB: speed on pitch (law 3), A/THR THR CLB, target
min(FCU, next climb constraint); a constraint level-off is ALT CST* / ALT CST
with CLB armed and resumes when the constraint is sequenced. Level at the FMS
cruise altitude: ALT CRZ. Descent path: built backwards from the runway
(threshold-crossing constraint, else elevation + 50 ft) along the flown path
(legs shortened by their fly-by arcs): coded FPA legs geometric, otherwise an
idle gradient of 3.0° (still air) × current TAS/GS × a learned idle factor
(0.7–1.3, learned while descending at idle with the speed on target), altitude
constraints as clamps (shallower = geometric, steeper = too steep), 1/3-gradient
deceleration segments before descent speed constraints and at the speed-limit
altitude. T/D, DECEL (1000 ft AAL on the final path + (Vlim − VAPP)/10 + 2 NM)
and a T/C estimate from the current climb rate are computed. DES submodes: on
path (VPATH law 9; THR IDLE on idle segments, SPEED/MACH on geometric ones),
above path or no path (idle, speed on pitch, +10 kt in the speed range; also
when more than 10 kt fast at idle), below path or before T/D (V/S −1000 ft/min,
SPEED). Floors are ALT CST; a constraint the path descends through is flown by
VPATH, not captured. `MORE DRAG` is an output only (the FBW A380 PFD bundle has
no MORE DRAG FMA message). DECEL activates the APPROACH phase
(`vnav_decel_reached`) when NAV is engaged.

**Managed speed.** Takeoff SRS: V2 on the ground, V2 + 10 airborne (engages
with T.O thrust, FD and a V2; hands over to CLB/OP CLB at the acceleration
altitude). Climb/cruise/descent: the FMS laws (250 kt below the speed-limit
altitude, ECON CAS/Mach with crossover, constraints); the FMGC additionally
starts the speed-limit and constraint decelerations ahead of the fix. Approach:
max(VAPP, GD/S/F/VAPP by lever). FMA: armed NAV/CLB/DES, ALT CST(*)/ALT CRZ(*),
SRS, magenta FM altitude constraint, linear deviation, DES speed margins and
T/D REACHED are published to the FBW PFD words (`a380x_xpkit_adapter.inc`); the
native PFD shows the armed NAV/CLB/DES texts. The MFD F-PLN gets basic
predictions (pseudo waypoints T/C, T/D, DECEL; per-leg time from the current
ground speed, altitude from the profile / climb rate; no fuel or wind).

**A/THR.** THR CLB in SRS/CLB/OP CLB, THR IDLE in OP DES and idle DES
submodes, SPEED/MACH (managed Mach when the managed target is Mach) elsewhere.
The port's speed regulator now has bumpless entry and speed-trend anticipation
(P 0.04/kt, I 0.003/kt/s, trend 0.25/(kt/s), engine-command units); THR IDLE
remains the existing 0.04 engine-command approximation.

**Verification.** `test_fmgc_guidance.c` (3682 checks: radius/anticipation,
90° fly-by sequencing at R·tan45 with ≤ 0.04 NM XTK, discontinuity NAV loss,
VA/CD/CR/CI terminations, RF arc ≤ 0.01 NM, HM hold circuits and IMM EXIT,
29 hold-entry sector cases incl. the ±5° boundaries and left turns, seven flown
entries (direct straight-in/abeam, teardrop R/L, parallel R/L/reciprocal: entry,
phases, inbound XTK ≤ 0.026 NM after the entry, excursions, IMM EXIT), holding
speed table and the hold-speed target/decel/MFD feed, DIR TO pursuit/freeze/reset,
capture zone, descent profile with FPA/constraints/deceleration/T/D, managed
altitude and approach speeds, MFD predictions, pseudo waypoints in climb (T/C,
level-off at the FCU / a constraint, start of climb, speed limit; NAV and HDG),
cruise/descent (T/D, SPDLIM, DECEL managed/selected, OP DES level-off, intercept)
and the MFD lines at the same distances) and the managed-mode cases in
`test_guidance.c`. `test_pfd_render.c` draws the list on the FBW-layout native
ND (positions, colours, vertex counts; preview `nd-fbw-pwp.svg`);
`tools/pwp_layer_check.js` runs the web layer after the real bridges on the
flight's ND states against the bundle's PseudoWaypointLayer/WaypointLayer
filters. `test_fmgc_flight.c` flies EGLL 27R BPK7F – BPK (HOLD) – LEST1M – ILS 23R
EGCC (174 NM + the hold, 420 t, wind 250°/35 kt) on the
installed CIFP with the real FMS, FMGC, guidance state machine, FBW laws and
actuator servos against a point-mass aircraft (lift/drag polars per
configuration, four spooling engines, ISA, pitch/roll actuator response as in
`test_guidance.c`; idle descent ≈ 3°), with a scripted crew (TOGA, CL at THR RED,
AP1 after 6 s, FCU 6000 for the hold at BPK, IMMEDIATE EXIT 20 s into the first
outbound leg, FCU FL240 + ALT push after the hold, FCU 3000 ft 2 NM before T/D
and ALT push 1 s later, speed brakes on MORE DRAG, flaps on schedule, FCU 1000 ft
in the approach phase). Hold BPK (inbound 314°M, right, 1 min, arrival track
064°): teardrop entry, hold speed 210 kt (green dot) reached before the fix
(CAS within 0.8 kt), inbound XTK ≤ 0.11 NM after the teardrop, max 0.35 NM in
the hold (25° bank in the wind at the 20° design radius), 6000 ft held, exit to
LESTA. Pseudo waypoints against the flight: (SPDLIM) predicted 4.8 NM, flown
4.3; (T/C) predicted 30.8 NM above FL120, flown 34.6 (+12 %); the T/D symbol
2.00 NM ahead at the descent clearance (position error 0.00 NM; MFD line
identical); the DECEL symbol 0.31 NM from where the approach phase activated.
Result (build
output `fmgc-flight.txt`): FMA MAN TOGA/SRS/NAV(30 ft RA) → THR CLB/CLB →
ALT CST* → ALT CST 6000 → CLB → ALT CRZ FL240 → DES (early V/S −1000, on path
THR IDLE / SPEED, above path) → APPROACH at DECEL → DES on the final path to
1500 ft AAL 4.9 NM from the threshold; XTK max 0.12 NM established on straight
legs, 0.51 NM in the tight SID turn sequence D278F–D304H–D325H, 0.007 NM on the
final; level-offs +26 ft (ALT CST) and +22 ft (ALT CRZ); every altitude and
speed constraint met within 250 ft / 10 kt; speeds 250 kt below FL100, 290 kt
climb, 289 kt descent, 250/230 kt at the constraints, GD→S→F→VAPP. These are
bench results on a model airframe, not X-Plane flight-model results.

**Needs in-sim tuning / not done.** The X-Plane A380's idle descent angle (the
learned factor and speed brakes compensate), A/THR gains and the 0.04 idle
command, the 20° transition bank, NAV capture zone, DES thresholds, SRS on the
X-Plane model, and the existing AP servo gains. The hold radius (20° bank at the TAS when
the hold leg activates) and the teardrop angle, the T/C climb-rate model and
the ND symbol scale/colours need checking in the sim. Not done: PI procedure
turns (flown as TF), RNP/offsets, APP DES / FINAL APP, engine-out, step climbs,
fuel/wind predictions, speed-change symbols at descent speed constraints,
FLAP1/FLAP2 and time markers, holding-pattern predictions (the profile takes
the hold exit at the fix), missed-approach sequencing outside the FMS GOAROUND
phase.

## When AP or A/THR will not engage

The FCU adapter reports every refusal and every automatic disconnect in the simulator window
(`XPLMSpeakString`; spoken too when X-Plane's text-to-speech is on). It also writes the same text to
`Log.txt` as `A380X AFS: ...`. The same text repeats at most once every 5 s. The AFS lines, AP/A/THR
pushbutton results and status lines are also written to `Output/A380X-XP12_AFS.log` (this X-Plane
session) and `Output/A380X-XP12_AFS.previous.log` (the session before). X-Plane keeps only a few
`Log.txt` archives, so these files keep the evidence of the last A380 flight.

* **AP1/AP2 not engaged** lists the failing `can_engage` terms in plain language
  (`a380x_guidance_engage_problems`):
  * not powered;
  * air data invalid (ADR) or attitude/heading/navigation invalid (IR);
  * no valid airspeed.

  A press on the ground is kept for after liftoff instead.

  It also reports when another add-on holds `override_joystick`, `override_joystick_pitch` or
  `override_joystick_roll`, because the press is then ignored.
* **AP disconnected** names the cause:
  * a joystick pitch/roll/yaw axis more than 20 % off centre (axis index and value: centre or
    recalibrate it);
  * rudder pedal input;
  * the flight controls taken by another add-on;
  * a lost control lease;
  * a guidance limit (`AP disconnected by the guidance: ...`).
* **A/THR not armed** is reported when another controller holds `override_throttles`, or when A/THR
  is disabled after a 15 s instinctive-disconnect hold.
* **A/THR armed but not active** is reported 2 s after the press when one of these holds:
  * thrust levers in FLX/MCT or TOGA (manual thrust), or at IDLE;
  * levers not recognised;
  * on the ground, speed data invalid, paused or unpowered.

In flight, while FD, AP or A/THR is in use, an `A380X AFS status:` line goes to `Log.txt` every 10 s. It
contains:

* AP/FD/A-THR flags and the thrust, lateral and vertical modes;
* IAS, VLS, VMAX, altitude, V/S and heading, plus the selected targets;
* the four lever ratios and their validity, against CL;
* the override and ownership flags, the thrust lease, and ADR/IR validity.

One short flight log is enough to see why a mode did not engage or follow.

## Boundaries that remain

This is selected and FMS-managed guidance plus raw ILS capture and an ILS autoland, not a
complete Airbus AFS. Takeoff SRS is the FBW law with the port's mode logic; LAND/FLARE/ROLL OUT,
RETARD and SRS GA/GA TRK follow the FBW PRIM FG mode logic ([AUTOLAND.md](AUTOLAND.md)). The FMS
go-around phase, alpha-floor and full flight-envelope protection, FADEC detent logic, TCAS guidance
and redundant PRIM/ARINC bus networks are outside this implementation. Without LAND engaged the AP
still disconnects below 100 ft radio altitude in captured glideslope modes. The landing
capability (APPR 1 / LAND 2 / LAND 3 SINGLE / LAND 3 DUAL) follows the FBW PRIM capability terms
evaluated on the simulated redundancy (two NAV receivers, two radio altimeters, two ADIRS sides,
AP1+AP2, A/THR, hydraulics, rudder), frozen below 200 ft, with a triple click on a downgrade. It is
an annunciation from simulated sensors, not certified fail-passive / fail-operational monitoring
(the port has one guidance computer). Missing performance values (VLS, V1, VR, green dot) are omitted.
Managed guidance and autoland were verified offline only and need in-simulator tuning.

## Verification

`native/avionics/build.py` compiles the unchanged FBW laws, numerical guidance
tests, display tests and the native adapter for universal macOS and Windows
x86-64. The source manifest and registry checks bind the binary contract to
the physical controls. Focused Python tests validate hit-zone separation,
custom command/dataref ownership, atlas orientation and packaging.

Offline tests verify controller transitions and response directions; the
managed-guidance flight (`test_fmgc_flight.c`, above) closes the loop on a
point-mass model, not on the X-Plane flight model. Runtime acceptance must include
managed SID/STAR/approach flying (NAV, CLB, ALT CST, DES, A/THR modes), HDG
and TRK turns, climb/descent and altitude capture, positive/negative V/S/FPA,
Mach transition, A/THR response, ILS intercept/loss, AP2 source failures,
pause/resume and manual disconnect. The existing ACF flight model and
port-specific actuator gains still require in-simulator tuning.
