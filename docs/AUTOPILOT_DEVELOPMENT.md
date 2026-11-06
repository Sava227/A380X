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
invalid sensors or excessive attitude disconnect the AP. VMAX uses the lower of the aircraft IAS and Mach limits, with a compressible air-data conversion. Simultaneous AP1+AP2
is allowed only with approach selected and does not imply autoland capability.

After ground startup, the FD remains in standby through liftoff until an explicit supported vertical selection or AP engagement. Engaging AP from standby captures the current airborne altitude, never the previous runway altitude.

The adapter owns pitch/roll joystick overrides only while AP guidance is
valid and throttle override only while A/THR is active. It refuses competing
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
outbound for the coded time/distance; direct entry only; IMMEDIATE EXIT returns
direct to the fix). Fly-by transitions: circular arcs of a 20° design-bank
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

**Verification.** `test_fmgc_guidance.c` (3551 checks: radius/anticipation,
90° fly-by sequencing at R·tan45 with ≤ 0.04 NM XTK, discontinuity NAV loss,
VA/CD/CR/CI terminations, RF arc ≤ 0.01 NM, HM hold circuits and IMM EXIT,
DIR TO pursuit/freeze/reset, capture zone, descent profile with FPA/constraints/
deceleration/T/D, managed altitude and approach speeds, MFD predictions) and the
managed-mode cases in `test_guidance.c`. `test_fmgc_flight.c` flies EGLL 27R
BPK7F – BPK – LEST1M – ILS 23R EGCC (174 NM, 420 t, wind 250°/35 kt) on the
installed CIFP with the real FMS, FMGC, guidance state machine, FBW laws and
actuator servos against a point-mass aircraft (lift/drag polars per
configuration, four spooling engines, ISA, pitch/roll actuator response as in
`test_guidance.c`; idle descent ≈ 3°), with a scripted crew (TOGA, CL at THR RED,
AP1 after 6 s, FCU 3000 ft and ALT push 2 NM before T/D, speed brakes on MORE
DRAG, flaps on schedule, FCU 1000 ft in the approach phase). Result (build
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
X-Plane model, and the existing AP servo gains. The existing rule that
disconnects the AP below VLS stays. Not done: hold entry types other than
direct, hold speeds, PI procedure turns (flown as TF), RNP/offsets, APP DES /
FINAL APP, engine-out, step climbs, fuel/wind predictions, ND pseudo-waypoint
symbols (T/C, T/D, DECEL appear on the MFD F-PLN only), missed-approach
sequencing outside the FMS GOAROUND phase.

## Boundaries that remain

This is selected and FMS-managed guidance plus raw ILS capture, not a complete
Airbus AFS. Takeoff SRS is the FBW law with the port's mode logic; go-around SRS, alpha-floor and full
flight-envelope protection, FADEC detent logic, LAND/FLARE/ROLLOUT, TCAS guidance
and redundant PRIM/ARINC bus networks are outside this implementation. The AP
disconnects below 100 ft radio altitude in captured glideslope modes; perform a
manual landing. LAND 2/3 and approach certification annunciations are not
invented. Missing performance values (VLS, V1, VR, green dot) are omitted.

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
