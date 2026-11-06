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

Managed speed, lateral and vertical requests require valid FMS target inputs.
There is no ported FMS route producer in this package. A push without those
inputs preserves selected guidance and briefly displays `FMS GUIDANCE N/A`;
it must not claim NAV, CLB or DES capture or silently use the stock GPS/VNAV.
The controller interface supports independently validated managed targets for
later FMGC integration. A managed preselection window times out after 45 s.

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
is allowed with approach selected, in LAND and in go-around. Below 400 ft RA the
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

## Boundaries that remain

This is selected-flight guidance plus raw ILS capture and an ILS autoland, not
a complete Airbus AFS. FMS predictions/constraints, takeoff SRS, the FMS
go-around phase, alpha-floor and full flight-envelope protection, FADEC detent
logic, TCAS guidance and redundant PRIM/ARINC bus networks are outside this
implementation. LAND/FLARE/ROLL OUT, RETARD and SRS GA/GA TRK follow the FBW
PRIM FG mode logic ([AUTOLAND.md](AUTOLAND.md)); without LAND engaged the AP
still disconnects below 100 ft radio altitude in captured glideslope modes.
The highest landing capability annunciated is LAND 2; LAND 3 SINGLE/DUAL is
never shown because the port has no redundant monitored computers. Missing
performance values (VLS, V1, VR, green dot) are omitted. The autoland was
verified offline only and needs in-simulator tuning.

## Verification

`native/avionics/build.py` compiles the unchanged FBW laws, numerical guidance
tests, display tests and the native adapter for universal macOS and Windows
x86-64. The source manifest and registry checks bind the binary contract to
the physical controls. Focused Python tests validate hit-zone separation,
custom command/dataref ownership, atlas orientation and packaging.

Offline tests verify controller transitions and response directions, not a
complete aerodynamic closed-loop flight. Runtime acceptance must include HDG
and TRK turns, climb/descent and altitude capture, positive/negative V/S/FPA,
Mach transition, A/THR response, ILS intercept/loss, AP2 source failures,
pause/resume and manual disconnect. The existing ACF flight model and
port-specific actuator gains still require in-simulator tuning.
