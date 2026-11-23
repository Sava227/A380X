# Native PFD update — 6 October 2026

The installed `A380X-XP12` cockpit uses `native/avionics/a380x_pfd_render.inc`.
The separate Desktop `avionics-sasl` preview is not this aircraft's panel
renderer. This update is installed through the native avionics provider.

## Display changes

- The PFD uses the bundled FlyByWire A380 EIS font and the pinned PFD sky,
  earth and tape colours. Heading, attitude, altitude, radio height, Mach,
  vertical speed and ILS remain separate captain/first-officer sensor channels.
- The V/S number follows its needle in absolute hundreds of ft/min. Amber
  begins at ±6000 ft/min, descent at or below −2000 ft/min at valid RA
  1000–2500 ft, or descent at or below −1200 ft/min at valid RA ≤1000 ft.
  The 1000 ft boundary belongs to both descent tests. This colour does not
  by itself engage a flight-control protection.
- Negative radio height is preserved. RA rounds to 1 ft below 5 ft, 5 ft
  through 50 ft, and 10 ft above 50 ft; it disappears at 2500 ft. The native
  radio-altimeter bug supplies the side's decision height. RA is green above
  400 ft or DH+100 ft and otherwise amber; DH appears at/below that setting.
- The rising-ground mask follows pitch, bank and radio height, hiding the
  lower pitch ladder on the ground. Its geometry is clipped to the horizon.
- Native positive-right bank now rotates the horizon, ladder, rising ground
  and moving bank index in the correct direction. The SVG-to-OpenGL conversion
  accounts for their opposite vertical coordinate directions.
- Mach appears above .500 and remains displayed until below .450. Excessive
  attitude decluttering uses separate entry and recovery thresholds.
- A.FLOOR and TOGA LK use a flashing amber frame when those guidance modes
  are actually supplied. The renderer never engages autothrust.
- The configuration area shows a clean wing, continuous actual slat/flap
  positions, cyan moving targets, green achieved positions, spoiler deployment,
  spoiler arming, and the gear-down indication. Fault/invalid positions use amber.

## Aircraft inputs and remaining fidelity limits

`a380x_pfd_inputs.inc` reads actual native surface deployment separately from
the requested position. Read-only `a380x/xp12/pfd/*` datarefs expose the exact
snapshot consumed by the renderer, including source validity and limit speeds.
The requested flap position is converted through the aircraft's native detent
angle table before comparing it with measured deployment. In the installed
aircraft, handle 0.5 requests 10° out of 40°, so the settled deployment is 0.25.

VMAX combines the installed VMO/MMO, configuration limits and gear limits.
The A380 reference flap limits are 263/222/220/196/182 kt; the port retains the
more restrictive selected or remaining actual configuration during movement.
Green Dot uses the pinned A380 weight/pressure-altitude table. These are port
calculations, not a completed PRIM/SFCC implementation.

Clean VLS uses the exact 96-point table from the pinned PRIM flight-envelope
computer. `clean_performance_valid` is separate from full-envelope validity.
This fallback requires healthy, fully retracted flaps/slats, a clean lever,
stowed speedbrakes, valid airdata, 250–600 tonnes and pressure altitude within
0–43,000 ft. It disappears outside those conditions. It is a steady-state
performance lookup, not a functioning stall-protection law.

With the FLAPS module ([FLAPS_LEVER.md](FLAPS_LEVER.md)) the slat/flap part of the configuration
area and the flap VMAX / VFE next come from the FBW SFCC and FPPU drives instead (`sfcc_valid`): the
LowerArea.tsx text, pointer laws, targets, F RELIEF and A LOCK; the paragraphs below describe the
legacy path used when that module is unavailable.

The current ACF has six generic flap detents. Its surface schedule, spoiler
logic, aerodynamic model and CG data are not a full A380 implementation.
The schematic maps native normalized deployment; it does not reproduce SFCC
synchro-angle conversion, alpha-lock, flap relief or spoiler fault monitoring.
The screenshot's lower-right wheel is the pitch-trim/CG display. That feature
is not implemented. X-Plane provides gross CG in %MAC, but an A380-calibrated
THS model, trim schedule and its hydraulic/flight-phase validity still need
integration; the display must not invent a trim target from a generic ratio.

Extended-configuration VLS, F/S speeds, alpha protection/limit speeds and takeoff speeds require a
validated envelope provider. The optional `a380x/xp12/systems/pfd/envelope_valid`
input must be 1 before its associated speed values are accepted; absent,
nonfinite or inconsistent values stay absent. The provider is not installed
in the current aircraft. The native stall annunciator drives the STALL visual
cue, but it is not an Airbus alpha-floor calculation. Full low-speed protection,
alpha-floor/TOGA-lock control laws, FMS managed guidance, autoland capability,
complete ARINC redundancy and flight-envelope dynamics remain unimplemented.
This update must not be described as exact real-Airbus system equivalence.

## Evidence

The build runs strict C/C++ tests and verifies raw input names against the
installed X-Plane `DataRefs.txt`, then builds macOS and Windows binaries.
The build report records source and binary hashes. PFD render regressions
cover the RA/V/S/Mach boundaries, FD direction, FMA flashing, configuration
movement, failure blanking and ILS validity. The Windows binary is compile
checked; native runtime testing is on macOS.

The runtime evidence is saved under `/Users/sava/Desktop/A380/proof/PFD-*`:

- `PFD-live-control-checks.json`: X-Plane 12.4.3 ground checks passed for
  configuration 2, full extension, retraction, spoiler arming and half
  deployment. It records actual deployment, targets, changing VMAX and
  restoration of controls. Both PFD channels were valid.
- `PFD-XPlane-visual-checks.json` and matching PNGs: actual cockpit captures
  of the ground display and paused, injected instrument values for descent
  amber, climb amber, climb green, overspeed stripes and excessive-attitude
  decluttering, corrected left/right bank and the clean VLS marker (202.4 kt
  in the test aircraft state). Injection checks the installed renderer and sensor wiring;
  it is not a flight-dynamics or protection-law test. Instrument values and
  pause state were restored afterwards.

Offline SVGs in the build directories are renderer fixtures, not simulator
screenshots. Each runtime report identifies its tested binary separately.
The final installed and tested build is `build/pfd-native-20261006-e`.
Earlier captures retained in `proof/PFD-interim-d` document the bank-direction
issue found during verification and are not the final result. Camera settings
were restored after the final captures.

Sources: pinned [FlyByWire A380 PFD](https://github.com/flybywiresim/aircraft/tree/2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2/fbw-a380x/src/systems/instruments/src/PFD),
[A380 PFD overview](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/main-panel/pfd/),
Airbus [speed envelope](https://safetyfirst.airbus.com/control-your-speed-in-cruise/)
and [configuration limits](https://safetyfirst.airbus.com/control-your-speed-during-descent-approach-and-landing/).

## Speed scale envelope (VMAX, VSW)

Without an aircraft-systems envelope provider (`a380x/xp12/systems/pfd/*`), the plugin computes the
speed-scale limits itself (`a380x_pfd_inputs.inc`) and sends them to the FBW PFD as PRIM words:

* **VMAX** (`L:A32NX_PRIM_n_V_MAX`, red/black overspeed barber pole): VMO/MMO, the VFE of the
  current configuration (FBW SFCC) and the landing-gear limits (250 kt / M.55). Always valid when
  known; it no longer depends on the provider.
* **VSW** (`L:A32NX_PRIM_n_V_STALL_WARN`, red/black stall-warning barber pole). This port has no
  Airbus normal-law alpha protection, so the PFD shows VSW (as in the laws without protection)
  instead of the alpha prot / alpha max strips (those stay provider-only). VS1g comes from the FBW
  A380 operating-speed tables at the aircraft's actual mass and flap lever (CG 30 %MAC);
  VSW = max(1.05 VS1g, VS1g + 5 kt) x sqrt(nz), the CS 25.207(c) stall-warning margin raised with
  the load factor (nz 0.5-2.5, 1 on the ground). Examples (1 g): 450 t CONF FULL VS1g 120 / VSW 126 /
  VLS 148 kt; 450 t clean VS1g 163 / VSW 171 kt.
* The stall-warning aural uses the same VSW (docs/COCKPIT_AURALS.md).

The native (Windows) PFD draws the same VSW strip from `pfd_guidance.stall_warning_knots`.
