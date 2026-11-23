# ECAM System Display pages: live data

The lower ECAM (SD) renders FlyByWire's own A380X SD pages (React pages ENGINE ... CB, STATUS; xp-kit
"SDv2" FSComponent pages F/CTL and VIDEO) inside the aircraft-local WebKit helper
(`plugins/A380XAvionics/mac_x64/xp-kit/sd.html`). Until this change the bundle's SimVar shim knew
about 25 X-Plane datarefs, so almost every FBW variable was "missing": the bundle turned the green
numbers of any component with a missing variable into amber `XX`, and its own placeholder footer
printed an amber `XX / AMBER: SYSTEM DATA INCOMPLETE` (STATUS: `FULL FWS NOT AVAILABLE`) on every page,
so every page looked inoperative.

Now the native plugin publishes every variable the pages read, under its exact FBW name, computed
from what the port really simulates; the bundle reads those names first, decodes FBW ARINC 429 words,
feeds the SDv2 bus, and draws FBW's own permanent-data area instead of the placeholder footer.

| Part | File |
| --- | --- |
| Native publisher (XPLM data access only) | `native/avionics/a380x_sd_data.inc` |
| Frame transport, STATUS lists from the native FWS | `native/avionics/a380x_xpkit_sd.inc` (includes the publisher) |
| Bundle patch (applied in the packaging reuse path) | `tools/patch_xp_kit_sd.py`, hooked in `tools/package_xp_kit_cockpit.py` `reuse_verified_assets()` |
| Census data + classification + this document's tables | `tools/sd_census.json`, `tools/sd_pages_census.py` |
| Tests | `native/avionics/test_sd_data.c`, `tools/test_patch_xp_kit_sd.py`, `tools/test_sd_pages_census.py` |

## Data flow

```
X-Plane datarefs ─┐
native modules ───┤ (published datarefs: a380x/xp12/elec, apu, adirs, fms, avionics/flight_phase,
                  │  efis, hydraulics surface state; native FWS status lists, EWD idle N1)
                  ▼
a380x_sd_data.inc  sdd_emit()  ──► frame.vars["L:A32NX_..."], frame.vars["ev:<topic>"]
                  (inside xpkit_sd_publish: 0.25 s throttle, FNV hash dedupe, XPKIT_SD_CAP 64 KiB)
                  ▼  shared memory (A380XXpKitSd)
sd.html  Ol()  ── exact FBW name first, then the legacy X-Plane mapping
         I()   ── raw ARINC word decoded with its SSM (fbw-sdk Arinc429Word)
         lf()  ── every "ev:" key sent on the SDv2 FSComponent bus (F/CTL)
```

The publisher only uses XPLM data access (cached `XPLMFindDataRef`, re-found once per second while a
producer is absent), never other modules' globals, so it is tested with mocked datarefs. The full frame
is about 25 KB (the module's part is checked below 32 KiB by `test_sd_data`), well under
`XPKIT_SD_CAP`; the shared struct is unchanged.

**ARINC 429 words** use the FBW raw encoding (fbw-sdk `Arinc429Word.getRawWord`): the float32 bit
pattern of the value in the low 32 bits plus `SSM * 2^32`; discrete words carry the bit field as a
float. SSM: normal operation (3) when the source is valid and its computer powered; no computed data
(1) when the source has no value (no FMS destination, ADR not valid, no X-Plane source); failure
warning (0) when the computer is unpowered (no DC ESS/DC 1/DC 2, APU ECB unpowered). No failure is
ever invented: failure discretes are 0 unless a port failure input (for example
`a380x/xp12/ovhd/fail/prim2`) is set.

## Bundle patch (`tools/patch_xp_kit_sd.py`)

The JS build workspace no longer exists and there is no node toolchain, so the verified bundle is
patched in place with anchored string edits. The input must have a known sha256
(`369149262b09…`, the verified seven-view build) and every anchor must occur exactly the expected
number of times; the result carries the marker `/*A380X-SD-LIVE-DATA v1*/`, and a marked input is
returned byte-identical (idempotent). Unknown or partially patched bundles are rejected; a non-SD
fixture is left alone. Edits:

1. **Prelude** (before `function xe`): `__a380Raw(frame, name)` reads `frame.vars[name]` (or its
   upper-case form) when it is a finite number; `__a380Word(raw)` decodes a raw ARINC word
   (`value`, `ssm`, `isNormalOperation`, `isNoComputedData`, `isFailureWarning`, `valueOr`,
   `bitValueOr` with FBW semantics: NO or FT count as valid); CRUISE helpers.
2. **`Ol`** (the `useSimVar` / `SimVar.GetSimVarValue` shim): the exact FBW name wins, the existing
   X-Plane mappings remain the fallback.
3. **`I`** (`useArinc429Var` shim): a directly published word is decoded with its real SSM, so FBW's
   own invalid handling (`XX`, `--`, amber) applies and the component is not flagged "missing".
4. **SDv2 bridge**: `lf` calls the original body, then sends every `ev:<topic>` key (numbers and
   booleans) with `T0.send(topic, value)`; the ARINC subject shim `Dr` decodes raw words. The xp-kit
   build had flipped FBW's always-true F/CTL `infoAvailable` / `positionInfoValid` /
   `deflectionInfoValid` subjects to false ("X"); they now follow `ev:fctlInfoValid`, true when the
   native side delivers the surface data.
5. **CRUISE** (the bundle's native CRUISE page): FU per engine, FU TOTAL, LDG ELEVN and the upper/main
   deck zone ranges are read from the FBW variables with FBW `CruisePage` rounding (FU to 50 kg,
   LDG ELEV to 50 ft from FM1 then FM2 when normal operation).
6. **Lower area**: the placeholder footer (black box, amber `XX / AMBER: SYSTEM DATA INCOMPLETE` /
   `FULL FWS NOT AVAILABLE`, `<PAGE> · AIRCRAFT LIVE INPUTS`) is replaced by FBW's SDv2
   `PermanentData` (`SDv2/StatusArea.tsx`) at FBW's coordinates and classes: separator lines at
   y 667/765 and x 257/512, TAT/SAT (sign prefix, amber `XX` when the ADR word is invalid), ISA when
   baro STD is selected, amber G LOAD outside 0.7..1.4 g, the UTC clock `HH:MM:` + `SS` + `GPS`,
   GW / GWCG / FOB (100 kg rounding, `--` cyan for no computed data). The ATC mailbox area below
   y 768 stays black (no datalink in the port).

The patch is applied by `reuse_verified_assets()` (the `build.py --xpkit --xpkit-assets` path) right
after the byte-verified copy; the staged `manifest.json` then records the patched hash in
`sd_html_sha256` and `asset_hashes["sd.html"]`, plus `sd_live_data_patch` (tool, version, input and
output hashes). `install_xp_kit_cockpit.py` verifies those hashes; after installation the next reuse
build finds the marker and keeps the bytes unchanged.

## STATUS page

The legacy React STATUS page reads `L:A32NX_SD_STATUS_INFO_LINE_1..5`,
`L:A32NX_SD_STATUS_INOP_ALL_LINE_1..10`, `L:A32NX_SD_STATUS_INOP_LDG_LINE_1..10`,
`L:A32NX_EWD_LIMITATIONS_ALL|LDG_LINE_1`, `L:A32NX_PFD_LIMITATIONS_LINE_1` (FBW 9-digit EcamMessages
codes, looked up in the bundle's own message tables) and `L:A32NX_FWS1|2_IS_HEALTHY`.
`a380x_xpkit_sd.inc` fills them from the native FWS (`fwsa_fws.status`: `information`,
`inop_sys_all_phases`, `inop_sys_appr_ldg`, first limitation ids; healthy = `fwsa_available()`).
With no item the page shows FBW's green `NORMAL`; without a running FWS FBW's own amber
`STATUS NOT AVAILABLE`. The `FULL FWS NOT AVAILABLE` text no longer exists.

## Derived values (no direct port source)

* **Engines.** X-Plane models two spools; its N2 is the HP core and is FBW's N3, exactly as FBW's
  FADEC does (`simN3 = simEngineN2`); FBW N2 = N3 + 0.7. ENGINE_STATE: starter running = 2
  STARTING, running = 1 ON, else 0 OFF (SHUTTING is not distinguished). Oil pressure uses FBW's
  `Polynomial_A380X::oilPressure(N3)`; oil quantity FBW's model (18.5 qt total, middle of FBW's
  17-20 qt, x X-Plane oil ratio x (1 - FBW oil gulp(thrust))). X-Plane has no vibration model: 0.0.
  ENG MODE SEL (not in the port's cockpit) reads IGN/START while a starter or igniter runs.
* **Pneumatics.** PRV = X-Plane engine bleed SOV and engine running, closed while the APU bleed valve
  is open (FBW). Regulated pressure = FBW PRV target 40 psi x X-Plane duct availability of that side;
  precooler outlet = FBW fan-air-valve setpoint 200 C while the PRV flows, else OAT; HP valve open
  below N3 80 % (FBW: IP pressure below 33.5 psi). X-Plane has two isolation valves: L, R live,
  centre = both. APU bleed container pressure: FBW PW980 (40 psi + 14.696 psia running).
* **Air conditioning.** Pack flow valves open with the pack pb, duct pressure and AC power; CPIOM B
  AGS/TCS/VCS words carry pack-operating, hot-air-valve and fan/isolation-valve bits (pbs not bound
  stay in FBW's default ON). The port has no cabin thermal model: every zone relaxes (first order,
  tau 10 min) to FBW's default selection (cabin/cockpit 24 C, cargo 15 C) while a pack operates,
  otherwise to OAT; pack outlet temperature shows the 24 C setpoint.
* **Hydraulics.** This ACF's hydraulic model runs at 3000 psi nominal; the A380 at 5000 psi (FBW
  5250 target, SD green above 2900). Displayed pressure = X-Plane x 5000/3000; pressure switches use
  FBW's hysteresis (on 3700, off 2900 psi); reservoir = X-Plane fluid ratio x FBW normal fill
  (12.0 / 12.7 gal), low below FBW's 5 l. Engine pump pbs follow X-Plane's engine pump switches;
  X-Plane electric pump 1/2 = GREEN A / YELLOW A.
* **Fuel.** The ACF has 8 tanks (747 layout); the A380 FQMS 11. Mapping: FEED 1/4 = main 1/4; FEED 2/3
  = main 2/3 up to FBW's feed capacity (7753.2 gal x 3.0387 kg/gal), the remainder is LEFT/RIGHT
  INNER (inner-to-feed transfer keeps feeds full); centre wing / 2 = LEFT/RIGHT MID; reserve 2/3 =
  LEFT/RIGHT OUTER; horizontal stab = TRIM. FQMS and both FQDC words carry the same value. Fuel used
  integrates FF over simulated time and resets on a ground start (FBW FADEC). Feed MAIN pumps = the
  bound FEED pbs (X-Plane engine fuel pumps) with AC; transfer pumps = the mapped X-Plane tank pump;
  STBY pumps are not modelled. LP valves = ENG MASTER (mixture) and fire pb; crossfeed = X-Plane
  crossfeed selector; APU valves = APU MASTER; the port has no A380 transfer/trim/jettison model, so
  those valves read closed (no transfer ever happens). No APU fuel-flow source: APU FU is NCD and the
  FUEL page totals the engines only (FBW's invalid path).
* **Wheel / doors.** Gear position per side = the lower of wing and body gear; gear doors share the
  gear animation, so they read open while that gear is in transit. Brake temperature = OAT + 1000 C x
  X-Plane `brake_absorbed_rat` (1.0 = X-Plane brake failure) of the wheel's gear. Doors use the same
  `door_open_ratio` indexes as the cockpit animation (M1L..M5R, U1L..U3R, cargo 17/18). Sliding
  windows are not animated: closed.
* **F/CTL.** Surfaces use the exterior surface state (`a380x/xp12/hydraulics/*`, ratio 1 = 30 deg;
  left ratio + = TE down, right + = TE up, which is FBW's normalised convention), with fallbacks to the
  A380XAileronDatarefs visual ratios and X-Plane's commanded ratios; spoilers 1-2/3-4/5-6/7-8 on the
  inboard/inner/middle/outer groups (deg / 50). THS = X-Plane trim ratio x ACF stab travel (radians,
  + UP). Rudder trim = X-Plane rudder trim x ACF trim limit x 30 deg (FBW + = LEFT). PRIM/SEC healthy
  with DC power and no port failure input; SEC 1 engages the rudder trim (SEC 3 if SEC 1 is lost).
* **Fixed controls.** Pushbuttons the port does not bind read FBW's normal position: RAM AIR off,
  MAN ALT / MAN V/S CTL AUTO, HYD electric pump pbs AUTO, FADEC GND PWR released, engine fire pbs in
  (unless `a380x/xp12/engine/fire_pushbutton` exists). "Never written by FBW" variables
  (`..._B0`, `REPORTED_BRAKE_TEMPERATURE_17..20/null`) are published as 0, which is what MSFS returns.

Not published (honest absence): the fuel pump `CIRCUIT CONNECTION ON:n` reads (no circuit model; the
bundle's `GetSimVarValue` returns undefined, so no pump fault is drawn).

## Census

Every variable read by the pages, with the class (see `tools/sd_pages_census.py`) and the port source.
`live` = direct reading (unit conversions only), `derived` = computed (above), `ncd` = published as
"no computed data", `missing` = not published, `bundle` = served by the bundle, `n/a` = dead FBW code.

<!-- sd-census:begin (generated by tools/sd_pages_census.py) -->

### Coverage

| Page | live | derived | ncd | missing | bundle | n/a | total |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ENGINE | 21 | 33 | 0 | 0 | 1 | 0 | 55 |
| APU | 19 | 5 | 1 | 0 | 1 | 0 | 26 |
| BLEED | 5 | 45 | 0 | 0 | 0 | 0 | 50 |
| COND | 0 | 40 | 0 | 0 | 1 | 0 | 41 |
| CAB PRESS | 45 | 31 | 0 | 0 | 0 | 0 | 76 |
| DOOR | 20 | 6 | 0 | 0 | 0 | 0 | 26 |
| ELEC AC | 63 | 4 | 0 | 0 | 0 | 0 | 67 |
| ELEC DC | 42 | 0 | 0 | 0 | 0 | 2 | 44 |
| FUEL | 19 | 88 | 1 | 21 | 2 | 0 | 131 |
| WHEEL | 6 | 26 | 0 | 0 | 0 | 0 | 32 |
| HYD | 14 | 46 | 0 | 0 | 0 | 0 | 60 |
| F/CTL | 35 | 12 | 0 | 0 | 0 | 0 | 47 |
| CRUISE | 21 | 31 | 0 | 0 | 2 | 0 | 54 |
| STATUS | 30 | 0 | 0 | 0 | 0 | 0 | 30 |
| PERMANENT DATA | 9 | 0 | 1 | 0 | 0 | 0 | 10 |
| **all** | 349 | 367 | 3 | 21 | 7 | 2 | 749 |

### ENGINE

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:XMLVAR_ENG_MODE_SEL` | useSimVar (@instruments/common/simVars) `Enum` | derived | no ENG MODE selector in the port: IGN/START (2) while a starter/igniter runs, else NORM (1) |
| `L:A32NX_ENGINE_STATE:1` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ENGINE_STATE:2` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ENGINE_STATE:3` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ENGINE_STATE:4` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `CONFIG_USING_METRIC_UNIT` | usePersistentSetting (NXDataStore setting, NOT a simvar) + NXUnits.metricWeight | bundle | flyPad setting shim in the bundle (metric, KG) |
| `L:A32NX_ENGINE_N2:1` | useSimVar (@instruments/common/simVars) `number` | derived | FBW FADEC: N3 + 0.7 (0 when stopped) |
| `L:A32NX_ENGINE_N3:1` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_PNEU_ENG_1_STARTER_VALVE_OPEN` | useSimVar (@instruments/common/simVars) `number` | live | `sim/flightmodel2/engines/starter_is_running` |
| `L:A32NX_OVHD_FADEC_1` | useSimVar (@instruments/common/simVars) `bool` | derived | FADEC GND PWR pb not bound: released (0) |
| `L:A32NX_FIRE_BUTTON_ENG1` | useSimVar (@instruments/common/simVars) `bool` | derived | `a380x/xp12/engine/fire_pushbutton` when a producer exists, else in (0) |
| `L:A32NX_ENGINE_FF:1` | useSimVar (@instruments/common/simVars) `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `TURB ENG VIBRATION:1` | useSimVar (@instruments/common/simVars) `number` | derived | X-Plane has no vibration model: 0.0 |
| `L:A32NX_ENGINE_OIL_QTY:1` | useSimVar (@instruments/common/simVars) `number` | derived | FBW: 18.5 qt total x X-Plane `oil_quantity_ratio` x (1 - FBW oil gulp(thrust)) |
| `GENERAL ENG OIL TEMPERATURE:1` | useSimVar (@instruments/common/simVars) `celsius` | live | `oil_temperature_deg_C` (F -> C when `acf_oilT_is_C` = 0) |
| `L:A32NX_ENGINE_N2:2` | useSimVar (@instruments/common/simVars) `number` | derived | FBW FADEC: N3 + 0.7 (0 when stopped) |
| `L:A32NX_ENGINE_N3:2` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_PNEU_ENG_2_STARTER_VALVE_OPEN` | useSimVar (@instruments/common/simVars) `number` | live | `sim/flightmodel2/engines/starter_is_running` |
| `L:A32NX_OVHD_FADEC_2` | useSimVar (@instruments/common/simVars) `bool` | derived | FADEC GND PWR pb not bound: released (0) |
| `L:A32NX_FIRE_BUTTON_ENG2` | useSimVar (@instruments/common/simVars) `bool` | derived | `a380x/xp12/engine/fire_pushbutton` when a producer exists, else in (0) |
| `L:A32NX_ENGINE_FF:2` | useSimVar (@instruments/common/simVars) `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `TURB ENG VIBRATION:2` | useSimVar (@instruments/common/simVars) `number` | derived | X-Plane has no vibration model: 0.0 |
| `L:A32NX_ENGINE_OIL_QTY:2` | useSimVar (@instruments/common/simVars) `number` | derived | FBW: 18.5 qt total x X-Plane `oil_quantity_ratio` x (1 - FBW oil gulp(thrust)) |
| `GENERAL ENG OIL TEMPERATURE:2` | useSimVar (@instruments/common/simVars) `celsius` | live | `oil_temperature_deg_C` (F -> C when `acf_oilT_is_C` = 0) |
| `L:A32NX_ENGINE_N2:3` | useSimVar (@instruments/common/simVars) `number` | derived | FBW FADEC: N3 + 0.7 (0 when stopped) |
| `L:A32NX_ENGINE_N3:3` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_PNEU_ENG_3_STARTER_VALVE_OPEN` | useSimVar (@instruments/common/simVars) `number` | live | `sim/flightmodel2/engines/starter_is_running` |
| `L:A32NX_OVHD_FADEC_3` | useSimVar (@instruments/common/simVars) `bool` | derived | FADEC GND PWR pb not bound: released (0) |
| `L:A32NX_FIRE_BUTTON_ENG3` | useSimVar (@instruments/common/simVars) `bool` | derived | `a380x/xp12/engine/fire_pushbutton` when a producer exists, else in (0) |
| `L:A32NX_ENGINE_FF:3` | useSimVar (@instruments/common/simVars) `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `TURB ENG VIBRATION:3` | useSimVar (@instruments/common/simVars) `number` | derived | X-Plane has no vibration model: 0.0 |
| `L:A32NX_ENGINE_OIL_QTY:3` | useSimVar (@instruments/common/simVars) `number` | derived | FBW: 18.5 qt total x X-Plane `oil_quantity_ratio` x (1 - FBW oil gulp(thrust)) |
| `GENERAL ENG OIL TEMPERATURE:3` | useSimVar (@instruments/common/simVars) `celsius` | live | `oil_temperature_deg_C` (F -> C when `acf_oilT_is_C` = 0) |
| `L:A32NX_ENGINE_N2:4` | useSimVar (@instruments/common/simVars) `number` | derived | FBW FADEC: N3 + 0.7 (0 when stopped) |
| `L:A32NX_ENGINE_N3:4` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_PNEU_ENG_4_STARTER_VALVE_OPEN` | useSimVar (@instruments/common/simVars) `number` | live | `sim/flightmodel2/engines/starter_is_running` |
| `L:A32NX_OVHD_FADEC_4` | useSimVar (@instruments/common/simVars) `bool` | derived | FADEC GND PWR pb not bound: released (0) |
| `L:A32NX_FIRE_BUTTON_ENG4` | useSimVar (@instruments/common/simVars) `bool` | derived | `a380x/xp12/engine/fire_pushbutton` when a producer exists, else in (0) |
| `L:A32NX_ENGINE_FF:4` | useSimVar (@instruments/common/simVars) `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `TURB ENG VIBRATION:4` | useSimVar (@instruments/common/simVars) `number` | derived | X-Plane has no vibration model: 0.0 |
| `L:A32NX_ENGINE_OIL_QTY:4` | useSimVar (@instruments/common/simVars) `number` | derived | FBW: 18.5 qt total x X-Plane `oil_quantity_ratio` x (1 - FBW oil gulp(thrust)) |
| `GENERAL ENG OIL TEMPERATURE:4` | useSimVar (@instruments/common/simVars) `celsius` | live | `oil_temperature_deg_C` (F -> C when `acf_oilT_is_C` = 0) |
| `L:A32NX_ENGINE_N1:1` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N1_percent` |
| `L:A32NX_ENGINE_N1:2` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N1_percent` |
| `L:A32NX_ENGINE_N1:3` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N1_percent` |
| `L:A32NX_ENGINE_N1:4` | useSimVar (@instruments/common/simVars) `number` | live | `sim/cockpit2/engine/indicators/N1_percent` |
| `L:A32NX_ENGINE_IDLE_N1` | useSimVar (@instruments/common/simVars) `number` | live | native EWD/FADEC idle N1 (`sda.warnings`) |
| `ENG OIL PRESSURE:1` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW Polynomial_A380X::oilPressure(N3) |
| `ENG OIL PRESSURE:2` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW Polynomial_A380X::oilPressure(N3) |
| `ENG OIL PRESSURE:3` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW Polynomial_A380X::oilPressure(N3) |
| `ENG OIL PRESSURE:4` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW Polynomial_A380X::oilPressure(N3) |
| `L:A32NX_PNEU_ENG_1_REGULATED_TRANSDUCER_PRESSURE` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_PNEU_ENG_2_REGULATED_TRANSDUCER_PRESSURE` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_PNEU_ENG_3_REGULATED_TRANSDUCER_PRESSURE` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_PNEU_ENG_4_REGULATED_TRANSDUCER_PRESSURE` | useSimVar (@instruments/common/simVars) `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |

### APU

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE` | useSimVar `Bool` | live | native APU ECB `a380x/xp12/apu/avail` |
| `L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON` | useSimVar `Bool` | live | native APU ECB `a380x/xp12/apu/master_on` |
| `L:A32NX_ELEC_APU_GEN_1_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_1_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_1_FREQUENCY` | useSimVar `hertz` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_CONTACTOR_990XS1_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `APU GENERATOR SWITCH:1` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/apu_gen_a\|b` |
| `L:A32NX_ELEC_APU_GEN_2_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_2_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_2_FREQUENCY` | useSimVar `hertz` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_CONTACTOR_990XS2_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `APU GENERATOR SWITCH:2` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/apu_gen_a\|b` |
| `L:A32NX_OVHD_PNEU_APU_BLEED_PB_IS_ON` | useSimVar `Bool` | live | `sim/cockpit2/bleedair/actuators/apu_bleed` (bound APU BLEED pb) |
| `L:A32NX_APU_BLEED_AIR_VALVE_OPEN` | useSimVar `Bool` | derived | FBW: valve opens with APU BLEED on and APU AVAIL |
| `L:A32NX_PNEU_APU_BLEED_CONTAINER_PRESSURE` | useSimVar `PSI` | derived | FBW PW980 model: 40 psi + ISA 14.696 psia running, 14.696 otherwise |
| `L:A32NX_ATT_HDG_SWITCHING_KNOB` | useSimVar `Position` | live | native ADIRS `captain\|first_officer_adr_source` (0 CAPT 3, 1 NORM, 2 F/O 3) |
| `L:A32NX_ADIRS_ADR_1_CORRECTED_AVERAGE_STATIC_PRESSURE` | useArinc429Var `number` | live | `sim/weather/aircraft/barometer_current_pas` / 100 (hPa), SSM from `a380x/xp12/adirs/adrN_valid` |
| `L:A32NX_ADIRS_ADR_3_CORRECTED_AVERAGE_STATIC_PRESSURE` | useArinc429Var `number` | live | `sim/weather/aircraft/barometer_current_pas` / 100 (hPa), SSM from `a380x/xp12/adirs/adrN_valid` |
| `L:A32NX_APU_N` | useArinc429Var `number` | live | `sim/cockpit2/electrical/APU_N1_percent`, SSM NO while the ECB is powered |
| `L:A32NX_APU_N2` | useArinc429Var `number` | derived | X-Plane APU is single-spool: N2 = N |
| `L:A32NX_APU_EGT` | useArinc429Var `number` | live | `sim/cockpit2/electrical/APU_EGT_c` |
| `L:A32NX_APU_EGT_WARNING` | useArinc429Var `number` | derived | FBW ECB: 900 C; 982 C while starting above FL250 |
| `L:A32NX_APU_FUEL_USED` | useArinc429Var `number` | ncd | X-Plane has no APU fuel-flow source: NCD (FUEL page totals engines only) |
| `L:A32NX_APU_LOW_FUEL_PRESSURE_FAULT` | useArinc429Var `number` | derived | no fault (the port has no APU fuel-pressure failure) |
| `L:A32NX_APU_FLAP_OPEN_PERCENTAGE` | useSimVar `Percent` | live | native APU ECB `a380x/xp12/apu/flap_open_ratio` x 100 |
| `CONFIG_USING_METRIC_UNIT` | NXDataStore.getSetting (GetStoredData key `${AIRCRAFT_PROJECT_PREFIX}_CONFIG_USING_METRIC_UNIT`) | bundle | flyPad setting shim in the bundle (metric, KG) |

### BLEED

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_COND_CPIOM_B1_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B2_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B3_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B4_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B1_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B2_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B3_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B4_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_APU_BLEED_AIR_VALVE_OPEN` | useSimVar `bool` | derived | FBW: valve opens with APU BLEED on and APU AVAIL |
| `L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON` | useSimVar `bool` | live | native APU ECB `a380x/xp12/apu/master_on` |
| `L:A32NX_PNEU_XBLEED_VALVE_L_OPEN` | useSimVar `bool` | live | `sim/cockpit2/bleedair/actuators/isol_valve_left\|right` |
| `L:A32NX_PNEU_XBLEED_VALVE_C_OPEN` | useSimVar `bool` | derived | X-Plane has two isolation valves: centre open when both are |
| `L:A32NX_PNEU_XBLEED_VALVE_R_OPEN` | useSimVar `bool` | live | `sim/cockpit2/bleedair/actuators/isol_valve_left\|right` |
| `L:A32NX_ENGINE_STATE:1` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_PNEU_ENG_1_PR_VALVE_OPEN` | useSimVar `bool` | derived | X-Plane `engine_bleed_sov` and engine running; FBW closes it while the APU bleed valve is open |
| `L:A32NX_PNEU_ENG_1_HP_VALVE_OPEN` | useSimVar `bool` | derived | PRV open and N3 < 80 % (FBW: HP stage while IP pressure < 33.5 psi) |
| `L:A32NX_PNEU_ENG_1_PRECOOLER_OUTLET_TEMPERATURE` | useSimVar `celsius` | derived | FBW fan-air-valve setpoint 200 C while the PRV flows, else OAT |
| `L:A32NX_PNEU_ENG_1_REGULATED_TRANSDUCER_PRESSURE` | useSimVar `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_ENGINE_STATE:2` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_PNEU_ENG_2_PR_VALVE_OPEN` | useSimVar `bool` | derived | X-Plane `engine_bleed_sov` and engine running; FBW closes it while the APU bleed valve is open |
| `L:A32NX_PNEU_ENG_2_HP_VALVE_OPEN` | useSimVar `bool` | derived | PRV open and N3 < 80 % (FBW: HP stage while IP pressure < 33.5 psi) |
| `L:A32NX_PNEU_ENG_2_PRECOOLER_OUTLET_TEMPERATURE` | useSimVar `celsius` | derived | FBW fan-air-valve setpoint 200 C while the PRV flows, else OAT |
| `L:A32NX_PNEU_ENG_2_REGULATED_TRANSDUCER_PRESSURE` | useSimVar `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_ENGINE_STATE:3` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_PNEU_ENG_3_PR_VALVE_OPEN` | useSimVar `bool` | derived | X-Plane `engine_bleed_sov` and engine running; FBW closes it while the APU bleed valve is open |
| `L:A32NX_PNEU_ENG_3_HP_VALVE_OPEN` | useSimVar `bool` | derived | PRV open and N3 < 80 % (FBW: HP stage while IP pressure < 33.5 psi) |
| `L:A32NX_PNEU_ENG_3_PRECOOLER_OUTLET_TEMPERATURE` | useSimVar `celsius` | derived | FBW fan-air-valve setpoint 200 C while the PRV flows, else OAT |
| `L:A32NX_PNEU_ENG_3_REGULATED_TRANSDUCER_PRESSURE` | useSimVar `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_ENGINE_STATE:4` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_PNEU_ENG_4_PR_VALVE_OPEN` | useSimVar `bool` | derived | X-Plane `engine_bleed_sov` and engine running; FBW closes it while the APU bleed valve is open |
| `L:A32NX_PNEU_ENG_4_HP_VALVE_OPEN` | useSimVar `bool` | derived | PRV open and N3 < 80 % (FBW: HP stage while IP pressure < 33.5 psi) |
| `L:A32NX_PNEU_ENG_4_PRECOOLER_OUTLET_TEMPERATURE` | useSimVar `celsius` | derived | FBW fan-air-valve setpoint 200 C while the PRV flows, else OAT |
| `L:A32NX_PNEU_ENG_4_REGULATED_TRANSDUCER_PRESSURE` | useSimVar `psi` | derived | FBW PRV target 40 psi x X-Plane duct `bleed_available_left\|right` |
| `L:A32NX_COND_PACK_1_FLOW_VALVE_1_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_1_FLOW_VALVE_2_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_2_FLOW_VALVE_1_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_2_FLOW_VALVE_2_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_PNEU_PACK_1_FLOW_VALVE_1_FLOW_RATE` | useSimVar `number` | derived | 0.5 kg/s x duct availability while open |
| `L:A32NX_PNEU_PACK_1_FLOW_VALVE_2_FLOW_RATE` | useSimVar `number` | derived | 0.5 kg/s x duct availability while open |
| `L:A32NX_PNEU_PACK_2_FLOW_VALVE_1_FLOW_RATE` | useSimVar `number` | derived | 0.5 kg/s x duct availability while open |
| `L:A32NX_PNEU_PACK_2_FLOW_VALVE_2_FLOW_RATE` | useSimVar `number` | derived | 0.5 kg/s x duct availability while open |
| `L:A32NX_OVHD_COND_RAM_AIR_PB_IS_ON` | useSimVar `bool` | derived | RAM AIR pb not bound: off |
| `L:A32NX_COND_PACK_1_OUTLET_TEMPERATURE` | useSimVar `celsius` | derived | no ACM model: FBW default zone selection 24 C |
| `L:A32NX_COND_FDAC_1_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_FDAC_1_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_PACK_2_OUTLET_TEMPERATURE` | useSimVar `celsius` | derived | no ACM model: FBW default zone selection 24 C |
| `L:A32NX_COND_FDAC_2_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_FDAC_2_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_LGCIU_1_LEFT_GEAR_COMPRESSED` | useSimVar `bool` | live | `sim/flightmodel2/gear/on_ground[1]` (left wing gear) |
| `L:A32NX_ADIRS_ADR_1_COMPUTED_AIRSPEED` | useArinc429Var `number` | live | native ADIRS `adr1_ias_knots`, SSM from `adr1_valid` |

### COND

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `CONFIG_USING_METRIC_UNIT` | usePersistentSetting | bundle | flyPad setting shim in the bundle (metric, KG) |
| `L:A32NX_COND_CPIOM_B1_VCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13-17 cargo isolation/extract fans and cabin fans ON with AC (pbs not bound); no fan/heater failure |
| `L:A32NX_COND_CPIOM_B2_VCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13-17 cargo isolation/extract fans and cabin fans ON with AC (pbs not bound); no fan/heater failure |
| `L:A32NX_COND_CPIOM_B3_VCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13-17 cargo isolation/extract fans and cabin fans ON with AC (pbs not bound); no fan/heater failure |
| `L:A32NX_COND_CPIOM_B4_VCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13-17 cargo isolation/extract fans and cabin fans ON with AC (pbs not bound); no fan/heater failure |
| `L:A32NX_COND_CPIOM_B1_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B2_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B3_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B4_TCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 15/16 hot-air valves open with their pack (HOT AIR pbs not bound: ON) |
| `L:A32NX_COND_CPIOM_B1_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B2_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B3_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B4_AGS_DISCRETE_WORD` | useArinc429Var `number` | derived | bits 13/14 pack 1/2 operating (pack pb, duct pressure, AC); SSM NO with DC power |
| `L:A32NX_VENT_OVERPRESSURE_RELIEF_VALVE_IS_OPEN` | useSimVar `bool` | derived | closed (no overpressure model) |
| `L:A32NX_OVHD_COND_RAM_AIR_PB_IS_ON` | useSimVar `bool` | derived | RAM AIR pb not bound: off |
| `L:A32NX_VENT_FWD_VCM_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_VENT_FWD_VCM_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_VENT_AFT_VCM_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_VENT_AFT_VCM_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_CKPT_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_1_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_2_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_3_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_4_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_5_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_6_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_7_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_8_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_1_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_2_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_3_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_4_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_5_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_6_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_7_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_TADD_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_TADD_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_CARGO_FWD_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_CARGO_BULK_TEMP` | useSimVar `celsius` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_PACK_1_FLOW_VALVE_1_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_2_FLOW_VALVE_1_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |

### CAB PRESS

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_COND_CPIOM_B1_CPCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B2_CPCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B3_CPCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B4_CPCS_DISCRETE_WORD` | useArinc429Var `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_PRESS_MAN_CABIN_ALTITUDE` | useSimVar `feet` | live | X-Plane pressurization indicators (delta P in psi) |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B1` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B2` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B3` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B4` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_OVHD_PRESS_MAN_ALTITUDE_PB_IS_AUTO` | useSimVar `bool` | derived | MAN pbs not bound: AUTO |
| `L:A32NX_PRESS_MAN_CABIN_VS` | useSimVar `feet per minute` | live | X-Plane pressurization indicators (delta P in psi) |
| `L:A32NX_PRESS_CABIN_VS_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_VS_TARGET_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_VS_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_TARGET_B1` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_PRESS_CABIN_VS_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_TARGET_B2` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_PRESS_CABIN_VS_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_TARGET_B3` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_PRESS_CABIN_VS_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_TARGET_B4` | useArinc429Var `number` | live | X-Plane `pressurization/actuators/cabin_altitude_ft\|cabin_vvi_fpm` |
| `L:A32NX_OVHD_PRESS_MAN_VS_CTL_PB_IS_AUTO` | useSimVar `bool` | derived | MAN pbs not bound: AUTO |
| `L:A32NX_PRESS_MAN_CABIN_DELTA_PRESSURE` | useSimVar `feet` | live | X-Plane pressurization indicators (delta P in psi) |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_FM1_LANDING_ELEVATION` | useArinc429Var `number` | live | native FMS `a380x/xp12/fms/landing_elevation_ft` (NCD without a destination) |
| `L:A32NX_FWC_FLIGHT_PHASE` | useSimVar `enum` | live | native FWS flight phase `a380x/xp12/avionics/flight_phase` |
| `L:A32NX_PRESS_MAN_OUTFLOW_VALVE_1_OPEN_PERCENTAGE` | useSimVar `percent` | live | X-Plane `outflow_valve` x 100 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_1_OPEN_PERCENTAGE_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_1_OPEN_PERCENTAGE_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_1_OPEN_PERCENTAGE_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_1_OPEN_PERCENTAGE_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_1_OPEN_PERCENTAGE_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OCSM_1_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_1_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_1_AUTO_PARTITION_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_MAN_OUTFLOW_VALVE_2_OPEN_PERCENTAGE` | useSimVar `percent` | live | X-Plane `outflow_valve` x 100 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_2_OPEN_PERCENTAGE_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_2_OPEN_PERCENTAGE_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_2_OPEN_PERCENTAGE_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_2_OPEN_PERCENTAGE_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_2_OPEN_PERCENTAGE_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OCSM_2_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_2_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_2_AUTO_PARTITION_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_MAN_OUTFLOW_VALVE_3_OPEN_PERCENTAGE` | useSimVar `percent` | live | X-Plane `outflow_valve` x 100 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_3_OPEN_PERCENTAGE_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_3_OPEN_PERCENTAGE_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_3_OPEN_PERCENTAGE_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_3_OPEN_PERCENTAGE_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_3_OPEN_PERCENTAGE_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OCSM_3_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_3_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_3_AUTO_PARTITION_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_MAN_OUTFLOW_VALVE_4_OPEN_PERCENTAGE` | useSimVar `percent` | live | X-Plane `outflow_valve` x 100 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_4_OPEN_PERCENTAGE_B0` | useArinc429Var `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_OUTFLOW_VALVE_4_OPEN_PERCENTAGE_B1` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_4_OPEN_PERCENTAGE_B2` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_4_OPEN_PERCENTAGE_B3` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OUTFLOW_VALVE_4_OPEN_PERCENTAGE_B4` | useArinc429Var `number` | live | X-Plane `pressurization/indicators/outflow_valve` x 100 (one valve: all four) |
| `L:A32NX_PRESS_OCSM_4_CHANNEL_1_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_4_CHANNEL_2_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_PRESS_OCSM_4_AUTO_PARTITION_FAILURE` | useSimVar `bool` | derived | no failure (not simulated; never invented) |
| `L:A32NX_COND_PACK_1_FLOW_VALVE_1_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_1_FLOW_VALVE_2_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_2_FLOW_VALVE_1_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |
| `L:A32NX_COND_PACK_2_FLOW_VALVE_2_IS_OPEN` | useSimVar `bool` | derived | pack pb (X-Plane `pack_left\|right`) with duct pressure and AC |

### DOOR

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:CPT_SLIDING_WINDOW` | useSimVar (@instruments/common/simVars) `number` | derived | sliding windows are not animated: closed |
| `L:FO_SLIDING_WINDOW` | useSimVar (@instruments/common/simVars) `number` | derived | sliding windows are not animated: closed |
| `LIGHT BEACON ON` | useSimVar (@instruments/common/simVars) `bool` | live | `sim/cockpit2/switches/beacon_on` |
| `L:A32NX_ENGINE_STATE:1` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ENGINE_STATE:2` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ENGINE_STATE:3` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ENGINE_STATE:4` | useSimVar (@instruments/common/simVars) `enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_FWD_DOOR_CARGO_LOCKED` | useSimVar (@instruments/common/simVars) `bool` | live | `door_open_ratio[17\|18]` < 0.01 |
| `L:A32NX_AFT_DOOR_CARGO_LOCKED` | useSimVar (@instruments/common/simVars) `bool` | live | `door_open_ratio[17\|18]` < 0.01 |
| `INTERACTIVE POINT OPEN:0` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:1` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:2` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:3` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:4` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:5` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:6` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:7` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:8` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:9` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:10` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:11` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:12` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:13` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:14` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `INTERACTIVE POINT OPEN:15` | useSimVar (@flybywiresim/fbw-sdk-react) `percent` | live | `sim/flightmodel2/misc/door_open_ratio[n]` x 100 (cockpit door animation indexes) |
| `L:PUSH_OVHD_OXYGEN_CREW` | useSimVar (@instruments/common/simVars) `boolean` | live | `sim/cockpit2/oxygen/actuators/o2_valve_on` (pb pushed = OFF) |

### ELEC AC

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_ELEC_CONTACTOR_980XU5_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_980XU6_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_980XU1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_980XU2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_980XU3_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_980XU4_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_900XU_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_990XS1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_990XS2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_3XC1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_3XC2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_3XB.1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_AC_1_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_2_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_3_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_4_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_APU_GEN_1_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_1_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_1_FREQUENCY` | useSimVar `hertz` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `APU GENERATOR SWITCH:1` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/apu_gen_a\|b` |
| `L:A32NX_ELEC_APU_GEN_2_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_2_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `L:A32NX_ELEC_APU_GEN_2_FREQUENCY` | useSimVar `hertz` | live | native A380 network `a380x/xp12/elec/apu_gen/potential\|freq\|load_pct[n]` |
| `APU GENERATOR SWITCH:2` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/apu_gen_a\|b` |
| `L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON` | useSimVar `Bool` | live | native APU ECB `a380x/xp12/apu/master_on` |
| `L:A32NX_ELEC_EMER_GEN_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/ext\|emer_gen\|stat_inv/...` |
| `L:A32NX_ELEC_EMER_GEN_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/ext\|emer_gen\|stat_inv/...` |
| `L:A32NX_ELEC_CONTACTOR_5XE_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_RAT_STOW_POSITION` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/rat/stow_ratio` |
| `L:A32NX_ELEC_ENG_GEN_1_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_ENG_GEN_1_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_CONTACTOR_990XU1_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `GENERAL ENG MASTER ALTERNATOR:1` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/genN` |
| `L:A32NX_ENGINE_STATE:1` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ELEC_ENG_GEN_2_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_ENG_GEN_2_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_CONTACTOR_990XU2_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `GENERAL ENG MASTER ALTERNATOR:2` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/genN` |
| `L:A32NX_ENGINE_STATE:2` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ELEC_ENG_GEN_3_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_ENG_GEN_3_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_CONTACTOR_990XU3_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `GENERAL ENG MASTER ALTERNATOR:3` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/genN` |
| `L:A32NX_ENGINE_STATE:3` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ELEC_ENG_GEN_4_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_ENG_GEN_4_LOAD` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/gen/potential\|load_pct[n]` |
| `L:A32NX_ELEC_CONTACTOR_990XU4_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `GENERAL ENG MASTER ALTERNATOR:4` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/pb/genN` |
| `L:A32NX_ENGINE_STATE:4` | useSimVar `Enum` | derived | X-Plane starter running -> 2 STARTING, ENGN_running -> 1 ON, else 0 OFF |
| `L:A32NX_ELEC_EXT_PWR_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/ext\|emer_gen\|stat_inv/...` |
| `L:A32NX_ELEC_EXT_PWR_FREQUENCY` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/ext\|emer_gen\|stat_inv/...` |
| `L:A32NX_EXT_PWR_AVAIL:1` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/ext/availN` (GPU connected) |
| `L:A32NX_EXT_PWR_AVAIL:2` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/ext/availN` (GPU connected) |
| `L:A32NX_EXT_PWR_AVAIL:3` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/ext/availN` (GPU connected) |
| `L:A32NX_EXT_PWR_AVAIL:4` | useSimVar `number` | live | ELEC panel `a380x/xp12/elec/ext/availN` (GPU connected) |
| `L:A32NX_LGCIU_1_NOSE_GEAR_COMPRESSED` | useSimVar `number` | live | `sim/flightmodel2/gear/on_ground[0]` |
| `L:A32NX_LGCIU_2_NOSE_GEAR_COMPRESSED` | useSimVar `number` | live | `sim/flightmodel2/gear/on_ground[0]` |
| `L:A32NX_ELEC_CONTACTOR_990XG1_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_990XG2_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_990XG3_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_990XG4_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_STAT_INV_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/ext\|emer_gen\|stat_inv/...` |
| `L:A32NX_ELEC_STAT_INV_FREQUENCY` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/ext\|emer_gen\|stat_inv/...` |
| `L:A32NX_ELEC_CONTACTOR_7XB_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_3XB.2_IS_CLOSED` | useSimVar `number` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |

### ELEC DC

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_ELEC_BAT_1_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_ELEC_BAT_1_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO` | useSimVar `bool` | live | ELEC panel `a380x/xp12/elec/pb/bat1\|bat2\|bat_ess\|bat_apu` |
| `L:A32NX_ELEC_CONTACTOR_990PB1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_6PC1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_BAT_2_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_ELEC_BAT_2_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_OVHD_ELEC_BAT_2_PB_IS_AUTO` | useSimVar `bool` | live | ELEC panel `a380x/xp12/elec/pb/bat1\|bat2\|bat_ess\|bat_apu` |
| `L:A32NX_ELEC_CONTACTOR_990PB2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_BAT_3_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_ELEC_BAT_3_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_OVHD_ELEC_BAT_ESS_PB_IS_AUTO` | useSimVar `bool` | live | ELEC panel `a380x/xp12/elec/pb/bat1\|bat2\|bat_ess\|bat_apu` |
| `L:A32NX_ELEC_CONTACTOR_6PB3_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_BAT_4_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_ELEC_BAT_4_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/bat/potential\|current[n]` (1, 2, ESS, APU) |
| `L:A32NX_OVHD_ELEC_BAT_APU_PB_IS_AUTO` | useSimVar `bool` | live | ELEC panel `a380x/xp12/elec/pb/bat1\|bat2\|bat_ess\|bat_apu` |
| `L:A32NX_ELEC_CONTACTOR_5PB_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_DC_1_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_DC_2_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_309PP_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_CONTACTOR_980PC_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_6PC2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_990PU1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_TR_1_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_TR_1_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_AC_2_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_CONTACTOR_990PU2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_TR_2_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_TR_2_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_AC_3_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_CONTACTOR_6PE_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_TR_3_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_TR_3_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_CONTACTOR_7PU_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_TR_4_POTENTIAL` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_TR_4_CURRENT` | useSimVar `volts` | live | native A380 network `a380x/xp12/elec/tr/potential\|current[n]` (TR 1, TR 2, ESS TR, APU TR) |
| `L:A32NX_ELEC_AC_4_BUS_IS_POWERED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_CONTACTOR_3XC1_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_3XC2_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_CONTACTOR_5XE_IS_CLOSED` | useSimVar `bool` | live | native A380 network `a380x/xp12/elec/contactor/<id>` (`.` written `_`) |
| `L:A32NX_ELEC_${simVarString}_POTENTIAL` | useSimVar `volts` | n/a | IndicationBox.tsx is never imported (dead FBW code) |
| `L:A32NX_ELEC_${simVarString}_CURRENT` | useSimVar `volts` | n/a | IndicationBox.tsx is never imported (dead FBW code) |

### FUEL

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_FUEL_USED:1` | useSimVar `kg` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_FUEL_USED:2` | useSimVar `kg` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_FUEL_USED:3` | useSimVar `kg` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_FUEL_USED:4` | useSimVar `kg` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_APU_FUEL_USED` | useArinc429Var `number` | ncd | X-Plane has no APU fuel-flow source: NCD (FUEL page totals engines only) |
| `L:A32NX_ENGINE_FF:1` | useSimVar `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `L:A32NX_ENGINE_FF:2` | useSimVar `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `L:A32NX_ENGINE_FF:3` | useSimVar `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `L:A32NX_ENGINE_FF:4` | useSimVar `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `FUELSYSTEM VALVE OPEN:1` | useSimVar `Percent over 100` | live | LP valve = ENG MASTER (`mixture_ratio` >= 0.5) and fire pb in |
| `FUELSYSTEM VALVE OPEN:2` | useSimVar `Percent over 100` | live | LP valve = ENG MASTER (`mixture_ratio` >= 0.5) and fire pb in |
| `FUELSYSTEM VALVE OPEN:3` | useSimVar `Percent over 100` | live | LP valve = ENG MASTER (`mixture_ratio` >= 0.5) and fire pb in |
| `FUELSYSTEM VALVE OPEN:4` | useSimVar `Percent over 100` | live | LP valve = ENG MASTER (`mixture_ratio` >= 0.5) and fire pb in |
| `L:A32NX_FQMS_LEFT_FUEL_PUMP_RUNNING_WORD` | useArinc429Var `number` | derived | feed MAIN = X-Plane engine `fuel_pump_on` (bound FEED pbs) with AC; transfer pumps = mapped X-Plane tank pump; STBY not modelled |
| `L:A32NX_FQMS_RIGHT_FUEL_PUMP_RUNNING_WORD` | useArinc429Var `number` | derived | feed MAIN = X-Plane engine `fuel_pump_on` (bound FEED pbs) with AC; transfer pumps = mapped X-Plane tank pump; STBY not modelled |
| `BUS LOOKUP INDEX` | SimVar.SetSimVarValue `Number` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:2` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:3` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:64` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:65` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:66` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:67` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:68` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:69` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:70` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:71` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:72` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:73` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:78` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:75` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:76` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:77` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:74` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:79` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:80` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `CIRCUIT CONNECTION ON:81` | SimVar.GetSimVarValue `Bool` | missing | no pump circuit model: GetSimVarValue returns undefined, so no pump fault is drawn |
| `FUELSYSTEM VALVE OPEN:46` | useSimVar `Percent over 100` | live | crossfeed = X-Plane `fuel_crossfeed_selector` != 0 |
| `FUELSYSTEM VALVE OPEN:47` | useSimVar `Percent over 100` | live | crossfeed = X-Plane `fuel_crossfeed_selector` != 0 |
| `FUELSYSTEM VALVE OPEN:48` | useSimVar `Percent over 100` | live | crossfeed = X-Plane `fuel_crossfeed_selector` != 0 |
| `FUELSYSTEM VALVE OPEN:49` | useSimVar `Percent over 100` | live | crossfeed = X-Plane `fuel_crossfeed_selector` != 0 |
| `FUELSYSTEM VALVE OPEN:52` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:53` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:56` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:54` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:55` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:5` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:11` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:6` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:8` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:12` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:14` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:7` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:9` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:13` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:15` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:10` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:16` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:17` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:18` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:19` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:20` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:21` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:22` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:23` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:27` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:24` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:28` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:25` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:29` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:26` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:30` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:33` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:39` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:32` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:38` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:31` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:37` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:34` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:40` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:35` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:41` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:36` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:42` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:43` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:59` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:44` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:45` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:57` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `FUELSYSTEM VALVE OPEN:58` | useSimVar `Percent over 100` | derived | no A380 transfer/trim/jettison model: closed |
| `L:A32NX_FQMS_LEFT_OUTER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_LEFT_OUTER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_LEFT_OUTER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_FEED_1_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_FEED_1_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_FEED_1_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_LEFT_MID_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_LEFT_MID_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_LEFT_MID_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_LEFT_INNER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_LEFT_INNER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_LEFT_INNER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_FEED_2_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_FEED_2_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_FEED_2_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_FEED_3_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_FEED_3_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_FEED_3_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_RIGHT_INNER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_RIGHT_INNER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_RIGHT_INNER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_RIGHT_MID_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_RIGHT_MID_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_RIGHT_MID_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_FEED_4_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_FEED_4_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_FEED_4_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_RIGHT_OUTER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_RIGHT_OUTER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_RIGHT_OUTER_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQMS_TRIM_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_1_TRIM_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_FQDC_2_TRIM_TANK_QUANTITY` | useArinc429Var `number` | derived | ACF tanks -> A380 tanks: feed 1/4 = main 1/4, feed 2/3 = main 2/3 up to the FBW feed capacity, rest inner; centre wing / 2 = mids; reserves = outers; stab = trim |
| `L:A32NX_ENGINE_N3:1` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_ENGINE_N3:2` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_ENGINE_N3:3` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_ENGINE_N3:4` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON` | useSimVar `Bool` | live | native APU ECB `a380x/xp12/apu/master_on` |
| `FUELSYSTEM VALVE OPEN:50` | useSimVar `Percent over 100` | live | APU isolation/LP valve = APU MASTER (FBW hack keeps them tied) |
| `FUELSYSTEM VALVE OPEN:51` | useSimVar `Percent over 100` | live | APU isolation/LP valve = APU MASTER (FBW hack keeps them tied) |
| `CONFIG_USING_METRIC_UNIT` | NXDataStore.getSetting | bundle | flyPad setting shim in the bundle (metric, KG) |
| `L:A32NX_ECAM_SD_PAGE_TO_SHOW` | useSimVar `number` | bundle | frame `page` (SD page controller) |

### WHEEL

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_GEAR_CENTER_POSITION` | useSimVar (@flybywiresim/fbw-sdk-react) `number` | live | `sim/flightmodel2/gear/deploy_ratio` x 100 (side = min of wing and body gear) |
| `L:A32NX_GEAR_LEFT_POSITION` | useSimVar (@flybywiresim/fbw-sdk-react) `number` | live | `sim/flightmodel2/gear/deploy_ratio` x 100 (side = min of wing and body gear) |
| `L:A32NX_GEAR_RIGHT_POSITION` | useSimVar (@flybywiresim/fbw-sdk-react) `number` | live | `sim/flightmodel2/gear/deploy_ratio` x 100 (side = min of wing and body gear) |
| `L:A32NX_GEAR_DOOR_CENTER_POSITION` | useSimVar (@flybywiresim/fbw-sdk-react) `number` | derived | doors share the gear animation: 100 while that gear is in transit, else 0 |
| `L:A32NX_GEAR_DOOR_LEFT_POSITION` | useSimVar (@flybywiresim/fbw-sdk-react) `number` | derived | doors share the gear animation: 100 while that gear is in transit, else 0 |
| `L:A32NX_GEAR_DOOR_RIGHT_POSITION` | useSimVar (@flybywiresim/fbw-sdk-react) `number` | derived | doors share the gear animation: 100 while that gear is in transit, else 0 |
| `L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH` | useSimVar (@flybywiresim/fbw-sdk-react) `boolean` | derived | FBW pressure switch: on >= 3700 psi, off < 2900 psi |
| `L:A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH` | useSimVar (@flybywiresim/fbw-sdk-react) `boolean` | derived | FBW pressure switch: on >= 3700 psi, off < 2900 psi |
| `L:A32NX_ELEC_AC_2_BUS_IS_POWERED` | useSimVar (@flybywiresim/fbw-sdk-react) `boolean` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_3_BUS_IS_POWERED` | useSimVar (@flybywiresim/fbw-sdk-react) `boolean` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_ELEC_AC_4_BUS_IS_POWERED` | useSimVar (@flybywiresim/fbw-sdk-react) `boolean` | live | native A380 network `a380x/xp12/elec/bus/<bus>_powered` (FBW AC_ESS_SHED = AC ESS 400XP, AC_ESS = AC EMER 491XP, 247XP = AC EHA, 309PP = DC APU) |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_1` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_2` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_3` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_4` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_5` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_6` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_7` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_8` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_9` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_10` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_11` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_12` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_13` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_14` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_15` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_16` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | OAT + 1000 C x X-Plane `brake_absorbed_rat` of the wheel group |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_17` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | never written by FBW (MSFS unset LVar = 0): 0 |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_18` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | never written by FBW (MSFS unset LVar = 0): 0 |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_19` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | never written by FBW (MSFS unset LVar = 0): 0 |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_20` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | never written by FBW (MSFS unset LVar = 0): 0 |
| `L:A32NX_REPORTED_BRAKE_TEMPERATURE_null` | useSimVar (@flybywiresim/fbw-sdk-react) `celsius` | derived | never written by FBW (MSFS unset LVar = 0): 0 |

### HYD

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_ENGINE_N3:1` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_HYD_ENG_1AB_PUMP_DISC` | useSimVar `boolean` | derived | no disconnection (not simulated) |
| `L:A32NX_ENGINE_N3:2` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_HYD_ENG_2AB_PUMP_DISC` | useSimVar `boolean` | derived | no disconnection (not simulated) |
| `L:A32NX_ENGINE_N3:3` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_HYD_ENG_3AB_PUMP_DISC` | useSimVar `boolean` | derived | no disconnection (not simulated) |
| `L:A32NX_ENGINE_N3:4` | useSimVar `number` | live | `sim/cockpit2/engine/indicators/N2_percent` (X-Plane HP core = FBW N3, as in FBW FADEC) |
| `L:A32NX_HYD_ENG_4AB_PUMP_DISC` | useSimVar `boolean` | derived | no disconnection (not simulated) |
| `L:A32NX_HYD_GREEN_PUMP_1_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_1A_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_GREEN_PUMP_2_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_1B_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_GREEN_PUMP_1_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_GREEN_PUMP_2_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_GREEN_PUMP_3_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_2A_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_GREEN_PUMP_4_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_2B_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_GREEN_PUMP_3_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_GREEN_PUMP_4_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_YELLOW_PUMP_1_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_3A_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_YELLOW_PUMP_2_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_3B_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_YELLOW_PUMP_1_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_YELLOW_PUMP_2_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_YELLOW_PUMP_3_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_4A_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_YELLOW_PUMP_4_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | engine pump on, N3 > 50 % and system switch |
| `L:A32NX_OVHD_HYD_ENG_4B_PUMP_PB_IS_AUTO` | useSimVar `boolean` | live | X-Plane `hydraulics/actuators/engine_pump\|engine_pumpA\|B[n]` |
| `L:A32NX_HYD_YELLOW_PUMP_3_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_YELLOW_PUMP_4_FIRE_VALVE_OPENED` | useSimVar `boolean` | derived | open unless the engine fire pb is released |
| `L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE` | useSimVar `psi` | derived | X-Plane `hydraulic_pressure_1\|2` x 5000/3000 (ACF 3000 psi, A380 5000 psi) |
| `L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | FBW pressure switch: on >= 3700 psi, off < 2900 psi |
| `L:A32NX_HYD_GREEN_RESERVOIR_LEVEL` | useSimVar `gallon` | derived | X-Plane `hydraulic_fluid_ratio_1\|2` x FBW normal fill (12.0 / 12.7 gal) |
| `L:A32NX_HYD_GREEN_RESERVOIR_LEVEL_IS_LOW` | useSimVar `boolean` | derived | level below FBW 5 l |
| `L:A32NX_HYD_GREEN_RESERVOIR_AIR_PRESSURE_IS_LOW` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_GREEN_RESERVOIR_OVHT` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_GREEN_PUMP_5_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | electric pump active and system switch |
| `L:A32NX_HYD_GREEN_PUMP_6_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | electric pump active and system switch |
| `L:A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE` | useSimVar `psi` | derived | X-Plane `hydraulic_pressure_1\|2` x 5000/3000 (ACF 3000 psi, A380 5000 psi) |
| `L:A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | FBW pressure switch: on >= 3700 psi, off < 2900 psi |
| `L:A32NX_HYD_YELLOW_RESERVOIR_LEVEL` | useSimVar `gallon` | derived | X-Plane `hydraulic_fluid_ratio_1\|2` x FBW normal fill (12.0 / 12.7 gal) |
| `L:A32NX_HYD_YELLOW_RESERVOIR_LEVEL_IS_LOW` | useSimVar `boolean` | derived | level below FBW 5 l |
| `L:A32NX_HYD_YELLOW_RESERVOIR_AIR_PRESSURE_IS_LOW` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_YELLOW_RESERVOIR_OVHT` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_YELLOW_PUMP_5_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | electric pump active and system switch |
| `L:A32NX_HYD_YELLOW_PUMP_6_SECTION_PRESSURE_SWITCH` | useSimVar `boolean` | derived | electric pump active and system switch |
| `L:A32NX_HYD_GA_EPUMP_ACTIVE` | useSimVar `boolean` | live | X-Plane `electric_hydraulic_pump_on` (GREEN A) / `pump2_on` (YELLOW A), AC powered |
| `L:A32NX_OVHD_HYD_EPUMPGA_OFF_PB_IS_AUTO` | useSimVar `boolean` | derived | pbs not bound: AUTO |
| `L:A32NX_HYD_GA_EPUMP_OVHT` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_GB_EPUMP_ACTIVE` | useSimVar `boolean` | derived | X-Plane has one electric pump per system: B pumps off |
| `L:A32NX_OVHD_HYD_EPUMPGB_OFF_PB_IS_AUTO` | useSimVar `boolean` | derived | pbs not bound: AUTO |
| `L:A32NX_HYD_GB_EPUMP_OVHT` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_YA_EPUMP_ACTIVE` | useSimVar `boolean` | live | X-Plane `electric_hydraulic_pump_on` (GREEN A) / `pump2_on` (YELLOW A), AC powered |
| `L:A32NX_OVHD_HYD_EPUMPYA_OFF_PB_IS_AUTO` | useSimVar `boolean` | derived | pbs not bound: AUTO |
| `L:A32NX_HYD_YA_EPUMP_OVHT` | useSimVar `boolean` | derived | no failure (not simulated) |
| `L:A32NX_HYD_YB_EPUMP_ACTIVE` | useSimVar `boolean` | derived | X-Plane has one electric pump per system: B pumps off |
| `L:A32NX_OVHD_HYD_EPUMPYB_OFF_PB_IS_AUTO` | useSimVar `boolean` | derived | pbs not bound: AUTO |
| `L:A32NX_HYD_YB_EPUMP_OVHT` | useSimVar `boolean` | derived | no failure (not simulated) |

### F/CTL

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `lgciu1LeftGearCompressed` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_LGCIU_1_LEFT_GEAR_COMPRESSED) | live | `sim/flightmodel2/gear/on_ground[1]` |
| `prim1Healthy` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_PRIM_1_HEALTHY) | derived | DC power and no `a380x/xp12/ovhd/fail/primN\|secN` input |
| `prim2Healthy` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_PRIM_2_HEALTHY) | derived | DC power and no `a380x/xp12/ovhd/fail/primN\|secN` input |
| `prim3Healthy` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_PRIM_3_HEALTHY) | derived | DC power and no `a380x/xp12/ovhd/fail/primN\|secN` input |
| `sec1Healthy` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_SEC_1_HEALTHY) | derived | DC power and no `a380x/xp12/ovhd/fail/primN\|secN` input |
| `sec2Healthy` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_SEC_2_HEALTHY) | derived | DC power and no `a380x/xp12/ovhd/fail/primN\|secN` input |
| `sec3Healthy` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_SEC_3_HEALTHY) | derived | DC power and no `a380x/xp12/ovhd/fail/primN\|secN` input |
| `thsDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number') -> MSFS base unit RADIANS` (A:ELEVATOR TRIM POSITION) | live | `sim/flightmodel2/controls/elevator_trim` x ACF stab trim travel (radians, + UP) |
| `greenPressureSwitch` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH) | derived | the HYD system pressure switch above |
| `yellowPressureSwitch` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH) | derived | the HYD system pressure switch above |
| `acEssPowered` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED) | live | native A380 network `a380x/xp12/elec/bus/...` (acEss = FBW AC_ESS = AC EMER 491XP) |
| `ac1Powered` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_ELEC_AC_1_BUS_IS_POWERED) | live | native A380 network `a380x/xp12/elec/bus/...` (acEss = FBW AC_ESS = AC EMER 491XP) |
| `acEhaPowered` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_ELEC_247XP_BUS_IS_POWERED) | live | native A380 network `a380x/xp12/elec/bus/...` (acEss = FBW AC_ESS = AC EMER 491XP) |
| `leftOuterAileronDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_AILERON_LEFT_OUTWARD_DEFLECTION) | live | exterior surface state `a380x/xp12/hydraulics/aileron_*_ratio` (fallback `a380x/xp12/visual/...`) |
| `leftMiddleAileronDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_AILERON_LEFT_MIDDLE_DEFLECTION) | live | exterior surface state `a380x/xp12/hydraulics/aileron_*_ratio` (fallback `a380x/xp12/visual/...`) |
| `leftInnerAileronDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_AILERON_LEFT_INWARD_DEFLECTION) | live | exterior surface state `a380x/xp12/hydraulics/aileron_*_ratio` (fallback `a380x/xp12/visual/...`) |
| `rightOuterAileronDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_AILERON_RIGHT_OUTWARD_DEFLECTION) | live | exterior surface state `a380x/xp12/hydraulics/aileron_*_ratio` (fallback `a380x/xp12/visual/...`) |
| `rightMiddleAileronDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_AILERON_RIGHT_MIDDLE_DEFLECTION) | live | exterior surface state `a380x/xp12/hydraulics/aileron_*_ratio` (fallback `a380x/xp12/visual/...`) |
| `rightInnerAileronDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_AILERON_RIGHT_INWARD_DEFLECTION) | live | exterior surface state `a380x/xp12/hydraulics/aileron_*_ratio` (fallback `a380x/xp12/visual/...`) |
| `leftOuterElevatorDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_ELEVATOR_LEFT_OUTWARD_DEFLECTION) | live | `a380x/xp12/hydraulics/elevator_ratio` (fallback `pitch_ratio`) |
| `leftInnerElevatorDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_ELEVATOR_LEFT_INWARD_DEFLECTION) | live | `a380x/xp12/hydraulics/elevator_ratio` (fallback `pitch_ratio`) |
| `rightInnerElevatorDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_ELEVATOR_RIGHT_INWARD_DEFLECTION) | live | `a380x/xp12/hydraulics/elevator_ratio` (fallback `pitch_ratio`) |
| `rightOuterElevatorDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_ELEVATOR_RIGHT_OUTWARD_DEFLECTION) | live | `a380x/xp12/hydraulics/elevator_ratio` (fallback `pitch_ratio`) |
| `leftSpoiler1Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_1_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler2Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_2_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler3Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_3_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler4Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_4_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler5Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_5_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler6Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_6_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler7Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_7_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `leftSpoiler8Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_8_LEFT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler1Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_1_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler2Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_2_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler3Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_3_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler4Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_4_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler5Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_5_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler6Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_6_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler7Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_7_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `rightSpoiler8Deflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_SPOILER_8_RIGHT_DEFLECTION) | live | `a380x/xp12/hydraulics/spoiler_<side>_<group>_deg` / 50 (pairs 1-2 inboard .. 7-8 outer) |
| `upperRudderDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_UPPER_RUDDER_DEFLECTION) | live | `a380x/xp12/hydraulics/rudder_ratio` (fallback `heading_ratio`) |
| `lowerRudderDeflection` | EventBus topic (ConsumerSubject via SDSimvarPublisher) .atFrequency(10) `SimVarValueType.Number ('number')` (L:A32NX_HYD_LOWER_RUDDER_DEFLECTION) | live | `a380x/xp12/hydraulics/rudder_ratio` (fallback `heading_ratio`) |
| `sec1RudderStatusWord` | Arinc429LocalVarConsumerSubject (EventBus topic) `SimVarValueType.Enum ('enum')` (L:A32NX_SEC_1_RUDDER_STATUS_WORD) | derived | bit 27 trim available, bit 28 engaged (SEC 1 first, SEC 3 if SEC 1 lost) |
| `sec3RudderStatusWord` | Arinc429LocalVarConsumerSubject (EventBus topic) `SimVarValueType.Enum ('enum')` (L:A32NX_SEC_3_RUDDER_STATUS_WORD) | derived | bit 27 trim available, bit 28 engaged (SEC 1 first, SEC 3 if SEC 1 lost) |
| `sec1RudderTrimPosition` | Arinc429LocalVarConsumerSubject.setConsumer (EventBus topic, .atFrequency(10)) `SimVarValueType.Enum ('enum')` (L:A32NX_SEC_1_RUDDER_ACTUAL_POSITION) | derived | X-Plane `rudder_trim` x ACF trim limit x 30 deg (+ = LEFT) |
| `sec3RudderTrimPosition` | Arinc429LocalVarConsumerSubject.setConsumer (EventBus topic, .atFrequency(10)) `SimVarValueType.Enum ('enum')` (L:A32NX_SEC_3_RUDDER_ACTUAL_POSITION) | derived | X-Plane `rudder_trim` x ACF trim limit x 30 deg (+ = LEFT) |
| `dcEssPowered` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED) | live | native A380 network `a380x/xp12/elec/bus/...` (acEss = FBW AC_ESS = AC EMER 491XP) |
| `dc1Powered` | EventBus topic (ConsumerSubject via SDSimvarPublisher) `SimVarValueType.Bool ('bool')` (L:A32NX_ELEC_DC_1_BUS_IS_POWERED) | live | native A380 network `a380x/xp12/elec/bus/...` (acEss = FBW AC_ESS = AC EMER 491XP) |

### CRUISE

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_FUEL_USED:1` | bus:engineFuelUsed_1 `number` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_FUEL_USED:2` | bus:engineFuelUsed_2 `number` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_FUEL_USED:3` | bus:engineFuelUsed_3 `number` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_FUEL_USED:4` | bus:engineFuelUsed_4 `number` | derived | FBW FADEC: integral of FF over simulated time, reset by a ground start |
| `L:A32NX_ENGINE_FF:1` | bus:engineFuelFlow_1 `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `L:A32NX_ENGINE_FF:2` | bus:engineFuelFlow_2 `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `L:A32NX_ENGINE_FF:3` | bus:engineFuelFlow_3 `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `L:A32NX_ENGINE_FF:4` | bus:engineFuelFlow_4 `number` | live | `fuel_flow_kg_sec` x 3600 (kg/h) |
| `CONFIG_USING_METRIC_UNIT` | NXDataStore.getSetting | bundle | flyPad setting shim in the bundle (metric, KG) |
| `L:A32NX_FM1_LANDING_ELEVATION` | bus:fmLandingElevation_1 `number` | live | native FMS `a380x/xp12/fms/landing_elevation_ft` (NCD without a destination) |
| `L:A32NX_FM2_LANDING_ELEVATION` | bus:fmLandingElevation_2 `number` | live | native FMS `a380x/xp12/fms/landing_elevation_ft` (NCD without a destination) |
| `L:A32NX_OVHD_PRESS_MAN_ALTITUDE_PB_IS_AUTO` | bus:cabinAltitudeIsAuto `Bool` | derived | MAN pbs not bound: AUTO |
| `L:A32NX_OVHD_PRESS_MAN_VS_CTL_PB_IS_AUTO` | bus:cabinVerticalSpeedIsAuto `Bool` | derived | MAN pbs not bound: AUTO |
| `L:A32NX_PRESS_MAN_CABIN_ALTITUDE` | bus:manCabinAltitude `number` | live | X-Plane pressurization indicators (delta P in psi) |
| `L:A32NX_PRESS_MAN_CABIN_DELTA_PRESSURE` | bus:manCabinDeltaPressure `number` | live | X-Plane pressurization indicators (delta P in psi) |
| `L:A32NX_PRESS_MAN_CABIN_VS` | bus:manCabinVerticalSpeed `number` | live | X-Plane pressurization indicators (delta P in psi) |
| `L:A32NX_COND_CPIOM_B1_CPCS_DISCRETE_WORD` | bus:cpcsBxDiscreteWord_1 `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B2_CPCS_DISCRETE_WORD` | bus:cpcsBxDiscreteWord_2 `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B3_CPCS_DISCRETE_WORD` | bus:cpcsBxDiscreteWord_3 `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_COND_CPIOM_B4_CPCS_DISCRETE_WORD` | bus:cpcsBxDiscreteWord_4 `number` | derived | bit 11 CPCS fault clear; SSM NO with DC power |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B0` | bus:pressCabinAltitude_0 `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B0` | bus:pressCabinDeltaPressure_0 `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_VS_B0` | bus:pressCabinVerticalSpeed_0 `number` | derived | never written by FBW (MSFS unset LVar = 0): raw 0 |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B1` | bus:pressCabinAltitude_1 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B1` | bus:pressCabinDeltaPressure_1 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_B1` | bus:pressCabinVerticalSpeed_1 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B2` | bus:pressCabinAltitude_2 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B2` | bus:pressCabinDeltaPressure_2 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_B2` | bus:pressCabinVerticalSpeed_2 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B3` | bus:pressCabinAltitude_3 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B3` | bus:pressCabinDeltaPressure_3 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_B3` | bus:pressCabinVerticalSpeed_3 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_ALTITUDE_B4` | bus:pressCabinAltitude_4 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B4` | bus:pressCabinDeltaPressure_4 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_PRESS_CABIN_VS_B4` | bus:pressCabinVerticalSpeed_4 `number` | live | X-Plane `pressurization/indicators/cabin_altitude_ft\|cabin_vvi_fpm\|pressure_diffential_psi` |
| `L:A32NX_COND_CKPT_TEMP` | bus:cockpitCabinTemp `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_CARGO_FWD_TEMP` | bus:fwdCargoTemp `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_CARGO_BULK_TEMP` | bus:aftCargoTemp `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_1_TEMP` | bus:condMainDeckTemp_1 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_2_TEMP` | bus:condMainDeckTemp_2 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_3_TEMP` | bus:condMainDeckTemp_3 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_4_TEMP` | bus:condMainDeckTemp_4 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_5_TEMP` | bus:condMainDeckTemp_5 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_6_TEMP` | bus:condMainDeckTemp_6 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_7_TEMP` | bus:condMainDeckTemp_7 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_MAIN_DECK_8_TEMP` | bus:condMainDeckTemp_8 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_1_TEMP` | bus:condUpperDeckTemp_1 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_2_TEMP` | bus:condUpperDeckTemp_2 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_3_TEMP` | bus:condUpperDeckTemp_3 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_4_TEMP` | bus:condUpperDeckTemp_4 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_5_TEMP` | bus:condUpperDeckTemp_5 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_6_TEMP` | bus:condUpperDeckTemp_6 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_COND_UPPER_DECK_7_TEMP` | bus:condUpperDeckTemp_7 `number` | derived | first-order zone model: to FBW default selection (cabin 24 C, cargo 15 C) while a pack operates, else to OAT, tau 10 min |
| `L:A32NX_ECAM_SD_PAGE_TO_SHOW` | bus:sdPageToShow `Enum` | bundle | frame `page` (SD page controller) |

### STATUS

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_SD_STATUS_INFO_LINE_1` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INFO_LINE_2` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INFO_LINE_3` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INFO_LINE_4` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INFO_LINE_5` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_1` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_2` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_3` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_4` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_5` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_6` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_7` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_8` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_9` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_ALL_LINE_10` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_1` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_2` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_3` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_4` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_5` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_6` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_7` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_8` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_9` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_SD_STATUS_INOP_LDG_LINE_10` | useSimVar `number` | live | native FWS `Fws.status` lists (FBW 9-digit codes) |
| `L:A32NX_EWD_LIMITATIONS_ALL_LINE_1` | useSimVar `number` | live | native FWS `Fws.status` limitations (first id) |
| `L:A32NX_EWD_LIMITATIONS_LDG_LINE_1` | useSimVar `number` | live | native FWS `Fws.status` limitations (first id) |
| `L:A32NX_PFD_LIMITATIONS_LINE_1` | useSimVar `number` | live | native FWS `Fws.status` limitations (first id) |
| `L:A32NX_FWS1_IS_HEALTHY` | useSimVar `number` | live | native FWS running (`fwsa_available()`) |
| `L:A32NX_FWS2_IS_HEALTHY` | useSimVar `number` | live | native FWS running (`fwsa_available()`) |

### PERMANENT DATA

| FBW variable | read as | class | port source |
| --- | --- | --- | --- |
| `L:A32NX_ADIRS_ADR_1_TOTAL_AIR_TEMPERATURE` | bus `number` | live | native ADIRS `adrN_tat_c\|sat_c` of the captain-selected ADR |
| `L:A32NX_ADIRS_ADR_1_STATIC_AIR_TEMPERATURE` | bus `number` | live | native ADIRS `adrN_tat_c\|sat_c` of the captain-selected ADR |
| `L:A32NX_ADIRS_ADR_1_ALTITUDE` | bus `number` | live | native ADIRS `adrN_altitude_ft` |
| `L:A32NX_FCU_EFIS_L_DISCRETE_WORD_2` | bus `number` | live | native EFIS `a380x/xp12/efis/<side>/baro_std` (bit 11) |
| `L:A32NX_FCU_EFIS_R_DISCRETE_WORD_2` | bus `number` | live | native EFIS `a380x/xp12/efis/<side>/baro_std` (bit 11) |
| `L:A32NX_ADIRS_IR_1_BODY_NORMAL_ACC` | bus `number` | live | `sim/flightmodel/forces/g_nrml`, SSM from the IR attitude validity |
| `L:A32NX_FQMS_TOTAL_FUEL_ON_BOARD` | bus `number` | live | `sim/flightmodel/weight/m_fuel_total\|m_total` |
| `L:A32NX_FQMS_GROSS_WEIGHT` | bus `number` | live | `sim/flightmodel/weight/m_fuel_total\|m_total` |
| `L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC` | bus `number` | ncd | X-Plane publishes no gross-weight CG in %MAC: NCD ("--") |
| `E:ZULU TIME` | bus `number` | live | frame input `sim/time/zulu_time_sec` |

<!-- sd-census:end -->

DOOR page escape slides: FBW's DoorPage passes `LIGHT BEACON ON` (a TODO proxy) as "slide armed" to
every door. The SD frame also publishes the native per-door state `L:A380X_SLIDE_ARMED:<n>` (n =
interactive point, from the EFB's `a380x/xp12/efb/slide_armed`, docs/EFB_DEVELOPMENT.md section 7). The
version 2 edit of `tools/patch_xp_kit_sd.py` (marker `/*A380X-SD-SLIDES v1*/`, applied over a v1 or an
original bundle) makes each CabinDoor prefer that value; without it the FBW proxy stays in effect.
(This note is kept outside the generated census above so `sd_pages_census.py --write` keeps it.)

## Tests and build

* `native/avionics/test_sd_data.c` (built and run by `build.py`, also compiled with MinGW
  `-std=c11 -O2 -Wall -Wextra -Werror`): per-page values from mocked datarefs, raw ARINC encoding and
  SSM, JSON syntax and size, no NaN/inf, no invented failures, unpowered computers (FW), absent inputs,
  late producers; `--contracts` lists its X-Plane datarefs for the `--datarefs` catalogue check.
* `tools/test_patch_xp_kit_sd.py`: anchors (no JS engine needed), idempotence, unknown/partial bundles
  rejected, reuse path hash bookkeeping and a second reuse after installation; with macOS
  JavaScriptCore (`jsc`) the whole patched bundle is parsed and the helpers are executed.
* `tools/test_sd_pages_census.py`: every census variable is classified, the published names match the
  classes, this document is current (`python3 tools/sd_pages_census.py --write`).

## Verify in the simulator

1. Install the staged build (`python3 tools/install_xp_kit_cockpit.py <staging>`, X-Plane closed), then
   check in `Log.txt` that the xp-kit helper started and `a380x/xp12/xpkit/sd_active` = 1 and
   `a380x/xp12/xpkit/errors` does not grow (a frame over `XPKIT_SD_CAP` would increment it).
2. Cold & dark, BAT 1 + ESS: ELEC DC shows battery voltages/currents and the DC ESS bar; other pages
   show FBW's own unpowered indications; the lower area shows TAT/SAT (XX until an ADR is aligned),
   the UTC clock, GW and FOB, and no `SYSTEM DATA INCOMPLETE` text.
3. APU start: APU page N/EGT/flap/bleed, ELEC AC APU GEN A/B 115 V 400 Hz; BLEED with APU BLEED on.
4. Engine start: ENGINE page start valve, N2/N3, FF, oil; HYD GREEN/YELLOW 5000 psi with engines
   running; FUEL tank quantities add up to FOB; WHEEL brake temperatures rise after braking.
5. Open a door (`door_open_ratio`): DOOR page shows it amber; F/CTL moves with the stick.
6. STATUS: `NORMAL` in green with no FWS item; a failure that the FWS reports adds INOP SYS / INFO.
