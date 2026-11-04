# A380 electrical network, cold & dark and flyPad presets (phase 2)

Phase 2 replaces the port's single "avionics master" power criterion with a native A380
electrical network. Every display, panel lamp and system in the port now asks it **which bus is
powered**. The aircraft spawns cold & dark when X-Plane starts with the engines stopped. The
flyPad offers FlyByWire's five aircraft-state presets as timed native procedures.

The normative design is [ELEC_SYSTEM_SPEC.md](ELEC_SYSTEM_SPEC.md). The panel controls are
described in [ELEC_PANEL.md](ELEC_PANEL.md). The behaviour comes from the FlyByWire A380X Rust
model (`a380_systems/src/electrical`, GPL-3.0) and FBW's `aircraft_preset_procedures.xml`.

| Part | File |
| --- | --- |
| Network core (XPLM-free C11) | `native/avionics/a380x_elec.h`, `a380x_elec.c` |
| flyPad preset tables and runner (XPLM-free) | `native/avionics/a380x_elec_presets.h`, `a380x_elec_presets.c` |
| Consumer power view (header only) | `native/avionics/a380x_elec_power.h` |
| X-Plane adapter | `native/avionics/a380x_elec_adapter.inc` |
| Contract (datarefs, commands, intercepted stock commands) | `tools/elec_system_contract.py` |
| Tests | `native/avionics/test_elec.c` (core and presets), `test_elec_adapter.c` (mock XPLM) |

## Cold & dark power-up: step by step

Load the A380 at a gate with **Start with engines running** unchecked. The aircraft is cold &
dark: all four batteries are OFF, every display, the ISIS and the FCU are dark, and the overhead
lamps are dark. Only the hot buses are live, which run the clock.

| # | Do this | You should see |
| --- | --- | --- |
| 1 | Overhead ELEC: press **BAT 1** (OFF light goes out) | Nothing else. BAT 1 alone powers nothing. |
| 2 | Press **ESS BAT** | DC ESS comes alive on BAT 1 + ESS BAT. The static inverter feeds AC EMER. The **EWD** and the **CAPT MFD** show the Thales boot bar, then *SAFETY TEST IN PROGRESS* for 15 s, then their pages. The **ISIS** starts its 90 s INIT. The lamps on DC ESS light: **GEN 1/2 FAULT** (engines stopped, FBW behaviour), the OFF legends of the pbs still OFF, BUS TIE / AC ESS FEED (no FAULT, because AC EMER is live). The CAPT PFD stays dark, as on FBW: it is on the in-flight DC ESS sub-bus 108PH, which is shed on the ground on batteries. The CAPT EFIS control panel is powered (DC ESS); the FCU is not. |
| 3 | Press **BAT 2** | No change. DC 2 needs AC. The BAT 2 OFF legend stays dark because its lamp is on DC 2. |
| 4 | Press **APU BAT** | The DC APU bus is live (APU start bus). |
| 5 | flyPad **GROUND → GROUND POWER** (or ISCS *EXTERNAL POWER*) | The ground power unit connects. **EXT 1-4 AVAIL** light green. |
| 6 | Press **EXT 2** | EXT 2 ON (blue). AC 1/2, AC ESS, DC 1, DC 2 (through the DC bus tie), DC ESS (TR ESS) and 108PH are powered. The CAPT PFD/ND, the F/O PFD/ND/MFD and the SD boot and self-test. The FCU and F/O EFIS panel light up. The batteries start charging. |
| 7 | Press **EXT 3** (then EXT 1 and EXT 4) | AC 3/4, TR 2, TR APU. Each bus moves to its own EXT. The 10 s display standby hides the source change. |
| 8 | **APU MASTER SW** | The ECB (DC ESS) opens the air intake flap (8 s). |
| 9 | **APU START** after the flap is open | The start contactors close on the DC APU bus and X-Plane's APU starter turns. The starter cuts out at 55 % N. **AVAIL** comes at 95 % N + 2 s, about 1 minute with X-Plane's 74 s APU spool-up. With EXT 2+3 on, the APU generators take only AC 1 and AC 4 (FBW priorities). Switch EXT 1-4 off before pushback and the APU feeds all four buses. |
| 10 | **ADIRS 1, 2, 3 → NAV** | ADIRU 1 runs on AC EMER, ADIRU 2 on AC 2 and ADIRU 3 on AC 1/AC ESS. **ON BAT** lights when a unit runs from its hot-bus backup, for example IR 2/3 with batteries only. Alignment takes the usual ADIRS time. |
| 11 | Continue the cockpit preparation: crew oxygen, lights, signs, fuel pumps, FCU/EFIS, MFD INIT | |
| 12 | Engine start | Each generator joins its bus 0.5 s after its frequency is in range (360-800 Hz). The GEN FAULT lights go out. |

Shutting down (spec 8.2): before the engine masters go OFF, put the APU or EXT power on. After
the masters are OFF, switch the APU MASTER off and **wait for the APU to stop** before BAT OFF.
If DC ESS is lost while the APU still turns above 8 % N, the plugin commands an APU emergency
shutdown and the APU MASTER pb pops out. With all BAT OFF the displays go to standby and then
dark after 10 s. A connected GPU keeps only the ground-service buses live.

## Spawn (spec 7.2)

The network decides the spawn once per aircraft load. It decides at plugin start, again at
`XPLM_MSG_PLANE_LOADED`, and during the next 90 frames, because X-Plane starts the engines a
frame or two after the load message.

* **Cold & dark** on the ground with the engines stopped. All BAT pbs are OFF. GEN / APU GEN /
  BUS TIE / AC ESS FEED / GALLEY / PAX SYS / COMMERCIAL / ELMU stay in their normal positions,
  and EXT is not connected. The DUs are Off and need a full boot. The ISIS is off and the ADIRS
  are OFF and not aligned. The plugin also switches the stock APU master, feed fuel pumps,
  exterior lights, seat-belt / no-smoking signs, crew oxygen and APU bleed off, and sets the
  parking brake, once at load.
* **Ready** when any engine runs (N1 > 15 %, or running with N2 > 50 %) or the aircraft is
  airborne. All BAT pbs are AUTO, the generators supply the network, the DUs and the ISIS are
  On without a boot, and the ADIRS are aligned in NAV.
* **Turnaround** is optional: write `a380x/xp12/elec/settings/default_spawn = 1`. All BAT pbs
  are AUTO, ground power is connected with EXT 1-4 ON, the DUs are On and the ADIRS are aligned.
* The decision is published as `a380x/xp12/elec/spawn_mode` (0 C&D, 1 turnaround, 2 ready).
  FMS, ADIRS and ISIS use it instead of their own engine checks. A late READY re-initialises
  ADIRS, ISIS and FMS.

## flyPad aircraft-state presets

Use the flyPad **STATES** page (PRESETS) on the cockpit tablets or in the pop-up window. The
buttons are **COLD & DARK**, **TURNAROUND**, **READY PUSHBACK**, **READY TAXI**, **READY TAKEOFF**
and **EXPEDITE ON/OFF**. The page shows the running preset, the step count and the name of the
current step. Tap a running preset again to stop it. The ISCS *AIRCRAFT STATE* buttons start
presets 1, 2 and 4. Commands: `a380x/xp12/elec/preset/1`…`5`, `preset/cancel`,
`preset/expedite_toggle`.

Each preset runs FBW's procedure lists in FBW order: 1 = TAKEOFF_OFF, TAXI_OFF, PUSHBACK_OFF,
POWERUP_OFF; 2 = … POWERUP_ON; 3 = … POWERUP_ON, PUSHBACK_ON; 4 = … TAXI_ON; 5 = … TAKEOFF_ON.
The steps keep FBW's names, types (STEP / PROC / NOEX / EXON / COND / NCON / ECON) and delays.
A step whose target is already set is skipped. Expedited mode skips the delays and the
normal-only steps. Presets work on the ground only, and presets 1-2 need the engines stopped.
The steps act on native controls:

* ELEC pbs. These are silent authority writes, the same as `pb/<id>`.
* Ground power (connect, plus the X-Plane GPU truck when it can come).
* APU MASTER (stock switch) and native START, then "Await APU AVAIL", then APU BLEED.
* ADIRS 1-3 NAV through the native ADIRS, and an EXON *ADIRS Align Now* before the alignment
  waits in expedited mode.
* Crew oxygen, strobe (OFF / AUTO / ON), nav & logo, beacon, seat belts, no smoking
  (OFF / AUTO / ON), feed-tank main fuel pumps 1-4 and the cockpit door lock.
* Engine masters and starters (X-Plane starters are held until the engine runs).
* Spoiler arm, rudder trim, flaps 1, nose light (TAXI / T.O), landing lights, RTO autobrake,
  TERR on ND and transponder.

POWERUP_ON: BAT 1, ESS, BAT 2 (1 s each), APU BAT (3 s), GPU connect, EXT 2, 3, 1 (1 s), EXT 4
(3 s), APU MASTER (3 s), APU START, await AC 1, crew oxygen, ADIRS 1/2/3 NAV, strobe AUTO,
nav & logo, seat belts, no smoking AUTO, feed fuel pumps, await APU AVAIL, APU BLEED.
POWERUP_OFF reverses it, waits until AC 1 is unpowered and removes the GPU.

These FBW steps are left out because the port has nothing to act on: ENG fire test, RCDR GND
CTL, EMER EXIT LT, RMP volumes/TX, ENG MODE SEL, cabin ready, T.O CONFIG test, FWC init, and
the standby, transfer and trim fuel pumps (the cockpit maps only the four feed main pumps). The
RWY TURN OFF lights are a native state only, because X-Plane has no separate switch. X-Plane
cannot do AUTO for the strobe or the no-smoking sign: strobe AUTO keeps the stock strobe off on
the ground, and no-smoking AUTO turns the sign on. Two port additions are not in FBW: GPU
connect/disconnect steps (FBW connects the GPU from its ground page), and the expedited-only
ADIRS Align Now.

## Implemented network

The network evaluates FBW's union-find graph in FBW statement order, in sub-steps of 50 ms or
less. It freezes in pause/replay and runs in simulator time.

* **Sources:** four VFGs. f = 12200 rpm × N2 % / 100 × 1.95 × 4 / 120 (FBW's Trent 900 HP-spool
  gearbox, 7.93 Hz per %), with N2 capped at 100 % because X-Plane's spool model overshoots it at
  TOGA (all four VFGs used to leave the band together during the take-off roll). A generator is
  normal at 360-800 Hz once stable for 0.5 s. GEN pb ON, DRIVE connected and no failure are
  required. Also: APU GEN A/B (N ≥ 77 %), EXT 1-4 (connected and EXT pb ON), the emergency
  generator (RAT), the static inverter, BAT 1/2/ESS/APU, BCRU 1/2/ESS and TR APU.
* **AC 1-4:** FBW's normal and special priority tables. One GEN or EXT feeds at most two buses,
  and one APU generator feeds at most two buses. In flight (ADR 1/3 > 54 kt) only one APU
  generator supplies. The tables set the bus-tie contactors 980XU1-6 and the system isolation
  contactor 900XU. **BUS TIE OFF** separates AC 1+2 from AC 3+4 and DC 1 from DC 2.
* **AC ESS FEED:** 3XC1 from AC 1 (NORM), 3XC2 from AC 4 when AC 1 is lost or the pb is ALTN.
* **Emergency:** below 100 kt nothing happens. With no AC above 100 kt, the RAT deploys
  (solenoids on HOT 1 / HOT ESS for MAN ON), spools up and the emergency generator feeds AC ESS
  through 5XE. The RAT & EMER GEN FAULT lamp is lit during the gap in flight. The TEFO and
  EMER CONFIG latches work, and EMER EVAC resets them. RAT MAN ON on the ground with the gear
  compressed deploys the RAT, but the GCU stays inactive.
* **AC EMER** is fed from AC ESS (3XB.1) or the static inverter (3XB.2, batteries). AC EHA is fed
  from AC 3 or AC ESS. AC GND SVC is fed from AC 3, or from EXT 1 with no AC (991PU6, EXT pb not
  needed).
* **DC:** each BCRU has a line contactor (holds 3 s after AC loss) and FBW's battery-contactor
  state machine (opens 5 s after AC loss). BAT 1 ELC 6PC1 and ESS BAT 6PB3 need BAT 1 **and**
  ESS AUTO. The DC ties 6PC2 / 980PC interlock on the previous frame's state. Also: TR APU
  7PU / APU BAT 5PB, the APU start contactors, the static inverter 7XB, DC EHA, DC GND SVC, and
  108PH (stays powered 10 s after AC loss, then shed on the ground on batteries).
* **Batteries:** FBW's 23 Ah model with its verbatim voltage polynomial. A full battery reads
  27.96 V; spec 3.10 says 28.96 V, but FBW's own polynomial gives 27.96 V. Charging is
  (V_bus − V_bat) / 0.15 Ω, limited to 10 A. Discharge uses the spec's constant loads: DC ESS
  1 kW, AC EMER 0.9 kW, DC APU 0.1 kW (+9 kW while the starter runs), a DC main bus 0.8 kW,
  15 W per hot bus, and 120 W per ADIRU on its backup. On the ground on BAT 1 + ESS, DC ESS
  lasts about 30 minutes. Below 24.5 V (SOC < 20 %, about 4.6 Ah) the ELC and the ESS BAT line
  contactor open and DC ESS is lost.
* **Shedding:** COMMERCIAL 1/2, GALLEY, PAX SYS and ELMU are **five independent pbs**. COMMERCIAL
  1 or 2 OFF counts as "commercial off", and GALLEY or PAX SYS OFF counts as "galley & cabin
  off". Main galley shed: no AC, either off, or the four AC buses supplied by two sources or
  fewer. FBW's code compares "<= 2" the other way round, which would shed with four
  generators. Secondary galley: no AC or either off. The result is published as `galley_shed`.
* **Failures → FAULT lamps:** GEN n (`rel_genera0-3` or `fail/genN`), BAT (`rel_batter0-3` or
  `fail/bat*`; contactors forced open), DRIVE oil (`fail/driveN`, FAULT only), APU GEN A/B, ELMU,
  TR 1/2/ESS/APU, static inverter, EXT 1-4 and galley (native `fail/...`). Bus failures:
  `rel_esys..rel_esys4` → AC 1-4, `rel_esys5` → DC 1, `rel_esys6` → DC 2, and `fail/dc_ess_bus`.
* **Lamps (spec 4.2):** each legend has its own supply. BAT 1/ESS, GEN 1/2, APU GEN A, DRIVE,
  EXT ON, BUS TIE, AC ESS FEED, PAX SYS and RAT are on DC ESS. BAT 2/APU, GEN 3/4, APU GEN B,
  GALLEY, COMMERCIAL 2 and ELMU are on DC 2. COMMERCIAL 1 needs DC 1 and DC ESS. EXT 1/2 AVAIL
  is lit by ground power or DC 1, EXT 3/4 AVAIL by ground power or DC 2. GEN FAULT = line
  contactor open with the pb ON. AC ESS FEED FAULT = AC EMER unpowered. Every other overhead
  legend (APU MASTER SW on DC APU, APU START on AC ESS or DC APU, bleeds, packs, fuel, anti-ice,
  ...) follows its FBW `SEQ_POWERED` bus too: [OVERHEAD_LAMPS.md](OVERHEAD_LAMPS.md). The opt-in
  setting `a380x/xp12/ovhd/settings/lamp_supply = 1` replaces every lamp supply (ELEC included)
  by "any DC bus live or a battery pb AUTO" (not FBW).
* **APU (spec 9.3 option A):** X-Plane's turbine with a native ECB. MASTER is the stock
  `APU_starter_switch` (cockpit manipulator). The ECB is on DC ESS. The flap opens in 8 s. START
  latches; the start contactors need DC APU and an open flap and run to 55 % N. AVAIL = N > 95 %
  for 2 s, or N > 99.5 %. The stock switch is held at START until X-Plane's APU runs. The stock
  APU generator follows APU GEN A or B.

### Display units (FBW CDS, spec 6.1-6.2)

| DU | Powered by |
| --- | --- |
| CAPT PFD | 108PH |
| CAPT ND | 108PH or DC 1 |
| CAPT MFD | DC ESS or DC 1 |
| F/O PFD | DC 2 |
| F/O ND, F/O MFD | DC 1 or DC 2 |
| EWD | DC ESS |
| SD | DC 2 |
| ISIS | DC ESS, or HOT 1 above 50 kt (own 90 s INIT) |

States: Off → Thales boot (0.25-0.45 s, white bar with a blue square) → self-test
(*SAFETY TEST IN PROGRESS / (MAX 30 SECONDS)*, 15 s,
`a380x/xp12/elec/settings/du_selftest_s` 0-30) → On. On → Standby (blank, 10 s) → Off. A unit
whose power comes back within 10 s returns without a self-test. A brightness knob at 0 counts
as unpowered. The boot screens are drawn on the panel-atlas PFD/ND/EWD and on the PFD, ND, MFD
and SD cockpit devices. The device brightness stays up while a unit boots.

## Migration: every consumer and its bus

Each consumer keeps its old criterion as the fallback when the network is unavailable, for
example when registration failed or in a module's own unit test.

| Consumer | Before | Now |
| --- | --- | --- |
| CAPT / F/O PFD (atlas, xp-kit device, `pfd_*_valid`) | `avionics_power_on` | CAPT PFD DU (108PH) / F/O PFD DU (DC 2), CDS state machine |
| CAPT / F/O ND (atlas, xp-kit device, cursor input) | `avionics_power_on` (EFIS power) | CAPT ND DU (108PH / DC 1) / F/O ND DU (DC 1 / DC 2) |
| EWD + "ADIRS ON BATT" memo | `avionics_power_on` | EWD DU (DC ESS) |
| SD (ECAM device) | `avionics_power_on` | SD DU (DC 2) |
| CAPT / F/O MFD (device, touch, xp-kit feed) | `avionics_power_on` | CAPT MFD DU (DC ESS / DC 1) / F/O MFD DU (DC 1 / DC 2) |
| FMS (FMC powered, position) | `avionics_power_on` | DC ESS, DC 1 or DC 2 (assumption) |
| FCU (AFS LCD, FCU lamps, AP/FD/A/THR availability, FCU backup page) | `avionics_power_on` | 108PH or DC 2 (FCU channel 1 / 2) |
| EFIS control panels | FCU power and PFD | CAPT DC ESS, F/O DC 2 (spec assumption) |
| ISIS 1 / 2 | stock `bus_volts[0]` / any battery | DC ESS / HOT 1 (above 50 kt) |
| ADIRU 1 / 2 / 3, ON BAT | stock bus, "battery without generator amps" | AC EMER / AC 2 / AC 1 or AC ESS, each backed by HOT ESS / HOT 2 / HOT 1 while DC ESS is live; ON BAT = backup only (spec 6.4 proposal) |
| Clock | any battery or bus voltage | HOT 1 |
| Autobrake / RTO availability | `avionics_power_on` | AC ESS 400XP |
| Glareshield MASTER WARN / CAUT / AUTOLAND / ATC lamps | always | FWS 1/2: DC ESS or DC 2 |
| Sidestick priority lamps | always | CAPT AC 1, F/O AC 2 |
| ELEC / EMER ELEC PWR panel lamps | any stock bus ≥ 18 V | per legend (spec 4.2) |
| flyPad tablet charging and STATES page | stock battery/GPU/APU/generator switches | any AC main bus (`ac_main_powered`) |
| flyPad checklist "BATTERIES OFF" | stock `battery_on` | the four native BAT pbs |
| flyPad / ISCS ground power, presets, APU quick start | `toggle_gpu_request`, stock quick start | native GPU connect, native presets, APU MASTER + native START |
| FCU / EFIS integral lighting, ELEC annunciators | `avionics_power_on` | FCU / EFIS / lamp bus (above); stock flood and panel lighting follow the slaved stock bus (= native DC) |

There is no native RMP or OIT in the port yet. X-Plane's radios and other stock consumers follow
the slaved stock bus, which is native DC. The OIT states are published for later use.

## X-Plane slaving and interception (spec 9.3, 9.5)

Every frame after the core step, the adapter writes:

* `battery_on[*]` = native DC ESS, DC 1 or DC 2. The stock battery charge is pinned to
  `battery_watt_hr_max` while native DC is live.
* `generator_on[n]` = GEN pb ON and DRIVE connected.
* `APU_generator_on` = APU GEN A or B.
* `GPU_generator_on` = a native EXT contactor is closed. This differs from the spec's "GPU
  connected", so that the stock bus is never live with a dark cockpit.
* `avionics_power_on` = DC ESS, `cross_tie` = 1 and `inverter_on[0]` = static inverter.

Hardware bindings to the stock battery 1/2, generator 1-4, generators toggle, APU generator,
GPU, APU start and avionics commands are consumed and act on the native pbs. The avionics
commands are ignored and logged once. A user's X-Plane ground-power request still connects or
removes the GPU. The GPU disconnects when airborne or above 1 m/s ground speed.

## Decisions on the spec's open questions

1. **Captain PFD dark on batteries** (12.3): the FBW CDS bus table is followed, so the CAPT PFD
   is on 108PH, which is shed on the ground.
2. **GEN 1-4 FAULT lit with the engines stopped** (12.4): FBW behaviour, contactor open with the
   pb ON.
3. **APU turbine** (12.7): X-Plane's turbine with a native ECB (option A). Spool-up is 74 s, not
   the PW980 curve.
4. **COMMERCIAL 1 / 2, GALLEY, PAX SYS, ELMU** (12.9) are five independent pbs.
5. Battery capacity is 23 Ah (FBW). The battery loads are the spec's constant table (L).
6. ADIRU supplies follow the spec 6.4 proposal. The hot-bus backup needs DC ESS live, so an
   ADIRU left in NAV does not run a cold & dark aircraft (L).
7. **BAT OFF with the APU turning** commands an APU emergency shutdown (spec 8.2, L).
8. The VFG frequency is mapped from N2 (FBW Trent 900 gearbox, N2 capped at 100 %, L), and the APU flap opens in 8 s (FBW
   uses 6-12 s at random).
9. EMER GEN TEST, GEN 1 LINE and the BAT selector have no panel controls in the FBW model. The
   BAT voltmeter is published as `bat/selector` (writable, 0 ESS, 1 APU, 2 OFF, 3 BAT 1,
   4 BAT 2) and `bat/display_volts` (−1 blank; 88.8 with ANN LT TEST and DC 2).

## Published names

See `tools/elec_system_contract.py`, 157 datarefs and 10 commands. Prefix `a380x/xp12/elec/`
unless shown otherwise.

* `bus/<ac1…dc_gnd_svc>_powered|_volts` and `contactor/<id>` (`3XB_1`, `10KA_5KA`, …).
* `gen/{potential,freq,load_pct,line_closed}[4]`, `drive/connected[4]`,
  `apu_gen/{potential,freq,load_pct}[2]`, `ext/{potential,freq}`,
  `emer_gen/{potential,load_pct}`, `rat/stow_ratio` and `stat_inv/{potential,freq}`.
* `bat/{potential,current,charge_ah(w)}[4]` in order BAT 1, BAT 2, ESS, APU; `bat/selector` (w)
  and `bat/display_volts`.
* `tr/{potential,current}[4]` in order TR 1, TR 2, TR ESS, TR APU.
* `emer/{active,tefo,config}` and `galley_shed`.
* `du/<capt_pfd…fo_oit>_state` (0 off, 1 boot, 2 self-test, 3 on, 4 standby) and
  `du/isis_powered`.
* `spawn_mode`, `settings/default_spawn` (w), `settings/du_selftest_s` (w), `network_active`
  and `ac_main_powered`.
* `fail/{tr1,tr2,tr_ess,tr_apu,stat_inv,ac_bus1-4,dc_bus1,dc_bus2,dc_ess_bus,ext1-4,galley}`
  (w).
* `preset/{active,step,step_count,step_name(data),expedited(w),done}`.
* `a380x/xp12/apu/{master_on,start_on,avail,flap_open_ratio,ecb_powered}`, and
  `a380x/xp12/apu/generator_{volts,hz,load_percent}` for the SD.
* Commands: `preset/1..5`, `preset/cancel`, `preset/expedite_toggle`, `bat_selector_left|right`
  and `a380x/xp12/apu/start`.

## Tests

* `test_elec.c` (190 checks) covers:
  * FBW configurations: everything off, GPU only, batteries only, each battery alone, EXT 2,
    EXT 2+3, all EXT, APU only, EXT + APU, 4 GEN, APU + one engine, the special table, BUS TIE
    OFF.
  * Flight: GEN losses, the single in-flight APU generator, total generator loss with RAT and
    EMER GEN, RAT MAN ON on the ground.
  * TR ESS / TR APU / static-inverter failures, AC ESS FEED, galley shedding, bus failures,
    battery fault.
  * Timers: VFG 0.5 s, BCRU line 3 s, battery contactor 5 s, 108PH 10 s, the DU state machine,
    ISIS / DU battery picture.
  * The battery curve, charge limit, discharge, SOC-low ELC and endurance.
  * Lamp supplies, the APU ECB, spawn controls and the power-view fallback.
  * Presets: FBW order and delays, waits, expedited mode, idempotent re-run, cancel.
* `test_elec_adapter.c` uses a mock XPLM with the production ELEC panel adapter. It covers the
  registry, the cold & dark spawn and the manual power-up steps 0-11 with stock slaving,
  charge pinning, GPU, APU starter handling and interception of stock battery, generator, GPU,
  APU start and avionics commands. It also covers failure mapping to FAULT lamps, ANN LT TEST,
  pause freezing the timers, the BAT voltmeter, a TURNAROUND preset (expedited), the ground-only
  rule, the late READY spawn, disable, shutdown and registration failure.
* `native/avionics/build.py` runs both, compiles the core with MinGW, and checks the registry
  against `tools/elec_system_contract.py`. The provider manifest lists the new names.

## In-simulator checklist

No in-simulator test has been run.

1. Load at a gate with the engines off. The aircraft is cold & dark: everything dark, stock
   `bus_volts` 0.
2. Follow the power-up table above. Check each step's displays and lamps, and check
   `a380x/xp12/elec/bus/*` in DataRefTool.
3. Load with *start with engines running*. The aircraft spawns READY: no boot screens, no ISIS
   INIT, ADIRS aligned.
4. flyPad presets 1-5, normal and expedited. Tap a running preset to stop it.
5. Bind a joystick button to `sim/electrical/battery_1_toggle`. BAT 1 follows it with a click.
6. Shutdown: switch BAT OFF with the APU running. The APU shuts down.
7. In flight, fail all four generators (`rel_genera0-3`) above 100 kt. The RAT deploys and the
   EMER GEN feeds AC ESS. RAT & EMER GEN FAULT is lit only during the gap.
8. Run at 4× time acceleration and pause during the APU start, the display self-test and ADIRS
   alignment.
