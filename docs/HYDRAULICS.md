# A380 hydraulics (ATA 29): custom simulation (2026-11-25)

The port now has its own hydraulic system: two independent 5000 psi systems, GREEN and YELLOW, with their
pumps, HSMU logic, accumulators and reservoirs. It is the source of truth for the overhead HYD and GND HYD
panels, the SD HYD page, the ECAM alerts and memos, and the port's pressure consumers. X-Plane's own
hydraulics now only follow it, so the flight model's gear, brakes, steering and flaps feel the same state.

**Source:** the Airbus A380 Technical Training Manual, Level II/III, ATA 29 Hydraulic Power (Ramp & Transit,
RR Trent 900, Jan 2009, pages 1-51), supplied as photos on 2026-11-25. FlyByWire's A380 hydraulics supplies the
numbers the manual does not give: `fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs` and
`fbw-common/src/wasm/systems/systems/src/hydraulic/{mod.rs,pumps.rs}`.

**Code:**

| Part | File |
|---|---|
| Model (XPLM-free, header-only) | `native/avionics/a380x_hyd.h` |
| Tests (502 checks) | `native/avionics/test_hyd.c` |
| X-Plane adapter | `native/avionics/a380x_hyd.inc` |
| Pressure view for the consumers | `native/avionics/a380x_hyd_view.h` |

## 1. What the manual describes (merged)

### Architecture

| | GREEN | YELLOW |
|---|---|---|
| Engine-driven pumps (EDP), 2 per engine | ENG 1 A/B, ENG 2 A/B | ENG 3 A/B, ENG 4 A/B |
| Electric motor pumps (EMP) | G-EMP A on AC BUS 1 (also the AC ground service bus, for cargo doors when it is the only bus); G-EMP B on AC BUS 2 | Y-EMP A on AC BUS 3, Y-EMP B on AC BUS 4 |
| EMP ground use | Cargo doors | Body wheel steering (towing) |
| HSMU supply | DC 1 / DC ground service bus | DC 1 / DC 2 |
| Reservoir air (engine HP) | ENG 1 | ENG 4 |

- Both systems run simultaneously at 5000 psi.
- In normal operation the EDPs pressurise each system. EMPs are used only on the ground with the engines
  stopped (cargo doors, body wheel steering, maintenance).
- One **fire shutoff valve** per engine is on the EDP suction line.
- One **HSMU** (hydraulic system monitoring unit) per system:
  - manages its EMPs;
  - sends warnings to the overhead panel and the displays;
  - collects the pressure transmitter, the low-pressure switches (one per pump), the reservoir sensors and
    the filter clogging indicators.

### EMP control

**Panel:** the GND HYD panel has, per EMP, an AUTO/ON pushbutton and a FAULT/OFF pushbutton.

**Inhibition:**
- The EECs inhibit the EMPs; four signals, any one is enough.
- The SD EMP symbols disappear once one engine runs with N3 above 50 %.

**HSMU automatic mode** (cargo door toggle switch → green; body wheel steering → yellow):
- The HSMU picks the EMP by **flight number** (normal electrical configuration):
  - even flight: EMP B main, EMP A back-up;
  - odd flight: EMP A main, EMP B back-up.
- An unhealthy pump is replaced by the other one.
- The mode already running has priority:
  - an EMP running in AUTO ignores manual ON requests;
  - an EMP running manually ignores automatic requests.
- An automatic start also lights the pump's **ON** legend.

**SD triangle states:**

| State | Triangle | Identifier |
|---|---|---|
| AUTO, not commanded | empty white | white |
| ON, normal pressure | filled green | white |
| ON, pressure switch < 1885 psi | filled amber | amber |
| OFF or faulty | empty amber | amber |
| Automatic mode | green filled, no link line to the circuit box | white |
| Overheat | empty triangle + amber **OVHT** | amber |

- When an EMP supplies the circuit at normal pressure (manual ON), a link line joins it to the circuit box and
  the memo **G (Y) ELEC PMP A (B) CTL** shows.
- In cargo-door automatic mode the green circuit box stays at 0 psi: the EMP feeds the cargo door section.
- **FAULT** light: on the FAULT/OFF pb; the crew selects OFF to stop the pump.
- **Overheat:** an overheat relay removes the pump's supply. OVHT and FAULT stay until the temperature is back
  to normal.

### EDPs

- The **ENG n PUMP A / B** pushbuttons (FAULT/OFF) depressurise one EDP.
- **A+B FAULT/DISC** (red, guarded) declutches both EDPs of one engine through a solenoid, in flight.
  - This is irreversible in flight.
  - Re-engagement is a ground task with the engine shut down, at the reset port (ON / OFF / RESET indicator).
  - On the ground a ring on the pump also declutches it mechanically.

### System pressure

- The pressure transmitter is on the HP belly fairing manifold, with 100 psi accuracy.
- The value shows in the SD circuit box and on the GSP gauge.
- Digits are green above 2900 psi and amber below.
- Below 200 psi the value shows 0.

### Accumulator

- One per system, upstream of the HP belly fairing manifold.
- Metal bellows, helium pre-charged, shop serviced.

### Reservoir

**Air pressurisation:**
- Nominal 65 psi (4.1 bar). Sources: engine HP air, the pneumatic system, or the GSP (10-17 bar ground cart;
  one air pressurisation valve pressurises both reservoirs).
- Each reservoir has a GSP depressurisation switch.
- A pressure switch gives **LO AIR PRESS**. Fault lights:
  - on the ground, all engines off: every EMP and EDP FAULT light of that system;
  - with an engine running: only its EDP FAULT lights.

**Quantity indication:**
- The SD scale reads 10-70 l (the reservoir holds 120 l; the indication saturates at 70 l).
- 0-10 l is the amber low-level band.
- The level shows green, or amber below 10 l, and XX when invalid.
- A movable green **nominal index** follows the fluid temperature and the ATA 32 LEHGS reservoirs and
  accumulators:
  - about 45.2 l at 20 C;
  - four tabulated levels at -20 / 0 / 20 / 40 C;
  - up to 29 l of variation with the ATA 32 state;
  - the level should be within ±3 l of it.
- **Low level** fault lights:
  - on the ground, engines off: EMP, EDP and DISC FAULT lights;
  - with an engine running: EDP and DISC FAULT lights.

**Overheat** (amber **OVHT**):
- Shown while the fluid entering the reservoir is above normal; not shown with a failed sensor.
- Every EMP and EDP FAULT light of that system comes on, whatever the engine state.

**Refilling (GSP):**
- Selector valve, level gauge, self-sealing coupling, hand pump, filter.
- Filling by ground cart or hand pump.
- The OMS servicing report shows each level against nominal.

### Filters

- HP, return and case drain filters have electrical and visual clogging indicators; the GSP filling filter has a
  visual one only.
- A clogged filter raises a class 4 message (PFR). After 700 FH it becomes class 1 (time-limited item).
- Its counter is reset through the OMS.

### Leak draining, sampling

- Bottles collect seepage from the EMPs, heat exchangers, slat/flap PCUs and the THS actuator.
- A sampling valve is on the HP manifold.

### Consumers

| Consumer | Supply |
|---|---|
| Slat PCU | Electrical unit + GREEN hydraulic unit |
| Flap PCU | GREEN + YELLOW hydraulic units |
| Cargo doors | GREEN |
| Body wheel steering | YELLOW |

## 2. What the port simulates

**Pumps:**
- EDP speed = N3 × 12200 rpm × 0.31 (X-Plane's N2 is the Trent's HP spool).
- Displacement maps are FBW's A380 EDP / EMP maps, which regulate near 5150 / 5100 psi.
- EMPs spin to 8000 rpm.
- An EDP delivers nothing below about 10 % N3 and full flow from about 25 % N3 (engine idle is above 60 %).
  Each pump section loses about 3 % of rated EDP flow to case drain (0.023 gal/s at 5000 psi). So pressure
  builds from about 15 % N3 during a start, and a windmilling engine shut down in flight shows PRESS LO.
- Suction efficiency falls with low reservoir absolute pressure (cavitation, FBW map) and with an empty
  reservoir.
- Cavitating pumps make the pressure **fluctuate** (the AIR PRESS LO procedure's condition).

**System section:**
- The section is one pressure node: fluid compliance (10 gal, bulk modulus) plus the accumulator (helium
  2612 psi pre-charge, 0.5 gal, `pV = const`).
- Each frame solves a backward-Euler volume balance: pump flow in; internal leak, consumer demand and any
  external leak out. It is stable at any frame rate.
- When pumps stop, the accumulator holds about 5-20 s before the pressure decays.

**Pump sections:**
- Each pump has its own section pressure (check valve): system pressure while it delivers, otherwise its own
  deadhead (0 when stopped).
- Low-pressure switches:
  - EDP 2900 / 3700 psi (FBW);
  - EMP 1885 / 2900 psi (manual: low below 1885 psi).
- System pressure switch: 2900 / 3700 psi (FBW).

**Fire valve and EDP controls:**
- ENG FIRE pb released: the valve closes and the EDPs starve and depressurise.
- **Depressurisation solenoid** on DC ESS. Unpowered, the pump keeps pressurising (FBW).
- **DISC:** clutch solenoid on DC 2 (engines 1/4) or DC 1 (2/3).
  - Latched; the clutch stays open even after the pb is released.
  - `a380x/xp12/hyd/edp_reset_N` re-engages it only on the ground with that engine below 5 % N3 and the DISC pb
    released.

**HSMU:** as in §1.
- Cargo door request: a cargo door (`door_open_ratio[16..19]`) moving. Body-wheel-steering request: the
  port's BWS order (`a380x/xp12/steering/body_deg`) above 0.5° below 15 m/s.
- Run-on after the request: 20 s (cargo) / 30 s (BWS), as FBW.
- Flight number parity: last digit of the FMS flight number, or `hsmu/flight_parity`. Without a number, the
  pumps alternate per operation.
- Normal electrical configuration means all four AC buses powered; otherwise the powered pump is used.
- Green automatic mode feeds the cargo door (auxiliary) section, so the green box stays at 0 psi. Yellow BWS
  automatic mode pressurises the system (it has no auxiliary section).

**Reservoir:**

| Quantity | Model |
|---|---|
| Level | Nominal index minus fluid lost |
| Nominal index | 45.2 l at 20 C, +0.1 l/C. Minus what the system and accumulator hold above 5000 psi. Green: minus 4.9 l with the gear up (fluid in the retraction actuators, FBW 1.3 gal). |
| Low level | Below 10 l (back above 11 l) |
| Air | Charges to 65 psi at 3 psi/s while ENG 1 (green) / ENG 4 (yellow) runs, any bleed source is available, or the GSP air cart is connected. Trapped air otherwise loses about 3 psi/h. GSP depressurisation vents it. |
| LO AIR PRESS | 21.76 / 25 psi (FBW switch) |
| Fluid temperature | Ambient, +40 C while pressurised (10 min time constant) |
| OVHT | Above 110 C, back below 95 C |
| EMP motor OVHT | 150 / 120 C |

**Consumer demand:**
- Gear in transit: green 0.5 gal/s.
- Flaps moving: 0.15 gal/s per system.
- Slats moving: green 0.12 gal/s.
- Flight control and speed brake activity: from stick, pedal and lever rates.

**FAULT lights** (FBW controllers + manual):

| Pushbutton | FAULT conditions |
|---|---|
| EDP | Section low while commanded (inhibited on the ground below 25 % N3); air low or level low with the pb in AUTO; reservoir overheat |
| DISC | (Level low or overheat) and not yet disconnected |
| EMP | Low pressure for 2 s while supplied; EMP or reservoir overheat; air low or level low on the ground with the engines off and the pb in AUTO |

**ON lights:** pb pressed, or the HSMU running that EMP automatically.

**Memo:** G/Y ELEC PMP A/B CTL when manually ON and the pump section is at normal pressure.

**Failures** (writable `a380x/xp12/hyd/fail/*`). X-Plane's own failures also act:

| X-Plane failure | Acts as |
|---|---|
| `rel_hydleak` / `rel_hydleak2` | GREEN / YELLOW external leak |
| `rel_hydpmp` / `rel_hydpmp2` | All EDPs of that system failed |
| `rel_hydpmp_ele` / `rel_hydpmp_el2` | That system's EMPs failed |

Our own failure datarefs add EDP, EMP, EMP overheat, reservoir overheat, leak and air leak.

**Simplified or not simulated:**
- Filter clogging and the OMS servicing pages.
- Leak measurement valves and sampling.
- The GSP gauges as 3D objects.
- The mechanical (ring) declutch.
- The heat exchangers.
- The SD nominal index is fixed at 45 l by FBW's page (the port publishes the computed one).

## 3. Interfaces

**Published** (read-only), `a380x/xp12/hyd/{green|yellow}/`:
- `pressure_psi`, `pressure_switch`, `aux_pressure_psi`, `accumulator_psi`;
- `section_psi[6]`, `section_switch[6]`: 0-3 = ENG A/B of the system's two engines, 4-5 = EMP A/B;
- `reservoir_level_l`, `reservoir_nominal_l`, `reservoir_level_low`, `reservoir_air_psi`, `reservoir_air_low`;
- `fluid_temp_c`, `overheat`;
- `emp_mode` (0 idle, 1 manual, 2 auto).

**Published** (read-only), `a380x/xp12/hyd/`:

| Dataref | Contents |
|---|---|
| `edp_active[8]`, `edp_fault[8]` | ENG1A, ENG1B, …, ENG4B |
| `edp_disconnected[4]`, `edp_clutch_engaged[4]`, `disc_fault[4]`, `fire_valve_open[4]` | Per engine |
| `emp_active[4]`, `emp_running[4]`, `emp_fault[4]`, `emp_ovht[4]`, `emp_ctl_memo[4]`, `emp_on_light[4]`, `emp_rpm[4]` | GA, GB, YA, YB |

**Writable:**
- `native_bridge` (1);
- GSP: `gsp/ground_cart[2]` (hydraulic ground cart), `gsp/air_cart`, `gsp/depress[2]`, `gsp/refill[2]`;
- `hsmu/flight_parity` (-1 = from the FMS);
- `fail/edp[8]`, `fail/emp[4]`, `fail/emp_ovht[4]`, `fail/rsvr_ovht[2]`, `fail/leak[2]`, `fail/air_leak[2]`.

**Commands:**
- `a380x/xp12/hyd/edp_reset_1..4`: reset port, held.
- `a380x/xp12/hyd/service`: GSP refill to nominal and air recharge, ground only.

**Overhead:**
- The existing latches `a380x/xp12/ovhd/pb/hyd_*` are the switch positions (defaults: EDP and EMP FAULT/OFF
  pressed in = AUTO; ON and DISC released).
- `A380XOvhdInputs` gained `sys_fault[]` / `sys_on[]`, which the model fills each frame for the 20 HYD and GND
  HYD pushbuttons.

**Consumers:**

| Consumer | What changed |
|---|---|
| SD HYD page (`a380x_sd_data.inc`) | All `L:A32NX_HYD_*` / `L:A32NX_OVHD_HYD_*` values come from the model |
| FWS (`a380x_fws_adapter.inc`) | Pressures, pump sections, pb positions, DISC, reservoir level, air, overheat, EMP faults and CTL memos |
| New FBW alerts | G/Y ELEC PMP A/B FAULT, G/Y RSVR AIR PRESS LO, G/Y SYS OVERHEAT, with FBW's items, phases and checks |
| Flaps / slats speed (`a380x_flaps.inc`) | Model pressures |
| FCU LAND capability (`a380x_fcu_adapter.inc`) | Model pressures |
| Normal-brake availability (`a380x_center_right.inc`) | Model pressures |

Each consumer falls back to X-Plane's native pressure when the model is not running.

**X-Plane native systems:**
- The ACF maps engines 1+2 and electric pump 1 to native system 1 (GREEN), engines 3+4 and electric pump 2 to
  native system 2 (YELLOW). It used to map engines 1/4 to system 1, 2/3 to system 2, and electric pump 1 to
  both.
- Each frame the adapter sets `engine_pump[e]` on while that engine's EDPs deliver into a pressurised system.
- `electric_hydraulic_pump_on` / `pump2_on` are on while an EMP or the ground cart pressurises GREEN / YELLOW.
  The ACCUS REINFLATE pb still runs electric pump 1.
- X-Plane's flight model (gear, brakes, nose steering, flaps; ACF hydraulic consumers on systems 1|2) therefore
  loses pressure when the model does. The native pressure stays in Plane Maker units (3000 psi nominal).
- `native_bridge = 0` leaves X-Plane's pumps alone.

## 4. Checks

`test_hyd.c` (502 checks, runs in `build.py`):
- **Cold & dark:** no pressure, no faults.
- **Engine start:** 5000-5150 psi in 2 s with no overshoot; steady at frame periods from 4 ms to 200 ms.
- **ENG 1 shut down in flight** (windmilling at 12 % N3): its PMP FAULT lights, cleared by OFF; green held by
  ENG 2, lost with ENG 2 too.
- **Accumulator decay** after shutdown.
- **Manual EMP:**
  - CTL memo, ON light and native pump;
  - EEC inhibition with an engine running;
  - AC bus loss;
  - pump failure → FAULT, cleared by OFF.
- **HSMU automatic mode:**
  - odd flight → A, even flight → B, B OFF → back-up A;
  - ON B ignored while in AUTO; AUTO ignored while manual runs;
  - 20 s run-on;
  - green box at 0 psi with the cargo section pressurised;
  - yellow BWS pressurises the system.
- **External leak:** RSVR LEVEL LO in minutes; EDP + DISC FAULT lights; the system lost when empty.
- **DISC:** latched, no reset in flight, reset on the ground; GSP refill.
- **Air leak:** LO AIR PRESS FAULT lights, cleared by ENG 1 HP air.
- **Reservoir overheat:** all lights; cleared when cool.
- **EMP overheat:** relay opens; FAULT/OVHT until cool, even with OFF.
- **Cavitation** at altitude without reservoir air: green degrades to about 3200 psi and fluctuates.
- **Gear-up fluid storage; ground cart.**
