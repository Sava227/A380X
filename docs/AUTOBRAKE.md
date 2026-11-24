# Autobrake: A380 BCU logic (2026-11-24)

## Research

- **Mode set:** the A380 has a four-mode landing autobrake (LO, 2, 3, HI), plus BTV and RTO.
- **Spoilers:** the autobrake doesn't activate unless the ground spoilers are extended (SKYbrary; FlyByWire A380X SOP).
- **BTV:**
  - It needs the landing runway in the FMS and a runway exit selected on the OANS.
  - It brakes late, toward that exit, and releases at the exit at 10 kt.
  - It disarms on pedal force, the selector at DISARM, or the A/THR instinctive-disconnect pushbuttons, which FlyByWire's guide calls "the normal method of disarming, at 70kts groundspeed". Those three trigger the AUTO BRK OFF memo and aural (FlyByWire OANS/BTV guide).
- **ROW/ROP:** Airbus's runway overrun warning and protection grew out of BTV. On the A380, ROP applies maximum braking when an autobrake mode is engaged and an overrun is predicted (Airbus Safety First; SKYbrary ROPS).
- **What wasn't found:** no public Airbus document gave per-mode A380 deceleration figures, and the Airbus FAST 44 BTV article and the FlyByWire SOP were only available as PDFs that couldn't be read here.

The detailed reference is FlyByWire's A380 autobrake controller (`fbw-a380x/.../hydraulic/autobrakes.rs`, GPL-3.0), which this work translates.

## Before

The autobrake used X-Plane's own autobrake, with its four levels set to 2.0/2.5/3.0/3.5 m/s² (taken from FlyByWire). Where that differed from the A380:
- **Engagement:** X-Plane's, not the A380 rule (ground spoilers, nose gear).
- **Ramp:** unknown.
- **RTO:** X-Plane's own maximum.
- **ACTIV/FMA:** guessed from the observed braking.
- **DECEL:** no indication.
- **Missing disarms:** no A/THR-pushbutton disarm and no 10 s-after-lift-off disarm.
- **BTV:** silently braked at HI.
- **Web PFD:** the main (FlyByWire web) PFD never showed the autobrake FMA or DECEL, because the plugin didn't publish those L-vars.

## Now

**Core** (`native/avionics/a380x_autobrake.h`, XPLM-free; a translation of FlyByWire's A380AutobrakeController, its deceleration governor and PID controller):

| Mode | Target | Ramp |
|---|---|---|
| LO | 2 m/s² | over 2.5 s |
| 2 | 2.5 m/s² | over 2.5 s |
| 3 | 3 m/s² | over 2.5 s |
| HI | 3.5 m/s² | steps to 2 m/s² at 0.1 s, then over 2.5 s |
| RTO | 6 m/s² | none |

- **Engagement:** mode armed, ground spoilers extended, and either the nose gear has touched down or the spoilers have been out for 5 s.
- **Control:** a PI governor (Kp 0.08, Ki 0.6) works on the measured deceleration along the track.
- **DECEL:**
  - Landing modes: on at 80 % of the target, off below 70 %.
  - RTO: on beyond 2.7 m/s², off below 2.0 m/s².
- **Disarm:**
  - pedals while braking (landing: one pedal > 53 % or both > 11 %; RTO: one > 77 % or both > 53 %);
  - the A/THR instinctive disconnect while braking (not RTO);
  - BCU arming lost (anti-skid, normal brakes, ADIRS, power);
  - ground spoilers retracting;
  - 10 s after lift-off, for a mode armed on the ground;
  - RTO, once in flight for more than 10 s.
- **Selector solenoid:** returns the knob to DISARM 1 s after the mode becomes DISARM or RTO. This return doesn't count as a crew selection, so an RTO armed with the knob away from DISARM survives it (in FlyByWire it would be cancelled).
- **BTV:** doesn't arm without an OANS exit (not ported), so the selector springs back to DISARM, as in FlyByWire. The old code fell back to HI.

**X-Plane side** (`a380x_center_right.inc`):
- **Brakes:** the BCU brakes through `sim/flightmodel/controls/l_brake_add` / `r_brake_add`, so the crew's pedals stay readable for disarming.
- **X-Plane's own autobrake:** kept OFF. Its switch, from hardware, X-Plane commands or situations, is read as the selector or RTO pushbutton.
- **Ground spoilers:** taken from the PRIM speed brake module.
- **A/THR disconnect:** read from `a380x/xp12/fcu/athr_disconnect`.
- **New dataref:** `a380x/xp12/center_right/autobrake_decel`.

**PFD:** `L:A32NX_AUTOBRAKES_ARMED_MODE`, `ACTIVE`, `DECEL_LIGHT` and `RTO_ARMED` are now published. The FMA shows BRK LO/2/3/HI/RTO in cyan when armed and green when braking, with a green DECEL under the speed tape.

**ECAM:** the memos use the armed mode.

## Checks

- **`test_autobrake.c`** (in the build), run closed-loop on a braking model:
  - LO/2/3/HI settle at 2.00/2.50/3.00/3.50 m/s², and RTO at 6.0;
  - the ramps, the nose-gear and 5 s engagement rules, and every disarm path;
  - the RTO thresholds, the 10 s-after-lift-off rules, BTV and the solenoid.
- **Lifecycle tests:** the updated `test_control_lifecycle.c` and `test_lg_grvty_adapter.c` pass.
- **Build:** the full build passes (avionics + flyPad tests, contract checks, Mac + Windows with WebView2). `validate_avionics_provider` reports no errors.
- **Not yet flown in X-Plane.**

## Not yet done

BTV braking-to-exit and ROW/ROP. Both need the OANS runway and exit selection, plus the runway/exit geometry.

## Source and install

**Source:** commit `e799685` on `latest-sync-20261124`.

**Installed:**
- `plugins/A380XAvionics/{mac_x64,win_x64}/A380XAvionics.xpl`
- `plugins/A380XAvionics/build-report.json`
- `avionics-providers/fbw-avionics-v1.json` (new dataref and hashes).
