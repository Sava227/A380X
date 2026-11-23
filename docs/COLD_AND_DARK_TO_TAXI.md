# Cold and dark to taxi (X-Plane A380X)

FlyByWire's A380X beginner guide, chapters
[Cockpit Preparation](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-beginner-guide/02_cockpit-preparation/),
[Preparing the FMS](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-beginner-guide/03_preparing-fms/) and
[Engine Start and Taxi](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-beginner-guide/04_engine-start-taxi/),
mapped to this X-Plane port. Only the steps that do something here are listed; switches FBW marks INOP
(recorder, EVAC, ELT, cockpit door, CVR test, body accumulator, ETACS) and the cosmetic overhead knobs
(temperatures, pack flow, LDG ELEV) can be set but have no system behind them.

**Shortcut:** flyPad → Presets → **READY FOR TAXI** sets the aircraft up in one step (it also has
COLD & DARK, BEFORE START, AFTER START, READY FOR PUSHBACK and READY FOR TAKEOFF). The FMS still needs
the data entry in step 4.

## 1. Pre-checks
ENG MASTER 1-4 OFF, PARK BRK ON, speed brake retracted, flaps 0, thrust levers IDLE, gear lever DOWN,
ENG START selector NORM, wipers OFF.

## 2. Power up (overhead ELEC)
1. BAT 1, ESS BAT, BAT 2, APU BAT → ON (pushed in, OFF lights out).
2. flyPad → Ground Services → **GPU** on, then EXT PWR 2, 3, 1, 4 → ON when AVAIL shows.
3. Cockpit lighting as required. ANN LT on the INT LT panel → TEST checks every overhead light, then BRT.

## 3. Fire test and APU
1. APU / ENG FIRE pushbuttons in and guarded, agents not discharged.
2. **FIRE TEST** (aft overhead) → press and hold: master warning and chime, ENG / APU FIRE on the ECAM,
   agent SQUIB lights. (The FIRE pushbuttons themselves have no lit art in this port.)
3. APU MASTER SW → ON: the SD shows the APU page by itself. APU START → ON; APU AVAIL lights in about a
   minute and the SD goes back to its previous page 10 s later.
4. APU BLEED → ON (needed for the engine start).

## 4. Cockpit preparation and FMS
1. ADIRS: IR 1-3 → NAV (alignment runs on the ADIRS panel and the PFD).
2. Exterior lights: STROBE AUTO, NAV ON; SEAT BELTS ON; EMER EXIT LT ARM.
3. RMP 1/2 (3) ON, frequencies set; MFD SURV page: transponder AUTO, TCAS TA/RA, squawk.
4. MFD FMS: INIT (flight number, FROM / TO, CI, CRZ FL) → DEPARTURE (runway, SID) → F-PLN (route, then
   INSERT TMPY) → ARRIVAL (runway, approach, STAR) → FUEL&LOAD (ZFW, ZFWCG, block fuel) → T.O PERF
   (V1 / VR / V2, FLEX, flaps, THS) as in FBW's chapter 3.
5. flyPad → **Fuel and Payload**: load the fuel and payload you entered (SimBrief import if available).

## 5. Before start
1. EXT PWR 1-4 off and flyPad GPU disconnected (APU carries the network).
2. BEACON → ON; flyPad → **Doors and Load**: doors closed (SD DOOR page).
3. Thrust levers IDLE, PARK BRK as required, ECP **C/L** → BEFORE START checklist.
4. Pushback: flyPad → **Pushback** (ground only).

## 6. Engine start (pedestal ENG panel)
1. ENG START selector → **IGN START**: the SD shows the ENGINE page by itself.
2. After about 10 s: ENG MASTER 1, then 2 → ON. Each starts automatically (starter and igniter until
   N2 50 %, then idle). Then ENG MASTER 3 and 4.
3. Starting needs bleed air: APU BLEED ON with the APU available, or an engine already running. Without
   it the starters do not turn (Log.txt: "A380X ENG START: no bleed air").
4. A start that does not reach 50 % N2 within 120 s is aborted and ENG MASTER FAULT lights; cycle the
   ENG MASTER or the selector to retry.

## 7. After start and taxi
1. ENG START selector → NORM (the SD returns to its previous page), APU BLEED OFF, APU MASTER SW OFF.
2. Ground spoilers ARM, rudder trim zero, flaps to the takeoff setting, pitch trim checked on the PFD.
3. SD STATUS checked; flight controls checked on the SD F/CTL page; ECP C/L → AFTER START.
4. NOSE light TAXI, PARK BRK OFF, a little thrust; brakes checked; steer with the rudder / tiller axis.
5. On the way: autobrake RTO, T.O CONFIG test (ECP), BEFORE TAKEOFF checklist.
