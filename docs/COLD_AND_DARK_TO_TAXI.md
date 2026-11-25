# Cold and dark to taxi (X-Plane A380X)

FlyByWire's A380X beginner guide, chapters
[Cockpit Preparation](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-beginner-guide/02_cockpit-preparation/),
[Preparing the FMS](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-beginner-guide/03_preparing-fms/) and
[Engine Start and Taxi](https://docs.flybywiresim.com/pilots-corner/a380x/a380x-beginner-guide/04_engine-start-taxi/),
mapped to this X-Plane port. Only the steps that do something here are listed; switches FBW marks INOP
(recorder, EVAC, ELT, cockpit door, CVR test, body accumulator, ETACS) and the cosmetic overhead knobs
(temperatures, pack flow, LDG ELEV) can be set but have no system behind them.

**Shortcut:** flyPad → Presets → **READY FOR TAXI** sets the aircraft up in one step (it also has
COLD & DARK, BEFORE START, AFTER START, READY FOR PUSHBACK and READY FOR TAKEOFF). As in FBW it starts
the engines through the FADEC (ENG START selector IGN START, ENG MASTER 1-2 then 3-4, selector back to NORM)
and ends with a T.O CONFIG test. The FMS still needs the data entry in step 4.

**Spawn:** a new flight on the ground with the engines stopped is cold & dark (BAT 1, ESS, 2, APU OFF; IR 1-3
OFF), also when X-Plane only places the aircraft on the ground after the plugins have loaded (Log.txt:
"A380X ELEC: on the ground with the engines stopped after load: cold & dark"). Engines running, or a start
in the air, is READY.

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
3. APU MASTER SW → ON: the SD shows the APU page by itself. APU START → ON; the intake flap opens (8 s), then the
   APU follows FBW's PW980 curve: starter off at N 55 % (34 s), APU AVAIL 2 s after N 95 % (about 42 s).
   The SD goes back to its previous page 10 s later. APU MASTER OFF: FBW's 60 s cooldown at 100 %
   (APU GEN still on line), then the spool-down.
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
2. After about 10 s: ENG MASTER 1, then 2 → ON. Each starts automatically as FBW's FADEC: 1.7 s after
   MASTER ON the starter and igniter engage and stay on until the engine is at idle (Log.txt: "A380X ENG
   START: engine N at idle ... s after MASTER ON"). Then ENG MASTER 3 and 4. The spool-up itself is
   X-Plane's engine model (FBW's comes from MSFS's engine model), so the time to idle is X-Plane's.
3. Starting needs bleed air: APU BLEED ON with the APU available, or an engine already running. Without
   it the starters do not turn (Log.txt: "A380X ENG START: no bleed air").
4. A start that does not reach idle within 120 s is aborted and ENG MASTER FAULT lights; cycle the
   ENG MASTER or the selector to retry.

## 7. After start and taxi
1. ENG START selector → NORM (the SD returns to its previous page), APU BLEED OFF, APU MASTER SW OFF.
2. Ground spoilers ARM, rudder trim zero, flaps to the takeoff setting, pitch trim checked on the PFD.
3. SD STATUS checked; flight controls checked on the SD F/CTL page; ECP C/L → AFTER START.
4. NOSE light TAXI, PARK BRK OFF, a little thrust; brakes checked; steer with the rudder / tiller axis.
5. On the way: autobrake RTO, T.O CONFIG test (ECP), BEFORE TAKEOFF checklist.

## 8. Ready for takeoff
1. ECP **T.O CONFIG** (hold) → T.O CONFIG NORMAL in the T.O memo when flaps / slats, speed brake, pitch and
   rudder trim, doors, T.O speeds and brake temperature are right; otherwise the ECAM names what is wrong.
2. MFD SURV: TCAS TA/RA; transponder AUTO. ECP C/L → BEFORE TAKEOFF.
3. Lights: STROBE ON, NOSE light T.O, landing lights ON (flyPad READY FOR TAKEOFF does these three).
4. PARK BRK OFF, thrust levers FLEX or TOGA; A/THR arms with the levers in the detent.
