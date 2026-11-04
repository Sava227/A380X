# Cockpit feedback pass (November 2026)

Motion, click targets and lamps for FBW-authored controls that were static in the port. One contract
(`tools/cockpit_feedback_contract.py`) drives the geometry patcher (`tools/cockpit_feedback.py`) and the
native module (`native/avionics/a380x_cockpit_feedback.inc`, table generated into
`a380x_cockpit_feedback_table.h`, checked by `native/avionics/build.py`).

| Group | Controls | Motion | Click | Lamps / function |
| --- | --- | --- | --- | --- |
| SWITCHING | ATT HDG, AIR DATA, FMS | FBW clip, 3 detents | rotary knob, `a380x/xp12/switching/<knob>_left/_right` | ATT HDG / AIR DATA switch the captain or F/O IR / ADR to unit 3 (native ADIRS); FMS drives `a380x/xp12/fms/switching_knob` |
| ECAM control panel | 28 keys (ENG keeps its own prototype) | 0.6 mm press on the existing `ecam/sd/*` commands | yes | green bar on the displayed SD page key; CLR lit while an ECAM warning is waiting |
| SURV panel | XPDR/TCAS SYS 1/2, WXR/TAWS SYS 1/2, ABV, BLW, TA ONLY, G/S MODE | FBW press clip | yes, `a380x/xp12/surv/<key>` | write the shared SURV datarefs (also used by the MFD SURV page); green bars follow them, TA ONLY "ON", G/S MODE "OFF" |
| KCCU | 63 key caps per side | FBW clip, follows the existing `kccu/<side>/<key>` commands | existing targets | key travel only |
| ENG MASTER 1-4 | number decal, FIRE / FAULT faces | follow the lever (`mixture_ratio[n]`) | existing toggle | the decals no longer float at the OFF pose |
| Overhead | 103 pushbuttons with a latched position (docs/OVERHEAD_LAMPS.md) | FBW press clip on `ovhd/pb/<id>` | yes | legends follow the position (dark cockpit); EMER EXIT LT OFF has no FBW clip, command only |
| RMP 1-3 | 43 keys, 12 reception knobs, OFF/BRT, NAV selector, INT/RAD, STBY NAV guard, standby LED | FBW clips (keys press; knobs push in/out with reception, turn with volume) | yes (knobs: turn on the body, push on the cap) | screens on cockpit devices `a380x.xp12.rmp.<n>`; VHF and SQWK pages (native/avionics/a380x_rmp.h, FBW logic); green bars on the transmitting CALL key |
| Glareshield side | CONSOLE & FLOOR LT (OFF / DIM / BRT), OIT SIDE (FLT OPS / NSS AVNCS) | FBW clip | up/down switch | state datarefs only (`lighting/<side>_console_floor`, `oit/<side>_side`) |
| Glareshield / sidestick keys | CHRONO, SIDESTICK, MASTER WARN / CAUT, ATC MSG, PTT (both sides), sidestick takeover, AUTOLAND lights, loudspeaker knobs | FBW clips on the glareshield module's commands (hooked) / `volume_<side>` 0..100 | existing targets; stick takeover and AUTOLAND new | takeover = `glareshield/takeover_<side>`; AUTOLAND is a press only |
| Sidesticks | captain / F/O stick | roll and pitch clips follow `sim/joystick/yoke_roll_ratio`, `yoke_pitch_ratio` (F/O: `_copilot`) | - | the F/O stick stays centred unless copilot axes are bound |
| EFIS / KCCU / OIT | EFIS PFD, RECONF, blank keys; OIT brightness knobs; KCCU blank keys, KBD / CCD switches (follow `kccu/<side>/kbd_on`, `ccd_on`), captain scroll wheel; OIT keyboard (88 keys, moves with the tray), ACD keypads | FBW clips | yes (switches and wheel: existing targets) | key travel; no OIT application behind the keys yet |
| Panel keys | VENTILATION SMOKE, CALLS (MECH, decks, PURS, rests, ALL), EVAC HORN OFF, MASK MAN ON, TMR RESET, door LOCKING SYS, GND HF DATALINK, MAINT GND CONNECTION, CARGO SMOKE TEST, PA, DFDR EVENT, ACMS TRIGGER, WXR VD AZIM push, two unlabelled overhead keys; CREW OXY SUPPLY (hooked `sim/oxy/crew_valve_toggle`) | FBW press clips, `a380x/xp12/cockpit/<key>` | yes | press only |
| Fire | ENG 1-4 / APU FIRE pushbuttons and guards | FBW clips, eased toggles `fire/<unit>_pb`, `fire/<unit>_guard` | yes | ENG pushbuttons released publish `a380x/xp12/engine/fire_pushbutton[0..3]` (EWD / SD / FWS inputs); no extinguishing or isolation yet |
| Guards, breakers | 42 overhead guards (`guard/<name>`), 60 circuit breakers (`cb/<name>`), laptop POWER switches | FBW clips, eased toggles | yes; a closed guard is in front of its pushbutton, open it first | state only |
| Overhead knobs / switches | PACK FLOW, CKPT / FWD CABIN / BULK / FWD CARGO temperature, LDG ELEV, MAN V/S CTL, READ / READING LT, X BLEED, RWY TURN OFF, WING, LOGO, STORM, EMER EXIT LT, STBY COMPASS, EVAC CAPT, ELT, pedestal lighting knobs, WXR VD AZIM | FBW clips; rotaries 0..1 (`cockpit/<key>`, `_up` / `_down`), selectors / switches as above | yes | state only (no system behind them yet) |
| BAT selector | KNOB_OVHD_BAT | follows `elec/bat/selector` 0..4 | rotary on `elec/bat_selector_right` / `_left` | the ELEC panel BAT display source |
| Parking brake | lever | follows `sim/cockpit2/controls/parking_brake_ratio` | `sim/flight_controls/park_brake_toggle` | X-Plane brake |
| Wipers | CAPT / F/O WIPER selector (OFF / SLOW / FAST), blades | selector clip; blades follow `wipers/<side>_sweep` | rotary | native sweep (1.2 / 2 strokes per s, needs AC), parks at OFF |
| Furniture | seats (height, fore-aft), armrests (stow, height, tilt), inboard armrests, tables, meal tables, keyboard trays, footrests, sliding windows, sun shades (forward, side), stowage doors (AFT OXY, AFT OIT, LH / RH, CAPT OXY, gear pin) | FBW clips (nested parts compose, e.g. the keyboard keys ride on the tray) | on FBW's click spots; seats / armrests: click up or down | state only |

Datarefs: `<key>_pressed` / `press_ratio` 0..1 for key travel; selectors and switches are writable ints.
Press handlers are registered in front of the owning module's handler and return 1, so the original
action still runs. Lamps are dark without DC power.

Geometry notes

* Only TRIS ranges that were not animated before are wrapped; VT / IDX / UV payloads are unchanged.
* The exported legend decals sit 2 mm out along their face normal (earlier legend repair); legends are
  matched at either position.
* Lamp faces in the emissive decal (SURV bars) get their own `ATTR_light_level` and are hidden while dark.
  Lamp faces in decals without a LIT texture (ECAM bars, TA ONLY, G/S MODE) get lit quads in the controls
  object (fcu_labels atlas cells), drawn alpha-tested so they write depth.
* A clip that moves several nodes moves each of them; a node below another animated node gets both
  motions (outer first). Rotations that change axis during the clip (sliding window cover, table hinge)
  are written as world Z / Y / X tables, exact at every FBW key.
* Not done: the captain / F/O main sun shades and the aft shades are skinned (bones, no mesh nodes) and
  cannot be moved by OBJ animation; the F/O big armrest has no FBW stow clip; the seat adjuster switch
  rocking motion and the KCCU KBD / CCD switch ON direction need the behaviour XML to confirm (the switch
  shows ON at the clip's last key).
* `tools/audit_fbw_animations.py --aircraft <aircraft>` lists every FBW clip as animated / static /
  partial / not exported in the X-Plane objects.

Run (X-Plane closed): `python3 tools/cockpit_feedback.py --aircraft "<X-Plane>/Aircraft/A380X-XP12"`.
Re-running is a no-op. Not yet tested in the simulator.

## RMP (radio management panels)

Logic: `native/avionics/a380x_rmp.h` (XPLM-free port of FBW's VhfComController, VhfComPage,
TransponderController, RmpStateController and RadioUtils; test `test_rmp.c`), adapter
`a380x_rmp_adapter.inc`.

* Power: RMP 1/2 on DC ESS, RMP 3 on DC 1; OFF/BRT at 0 is standby (green LED), screen dark.
* VHF page: ADK/LSK of another row select it; on the selected row ADK swaps active and standby, LSK enters
  the typed standby. Typing follows FBW auto-completion (8 -> 118.000, 875 -> 118.750); invalid entries show
  amber; CLR deletes a digit, held 1 s clears. UP/DOWN change the VHF 3 standby mode (DATA / EMER / frequency).
* SQWK page: AUTO/STBY, code entry (8/9 invalid, validates 1 s after the 4th digit or on LSK), ADK3 IDENT.
* VHF 1 / 2 are X-Plane COM 1 / 2 (8.33 kHz), followed both ways; the code is X-Plane's transponder code.
  RMP 1 also sets X-Plane's transmit selector and COM 1/2 reception and volume.
* HF, TEL, MENU, NAV pages and the call/audio functions beyond VHF are not modelled (FBW has no such pages);
  their keys and knobs move and click.
