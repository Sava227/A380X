# Flight controls: fly-by-wire laws and steering

The avionics plugin runs the A380's PRIM control laws and its nosewheel / body-wheel steering in place of
X-Plane's direct stick-to-surface link.

* Core: `native/avionics/a380x_fctl.h`, header-only and XPLM-free. It is tested by `test_fctl.c`, which
  includes a closed-loop bench.
* X-Plane adapter: `native/avionics/a380x_fctl.inc`, tested by `test_fctl_adapter.c`.

This is simulation software. The law is an original port law written from the public Airbus normal-law
description, not FlyByWire's PRIM code. Sources:

* [FlyByWire A32NX normal-law protections](https://docs.flybywiresim.com/pilots-corner/a32nx/a32nx-advanced-guides/protections/overview/).
  The FlyByWire A380X beginner guide refers to this page because the A380's protections are similar.
* The FlyByWire A380X briefing pages:
  * the sidesticks have no force feedback;
  * the THS is controlled by the computers in normal and alternate law and by the trim switch on the
    ground and in direct law.
* Airbus training material as summarised on those pages and on SKYbrary.

The structure and the limits are Airbus'. The gains are port values for X-Plane's flight model.

## How it reaches X-Plane

| Channel | Mechanism |
|---|---|
| Pilot input | `sim/joystick/yoke_*_ratio`, exactly as X-Plane reads joystick, mouse or keyboard. No input path is taken away. |
| Law output | X-Plane's artificial-stability channel: `override_artstab` + `artstab_*_ratio` = order − pilot input, so X-Plane's flight controls receive the law's order. |
| Autopilot | It keeps its own path (`override_joystick_pitch/roll`). While it flies, the law adds nothing and tracks it, so an AP disconnect is bumpless. |
| Auto-trim | `sim/cockpit2/controls/elevator_trim`, within the ACF's stabiliser range. A380XTrim keeps trimming for the AP and allows manual trim only on the ground or in direct law (`a380x/xp12/fbw/normal_law`). |
| Steering | `override_wheel_steer` + `tire_steer_actual_deg`. The nose gear is the most forward gear; the body gears are the main gears inboard of the wing gears. |

The module never takes an override another plugin already holds. Steering stands down while
BetterPushback is connected (`bp/connected`, `bp/started`). In replay, on plugin disable and on aircraft
reload everything is released.

## Pitch: normal law

* **Sidestick** = load-factor demand (C*-type), flown as a pitch-rate demand Δnz·g/V.
  * Full aft: +2.5 g clean, +2.0 g with flaps. Full forward: −1 g clean, 0 g with flaps.
  * The input has a 3 % deadband, then a half-linear, half-cubic curve for fine control near neutral.
* **Stick released**: 1 g, and the flight path angle is held one second after the release. Pitch is
  compensated in turns up to 33° bank.
* **Heavy-aircraft response**: the demand is lagged (0.45 s) and limited to 4°/s² pitch acceleration.
  The inner loop is PI with feed-forward, scheduled on dynamic pressure (gain × (250 kt / IAS)²,
  0.35..2.5).
* **Auto-trim**: the THS unloads the elevator at up to 0.3°/s clean and 0.5°/s with flaps. It stops in
  the protections, beyond 33° bank, outside 0.5..1.5 g, in flare and during the liftoff blend.
* **Modes**:
  * Ground mode is direct.
  * After liftoff, ground mode blends into the flight law over 5 s.
  * Flare mode starts below 100 ft RA when descending: the attitude at entry is memorised and lowered 2°
    over 8 s, so the pilot must flare. A climb above 120 ft returns to the flight law.
  * After touchdown, ground mode is restored within 2 s.

### Protections

| Protection | Behaviour |
|---|---|
| Load factor | Within the limits above (soft feedback on the measured nz). |
| Pitch attitude | +30° nose up (+25° in CONF FULL), −15° nose down; the pitch rate fades as the limit nears. |
| High angle of attack | Above α prot the stick commands α: released = α prot, full aft = α max. Auto-trim stops, bank limit 45°. |
| Alpha floor | Between α prot and α max, or full aft stick in α protection; from liftoff to the flare, below M 0.6. Sends A.FLOOR to the A/THR (TOGA). |
| High speed | From VMO+6 / MMO+0.01, anticipated with the 5 s speed trend. Nose-down authority fades to zero by VMO+16 / MMO+0.04 and a nose-up demand grows. Trim frozen; bank limit 40°, a released stick returns wings level. |

α prot / α max are port values below the installed model's 15° stall-warning AoA (X-Plane body AoA):

| FLAPS | 0 | 1 | 2 | 3 | FULL |
|---|---|---|---|---|---|
| α prot | 8.5° | 11° | 11° | 11° | 10.5° |
| α max | 11° | 14° | 14° | 13.5° | 13° |

Above M 0.5 both are reduced by 6° per Mach.

## Roll and yaw

* **Sidestick** = roll-rate demand, maximum 15°/s, with a 0.25 s lag and a 15°/s² limit.
* **Stick released**: the bank is held up to 33°; beyond 33° the aircraft returns to 33°.
* **Stick held beyond 33°**: spiral stability adds a counter-rate, so full stick ends at 67°.
* **Bank limit**: 67°, or 45° in high-AoA protection and 40° in high-speed protection.
* **Yaw**: the pedals act directly, with a yaw damper and turn coordination (yaw rate g·sinφ/V) added in
  flight. The damper is off on the ground and during the autoland's rudder phase.

## Direct law

Without valid IR attitude or ADR air data on both sides, the law is direct: the stick passes straight
through, there are no protections and manual trim is available.

## Steering

| Input | Authority |
|---|---|
| Hand wheel (tiller) | ±70°: full up to 20 kt, zero at 70 kt. |
| Rudder pedals | ±6°: full up to 40 kt, zero at 130 kt. |
| Pedals without a hardware tiller | Also steer the full ±70° up to 10 kt, blended out by 30 kt (keyboard or twist-grip pilots). |
| Body-wheel steering | Opposite to the nose gear: once the nose wheel is beyond 20° below 20 kt, up to 15° at 70°. |

The pilots' figures for the A380 are about 70° by tiller and 6–7° by pedals, with the rudder doing the
work above about 60 kt.

The actuators are rate limited: nose gear 20°/s, body gear 5°/s. In flight the nose wheel is centred.

A hardware tiller is an axis assigned to X-Plane's **Nosewheel tiller**. Keyboard users can bind the held
commands `a380x/xp12/steering/tiller_left` and `tiller_right`: the virtual tiller moves at 1.5/s and
springs back to centre.

## Datarefs and commands

| Kind | Name | Meaning |
|---|---|---|
| Command | `a380x/xp12/fbw/toggle` | Laws on/off (off = X-Plane's direct controls). |
| Command | `a380x/xp12/steering/toggle` | Steering on/off (off = X-Plane's own steering). |
| Command | `a380x/xp12/steering/tiller_left`, `tiller_right` | Hold to turn the keyboard tiller. |
| int | `a380x/xp12/fbw/enabled` | 1 = the module runs. |
| int | `a380x/xp12/fbw/normal_law` | 1 = normal law (read by A380XTrim). |
| int | `a380x/xp12/fbw/law` | 0 off, 1 normal, 2 direct. |
| int | `a380x/xp12/fbw/mode` | 0 ground, 1 flight, 2 flare. |
| int | `a380x/xp12/fbw/protections` | Bits: 1 α, 2 high speed, 4 pitch attitude, 8 bank, 16 load factor, 32 α floor. |
| int | `a380x/xp12/fbw/auto_trim` | 1 = auto-trim active. |
| int | `a380x/xp12/steering/active` | 1 = the steering override is owned. |
| float | `a380x/xp12/fbw/nz_command` | Load-factor demand, g. |
| float | `a380x/xp12/fbw/pitch_rate_command`, `roll_rate_command` | Rate demands, deg/s. |
| float | `a380x/xp12/fbw/alpha_prot`, `alpha_max` | Current α limits, deg. |
| float | `a380x/xp12/steering/nose_deg`, `body_deg` | Steering angles, deg, + right. |
| float | `a380x/xp12/steering/tiller` | Tiller input −1..1. |

Law changes and protection entries are written to `Log.txt` and to `Output/A380X-XP12_AFS.log`.

## Verification and limits

`test_fctl` covers the modes, AP bypass and handover, direct law, protections and the steering
schedule. Its closed-loop bench flies a hand-written heavy-jet airframe, not X-Plane's flight model.
Every case is re-flown at half, nominal and double control power:

* hands-off at 140 (flaps), 150, 250 and 320 kt;
* a 0.3 pull, full aft, full aft at low energy and full forward;
* a VMO dive;
* bank hold, full roll, return to 33° and a 30° level turn.

`test_fctl_adapter` checks the X-Plane side against a mock XPLM.

The gains have not been flown in X-Plane yet. If the aircraft feels too soft, too twitchy or
oscillates, the `A380X_FCTL_*` gain constants in `a380x_fctl.h` are the place to tune.

Not modelled:

* alternate law (only normal and direct);
* the sideslip-demand rudder law;
* engine-failure compensation;
* low-energy (SPEED SPEED SPEED) and windshear protection;
* per-PRIM redundancy.
