# Landing gear: spinning wheels and steering (installed 2026-11-24)

## What changed

**Spin:** all 22 wheels spin with `sim/flightmodel2/gear/tire_rotation_angle_deg[n]`.
- **Gear indices:** 0 nose, 1/2 wing L/R, 3/4 body L/R.
- **Direction:** forward roll is a positive rotation about object +x, because the gear objects are attached yawed 180°. Laminar's A330 uses the same sign.
- **Tyre radii:** the acf radii match the model (nose 0.68 m, mains 0.714 m), so the wheels don't slip visually.

**Nose steering:** the lower strut and both wheels turn about the strut axis.
- **Turning parts:** steering collar, gland, both torque links, sliding piston, axle housing and the parts below the axle.
- **Fixed parts:** main fitting, drag brace, steering actuators and the taxi-light housings.
- **Axis:** fitted from the piston and main-fitting rings, raked 3.55°, with 13 cm of trail at the axle.

**Body wheel steering:** each body gear's aft axle (axle beam plus its two wheels) turns about the vertical through the axle centre. Nothing else on the bogie moves; the steering actuator stays static.

**Fix:** FlyByWire's model rests at full body wheel steering, so the installed aft axles were permanently turned about 15° right (15.18° left gear, 15.40° right gear). The steering keys include that offset, so the axles are now straight at 0.
- Retracted, the straight aft wheels are no wider than the other axles (x 1.664–3.590 m). The turned ones stuck out 12 cm further each side.

**Steering dataref:** the plugin `A380XDoors` (Mac and Windows) now publishes `a380x/xp12/gear/steer_vis_deg[5]`, in degrees, positive right.
- **On the ground:** X-Plane's enacted steering (`tire_steer_actual_deg`).
- **In the air:** a gear that leaves the ground centres within 1 s, as the A380's nose and body wheel steering do. Its wheels never turn in flight or in the bay.
- **Wing gears:** always 0.
- **Body gears:** the angle is X-Plane's own body-gear steering (acf: 9° at low speed, 7° at high speed).

## Files

**Objects** (`objects/exterior/`):
- `gear-nose/gear_nose_a380_exterior_tyres.obj`, `gear_nose_a380_exterior_nlg.obj`
- `gear-wing-lh/gear_wing_lh_a380_exterior_tyres.obj`, `gear-wing-rh/gear_wing_rh_a380_exterior_tyres.obj`
- `gear-body-lh/gear_body_lh_a380_exterior_tyres.obj`, `gear_body_lh_a380x_blg.obj`
- `gear-body-rh/gear_body_rh_a380_exterior_tyres.obj`, `gear_body_rh_a380x_blg.obj`

Only ANIM blocks were added, around 26 draw calls. Vertices and indices are unchanged.

**Plugin:** `plugins/A380XDoors/{mac_x64,win_x64}/A380XDoors.xpl`.
- Source: `door-timing-src/A380XDoors.c`. The previous version is `A380XDoors.c.v5-slides`.
- The same source is in the worktree `avionics-src-20261115/native/doors/` (not committed).

## Tools

- `tools/gear_wheels.py <objects/exterior> <out>` writes the objects and `wheels_report.json`. Rules are geometric; only the 8 nose-strut draw calls are listed by offset and count.
- `tools/check_wheels.py <original> <new>` checks that:
  - the neutral pose is unchanged at deploy 1, 0.5 and 0, apart from the aft axles;
  - every wheel spins in place;
  - steering by +8° or +30° turns the wheels by exactly that, to the right.
- `tools/test_gear_main.c` + `test_gear_mock.c` is the offline plugin test: ground tracking, ±90° clamp, centring in 1 s after lift-off, NaN input.
- `renders/` holds the before/after previews (Blender, `render_multi.py` + `xp2wave3.py` with `XPV`).

## Checks

- `check_wheels.py`: ALL OK.
- `validate_obj`: OK for all 8 objects.
- Plugin builds with `-Werror` (Mac + Windows); offline test passes.
- **Not yet tested in X-Plane.**

# Update 2026-11-24: main gear bogie tilt (nose gear unchanged)

## Behaviour

**In flight:** the four main gear bogies (wing and body) pitch about their pivot pins, rear wheels down, with `a380x/xp12/gear/bogie_tilt_deg[n]`. The angle is in degrees relative to the airframe; + means rear down.

**Plugin (`A380XDoors`):**
- **In flight, gear down:** the bogie tilts to 10° (`BOGIE_TILT_MAX_DEG`) after lift-off, at 8° per second.
- **Near the ground:** each main gear's height comes from `y_agl`, pitch, roll and X-Plane's wheel position (`tire_*_no_deflection`). On the ground it is calibrated against the strut deflection over 1 s.
  - The tilt is limited so the rear wheels never go below the runway. On approach the rear wheels touch first and the bogie levels as the aircraft settles.
- **On the ground:** the bogie follows the runway. During the takeoff rotation it stays flat, at minus the pitch relative to the airframe.
- **Gear travel:** the tilt fades out over the last 15 % of gear travel, so the gear stows exactly as modelled.
- **Diagnostics:** `a380x/xp12/gear/height_m[5]` shows the computed wheel heights.

**Bogie definition:** each bogie is the node holding its tyres, plus that node's children, minus the sliding piston (which shares the wing gear's bogie node).
- **Tilted:** 11 calls (wing left), 9 (wing right), 39 per body gear.
- **Fixed:** the piston, torque links, pitch-trimmer linkage and leg.

**Pivots** are fitted from the pins on the piston forks:

| Gear | x | y | z | Position |
|---|---|---|---|---|
| Wing L | 6.142 | -4.061 | 1.764 | midway between the axles, 0.87 m from each |
| Wing R | -6.142 | -4.063 | 1.764 | midway between the axles, 0.87 m from each |
| Body L | 2.631 | -4.015 | -1.863 | at the middle axle; front and aft axles 1.66 m away |
| Body R | -2.629 | -4.011 | -1.874 | at the middle axle; front and aft axles 1.66 m away |

**Levelling:** FlyByWire left the right wing bogie 1.8° front-down while the left one is level. The tilt keys now level it at 0. Bogies off level by less than 0.5° are left alone.

**Order of the rotations:** tilt is outermost, then steering, then spin. The aft axle steers and the wheels spin inside a tilted bogie; with 10° tilt and 8° steering the aft wheels stay on the axle within 1 cm.

## Checks

- **`tools/gear_wheels/check_wheels.py <new objects> <original objects>`:** ALL OK, 75 checks.
  - At 10° tilt each axle moves by exactly the rotation about the pin: body ±0.288 m, wing ±0.15 m.
  - Pistons, links and legs stay put.
- **`tools/test_bogie_main.c`** (offline, mock XPLM) covers:
  - taxi gives 0;
  - rotating at 9° gives −9;
  - climbing reaches 10;
  - retraction fades the tilt out;
  - on approach, the rear axle clears the runway (≥ 0) all the way down;
  - touchdown at 4° nose-up gives −4.
- **Steering test:** `test_gear_main.c` still passes.
- **Builds:** the plugin builds with `-Werror` for Mac and Windows.
- **Build-flag fix:** the first wheels build passed its `-DXPLM...` flags as one shell argument under zsh; it worked anyway. This build passes each flag separately.
- **Preview:** `renders/bogie_tilt_preview.png`.
- **Not yet tested in X-Plane.**

# Fix 2026-11-24 (evening): the bogies tilt front wheels down

The first tilt version hung the bogies the wrong way (rear wheels down). On the real A380 the wing and body gear bogies hang **front wheels down** in flight, as a photo of an A380 on approach shows, so the front axle touches first. Only the A380XDoors plugin changed. The gear objects already rotate both ways (keys −30…+30), and the dataref convention is unchanged (+ = rear wheels down), so the plugin now sends negative values.

- **In flight, gear down:** −10° (`BOGIE_TILT_FLIGHT_DEG`), front wheels down. The pitch trimmer moves the bogie at 8° per second while it is free of the ground.
- **Near the ground:** the bogie's angle to the ground is limited so that neither the front nor the rear wheels go below the runway (`BOGIE_TILT_LIMIT_DEG` ±15° overall). Ground limits apply at once.
- **On approach** at 4° nose up (−6° to the ground), the front axle reaches the runway first. The bogie then levels as the aircraft settles: −4° (flat on the runway) at touchdown, and 0° once the nose wheel is down.
- **Takeoff:** during rotation it stays flat on the runway (−pitch). After lift-off it goes to −10°, a small move from about −9°.
- **Go-around:** from the runway it returns to −10° at 8° per second.

**Checks** (`native/doors/test/test_bogie_tilt.c`, offline with a mock XPLM; slides and steering tests also pass):
- The test descends the body and wing gears to 0 m. The front axle never goes below the runway, and it is the lowest point until touchdown.
- The front axle limit at 5 cm gives −5.73°, which is −asin(0.05/1.66) − 4.
- The plugin builds with `-Werror` for Mac and Windows.
- **Preview:** `Output/A380X-XP12 backups/bogie-tilt-flip-20261124/preview.png`.
