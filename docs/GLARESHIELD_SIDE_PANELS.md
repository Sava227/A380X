# Glareshield side panels

The aircraft-local `A380XAvionics` plugin owns the captain and first-officer
side-panel controls. The installed aircraft is
`output/A380X-XP12`; the local X-Plane 12 `Aircraft/A380X-XP12` path is a
symlink to that package. The implementation is in
`native/avionics/a380x_glareshield.inc`, with the physical mesh mappings in
`config/cockpit_controls.json`.

- Each CHRONO button cycles off → running → frozen → off. Its own ND shows the
  timer only while running or frozen.
- The TAKEOVER buttons set or cancel a priority condition when the opposing
  stick is moving. Green priority indications flash for simultaneous motion
  without priority. Red arrows and green winner lights follow priority state.
  The buttons also disconnect the custom AP guidance. Voice callouts are played
  through X-Plane's interior audio bus.
- MASTER WARN flashes; MASTER CAUT is steady. Both use X-Plane's master
  annunciator inputs. Pressing either side acknowledges its own severity and
  holds X-Plane's clear command through the next flight-loop frame.
- The AUTOLAND warning checks both AP channels, LOC and glide deviations or
  signal loss, and a 15 ft radio-altimeter disagreement below 200 ft RA.
  Relevant PFD scales and flight-director bars flash. A new cause triggers
  three clicks.
- ATC MSG flashes on both sides. A normal message rings after 10 seconds;
  an urgent message rings on the next update. Reading/acknowledging cancels
  the light and sound. Flight phases 3–7 and 9–10 inhibit the alert.
- The speaker knobs adjust the generated panel sound level. RADIO PTT holds
  `xpilot/ptt` while pressed if xPilot is installed and exposes that command.

## Adapter inputs

The current port has no dual-sidestick authority computer or DCDU message
producer. The plugin exposes the following inputs for those systems as they
are ported. The LAND window is produced by the native guidance
([AUTOLAND.md](AUTOLAND.md)) and is read-only:

| Input | Type | Meaning |
| --- | --- | --- |
| `a380x/xp12/glareshield/stick_left_moving` | writable int | Captain stick outside neutral |
| `a380x/xp12/glareshield/stick_right_moving` | writable int | First-officer stick outside neutral |
| `a380x/xp12/glareshield/land_mode` | read-only int | LAND engaged with an AP in it, airborne, before ROLL OUT |
| `a380x/xp12/glareshield/atc_receive_normal` | command | New normal ATC message |
| `a380x/xp12/glareshield/atc_receive_urgent` | command | New urgent ATC message |
| `a380x/xp12/glareshield/atc_ack_left` / `atc_ack_right` | commands | Message read or acknowledged |

The physical buttons, annunciators, audio and both ND/PFD presentations are
installed. Sidestick priority currently changes the side-panel state and
AP disconnect but cannot arbitrate independent control inputs until a dual
sidestick source is added. `land_mode` follows the guidance LAND mode once an
AP has been engaged in it and ends at ROLL OUT, so an AP disconnect after
touchdown does not flash the light. No CPDLC receiver is connected, so ATC reception commands
must be called by an adapter. RADIO PTT requires the xPilot command at runtime.
The new signals are also published as read-only `a380x/xp12/glareshield/*`
datarefs for cockpit indications and external hardware.

`native/avionics/build.py` compiled and ran its native tests for macOS and
Windows. `tools/validate_aircraft.py` passed on the installed output package.
Visual placement, plugin startup, audio and flight behavior still need a live
X-Plane check.
