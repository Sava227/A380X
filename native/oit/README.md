# A380X OIT (Onboard Information Terminal) - standalone plugin

Installed 2026-11-12 into `Aircraft/A380X-XP12 Latest`:
- `plugins/A380XOIT/{mac_x64,win_x64}/A380XOIT.xpl` (this source, `./build.sh`).
- Two cockpit-device quads `a380x.xp12.oit.captain` / `.first_officer` added to
  `objects/interior/cockpit-panels/a380x_mfd_screens.obj` by `tools/add_oit_quads.py`.
  - They sit exactly on the OIT glass of `screen_powered_off_display_faces.obj`.
  - Lighting channels 26 and 27.
  - The original object is in `../oit-install-20261112/original/`.

Undo (X-Plane closed): delete `plugins/A380XOIT` and copy the original screens object back.

## Behaviour

The design follows the FlyByWire A380X OIT (`fbw-a380x/src/systems/instruments/src/OIT`) and the Airbus OIS
description (EASA A380 EFB/OIS evaluation, ATA 46).

- **Power and start-up**
  - OIT knob 0 = off.
  - On a start with the engines running, the OIT counts as on until the knob is first moved.
  - Power-on runs a 10 s self test (black screen).
- **Domains:** OIT SIDE selects FLT OPS or NSS AVNCS.
  - FLT OPS needs that side's laptop. Use the laptop POWER switch; laptops are on automatically at a runway start.
  - Each domain shows a loading screen and then a PILOT login.
- **FLT OPS**
  - MENU and FUNCTIONS dropdowns, a message box and a switching bar.
  - FLT OPS MENU in four columns.
  - STATUS PAGE: FLT NUMBER, FROM and TO entry. SYNCHRO AVIONICS fills FROM with the nearest airport on the ground and TO with the FMS destination.
  - T.O PERF, LOADSHEET and LDG PERF cross-check the MFD/FMS entries; there is no performance database.
  - IN-FLT PERF: fuel by tank and endurance.
  - OIS XFR: X-OIS mirror of the other OIT.
- **NSS AVNCS**
  - Folder-tree menu.
  - COMPANY COM:
    - Inbox: generated MVT messages and the session-opened message.
    - Flight Log: OUT/OFF/ON/IN with FOB, using FBW logic.
  - UTILITIES > A380X SYSTEMS: current faults (X-Plane failures), doors and engines.
- **Input**
  - OIT keyboard and ACD keys: commands `a380x/xp12/oit/<side>/{kbd,acd}/*`, passed through by the cockpit feedback.
  - Mouse clicks on the screen.
  - In the pop-out window, the PC keyboard. The window is toggled with `a380x/xp12/oit/<side>/popup`.

## Layout

| Path | Contents |
|---|---|
| `src/oit_core.c` | Pages, state, flight log; no X-Plane dependency |
| `src/oit_xplm.c` | Devices, keys, datarefs |
| `src/oit_draw.c` | Drawing primitives and atlas text |
| `src/oit_gl.c` | OpenGL backend |
| `src/oit_soft.c` | Software backend, used for tests |
| `tools/make_font.py` | Font atlas: Manrope and JetBrains Mono, SIL OFL (`fonts/OFL.txt`) |
| `test/test_oit.c` | Offline tests; page renders in `test/out/*.png` |
