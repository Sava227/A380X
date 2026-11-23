# Spoilers and ailerons v2 on the GitHub c84f813 wings (installed 2026-11-14)

> Written for the local install. `tools/wing_surfaces.py` is in `tools/`, the A380XDoors source in `native/doors/`; `original/` refers to the local backup folder.


## Base wings

- The four wing objects come from GitHub Sava227/A380X `c84f813` (PR #16, slat skin repair, `SLAT_SKIN_REPAIR.md`).
  - Downloaded from the LFS media URLs. The sha256 values match the LFS oids.
  - Their vertices, indices and spoiler/aileron blocks are identical to d3cfbf2. Only the slat draw ranges were regrouped.

## Changes

- **Wing objects** (`tools/wing_surfaces.py`, all 6 files in `objects/exterior/wings`):
  - every spoiler segment rotation now reads its A380 panel, `a380x/xp12/surfaces/spoiler_deg[0-15]` (left panels 1-8, right panels 1-8, inboard to outboard);
  - each aileron gets an extra rotation about its hinge, driven by `a380x/xp12/surfaces/aileron_ground_up` (0..1 = 0..40 deg trailing edge up; 40 is the ACF ground speed-brake aileron value).
- **A380XDoors plugin** (source `../door-timing-src/A380XDoors.c`):
  - **Panel deflection:** the larger of the PRIM panel order and the roll spoiler deflection.
    - The PRIM order comes from the avionics speedbrake module (`a380x/xp12/speedbrake/spoiler_N_deg`): speed brake, ground spoilers at touchdown when armed, and 5 deg spoiler lift on the ground with the flaps retracted.
    - The roll part is X-Plane's wing 4/5 roll spoiler deflection minus its speed-brake share, on panels 2-8.
  - **Ailerons up:** follows the ground spoiler command (lift dumping).
  - **Hydraulics:** surfaces move only with hydraulic pressure (900/450 psi hysteresis). With no pressure, for example engines off, they hold their position, so the 5 deg lift needs an engine running.
  - **Rate limit:** 50 deg/s.

## Aerodynamics

The aerodynamics are unchanged. X-Plane already deflects the spoilers and ailerons as speed brakes from the handle that the avionics speedbrake module drives.

## Undo

With X-Plane closed, copy `original/` back. That restores the d3cfbf2 wings and the previous A380XDoors.
