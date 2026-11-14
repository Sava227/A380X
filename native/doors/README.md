# A380XDoors plugin

A small aircraft plugin (`output/A380X-XP12/plugins/A380XDoors`, Mac + Windows). It does three jobs:

- **Door timing:** sets `sim/flightmodel2/misc/door_cycle_time` from `door_timing.ini` (defaults: main deck 8 s, upper deck 8 s, cargo 20 s). Door indices: 0-9 main deck, 10-15 upper deck, 16-19 cargo / bulk.
- **Cabin shades and bins:** owns `a380x/xp12/cabin/shade[i]` / `bin[j]` (click targets, 0 or 1) and moves `shade_ratio[i]` / `bin_ratio[j]` toward them (shades 1.5 s, bins 1.2 s). The arrays hold 256 entries; the cabin uses 199 shades and 133 bins. Commands: `a380x/xp12/cabin/{shades_close_all, shades_open_all, bins_open_all, bins_close_all}`. See `docs/CABIN_V2.md`.
- **Spoiler and aileron visuals:** publishes `a380x/xp12/surfaces/spoiler_deg[16]` and `aileron_ground_up` from the PRIM panel orders, the roll spoilers and the ground spoiler command, gated on hydraulic pressure. See `docs/SPOILERS_AILERONS_V2.md`.

Build with `XPLM_SDK=/path/to/SDK ./build.sh`.
