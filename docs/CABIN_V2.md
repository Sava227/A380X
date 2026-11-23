# A380X cabin v2: shades on every deck, bins, doors, upper-deck ceiling (installed 2026-11-14)

> Written for the local install of "A380X-XP12 Latest". The generator scripts are in `tools/cabin_v2/`, the upper-deck door lining tools in `tools/upper_deck_doors/`; `original/`, `build/` and `source-unmodified/` refer to the local work folder.


This replaces the 2026-11-12 cabin work (`../cabin-bins-shades`). Each fix below lists the object it changed.

| Fix | Where |
|---|---|
| **Window shades on all decks, 199 in total.** Economy has 85: the model's own panels, now including the 12 angled ones in the tapering aft cabin that v1 missed. First class (main deck forward) has 24 and the upper deck has 90; neither had shade panels in the model, so one was made per window. | `cabin_a380x_main_deck_yc_sidewalls.obj`, `..._pec_sidewalls.obj`, `cabin_a380x_fc_deck_sidwealls.obj` (`tools/cabin_anim2.py`) |
| **Overhead bins no longer vanish when open.** The doors are single-sided, so an open door got culled. Each door now has an inner surface that moves with it. | yc / pec sidewalls |
| **Bin handles move with their doors.** Each handle is a separate 294-triangle part in the assets object; all 133 are now inside their door's rotation. | `cabin_a380x_main_deck_assets.obj` |
| **Upper-deck door windows travel with the door.** They were exported outside the door animation, so the glass stayed in the frame. | `a380x_pax_windows.obj` (`tools/doors_v2.py`) |
| **Upper-deck door handles turn, inside and outside.** The doors now stay shut for the first 12 % of the door ratio (about 1 s of the 8 s cycle) while the handles move: the inside lever swings 120° up, the outside handle swings 90° out. Then the doors slide exactly as before. Main-deck doors already did this and are unchanged. | `cabin_a380x_pax_doors.obj`, `cabin_a380x_upper_deck_door_surrounds.obj`, `a380x_fuse2.obj`, `a380x_fuse3.obj` |
| **Doors clickable inside and outside.** All 16 passenger doors have an invisible click box around the inside handle and another around the outside handle. Each box runs `sim/flight_controls/door_toggle_N` and follows its handle and door through the animation. The old fixed door spots are commented out (marked `A380X DOOR CLICK V1 RETIRED`); the cockpit door is untouched. | `a380x_cockpit_controls.obj` (`tools/cockpit_clicks_v2.py`) |
| **Upper-deck ceiling holes closed.** The model had no ceiling or upper wall in three places: around U1 (z 12.3 to 13.9), around U2 (z −6.6 to −3.2), and from the last windows back past U3 (z −20 to −12.8). The ceiling centre strip was also culled. These are now filled with a plain light surface. It sits 2 cm behind the surrounding panels and at least 4 cm inside the skin, never covers a window (it only goes where the skin is solid) or a door opening, and the gap strips along the left door edges move with the doors. | `cabin_a380x_fc_deck_sidwealls.obj` (`tools/roof_filler.py`, from `build/roof_cast.json` by `tools/roof_cast_cull.py`) |
| Plugin: shade arrays enlarged to 256 | `plugins/A380XDoors` (source `../door-timing-src/A380XDoors.c`) |

## How it was checked

- **Renders:** Blender renders of the objects with all animations evaluated:
  - bins closed and open;
  - shades raised and lowered on every deck;
  - upper doors at ratios 0, 0.12 and 1, inside and outside;
  - textured upper-deck views.
- **Click boxes:** compared against their handles at ratios 0, 0.12 and 1, for M1L and U1L, inside and outside.
- **Ceiling holes:** rays cast from the upper-deck centreline, looking through back faces as X-Plane does. Rays that reach the sky with fuselage skin behind them (not windows) between z −20 and 23.4: 7199 before, 85 after. All 85 are on the z −20 boundary row, behind the aft stairs.
- **Object structure:** every object passes `tools/validate_obj.py` (counts, index ranges, balanced animation blocks).
- **Plugin:** the plugin's mock test passes.

## Rebuild

1. Run `cabin_anim2.py`, with `source-unmodified/` (the four cabin objects as they were before any cabin work) and the aircraft folder for textures, into `build/`.
2. Run `roof_cast_cull.py` in Blender over cropped Wavefront conversions of the upper-deck interior and fuselage, producing `build/roof_cast.json`.
3. Run `roof_filler.py` on the fc object.
4. Run `doors_v2.py` on the aircraft (the installed objects at that time, saved in `original/`), into `build/doors/`.
5. Run `cockpit_clicks_v2.py` with `source-unmodified/a380x_cockpit_controls.obj`.

`install_list.txt` maps each build file to its place in the aircraft.

## Undo

With X-Plane closed, copy `original/` back over the aircraft. Those are the files exactly as they were before this install (including the v1 bins/shades and the A380XDoors plugin with 96 shades).

## Limits

- **Outside door handles:** they sit in the cockpit object like every click spot. X-Plane only sends clicks there from the 3D cockpit view, so move the head out to the door.
- **Door motion:** the doors keep X-Plane's door commands. Each click toggles the door, and the handle turns at the start of the motion.
- **New shades:** they are a plain light panel, using the closest colour each object's texture has to the economy shade.
- **Upper-deck outboard bins:** the openable bins are the 133 found in the main-deck objects, including the upper-deck centre bins. The upper-deck outboard bins were not examined.
- **Ceiling filler:** it is a smooth surface, not modelled panels. Next to the bins it continues the bin-bottom level.
