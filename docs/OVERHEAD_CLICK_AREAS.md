# Overhead: bigger pushbutton click areas (2026-11-15)

## Click areas (`tools/ovhd_click_areas.py`)

The overhead pushbuttons' invisible click squares were about 17 × 17 mm, smaller than the buttons. All 171 overhead pushbutton click areas are now enlarged; that covers the overhead lamps panel, ELEC, ADIRS, the native-toggle buttons and the cockpit pushbuttons in the overhead area.

- **How each area is grown:** in its own plane, about its own centre, to the full button field (27 × 27 mm). It stops 0.4 mm short of the half-way line to a neighbouring control on the same panel, and never gets smaller than before. A control stacked on its button (a guard) doesn't count as a neighbour.
- **Results:** median area 299 → 702 mm². Tightly packed rows (22.5 mm apart, for example ENG ANTI ICE 1-4, GEN beside DRIVE, FEED pumps) get about 20 × 27 mm, which is their whole field.
- **What changed in the file:** only vertex positions, 684 VT lines. Commands, tooltips and animations are unchanged. Click areas inside a button-press animation are edited in that animation's own coordinates, so they still move with the button.
- **Map:** `build/click_map.png` shows the overhead flat, old area red and new area green.

## Rebuilding the cockpit object

`tools/ovhd_clicks/ovhd_click_areas.py <in cockpit_controls.obj> <out.obj> <report.json>` regenerates the larger click areas; re-run it after any rebuild of `a380x_cockpit_controls.obj` (it was re-applied on top of PR #19's lit legends: same 171 spots and sizes, only 684 VT lines changed).

(A legend glow tried at the same time was reverted; PR #19's lit legends replace it.)
