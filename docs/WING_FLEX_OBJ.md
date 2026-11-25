# Wing flex in the installed wing objects (2026-11-25)

PR #22 put FBW's wing-flex model in the avionics plugin, which publishes
`a380x/xp12/wing/flex_<left|right>_<inboard|inboard_mid|outboard_mid|outboard>`, and mapped the eight flex clips
for a wing re-export. The wing objects were **not** re-exported: `tools/wing_flex/wing_flex_obj.py` adds the bend to
the installed objects instead. This document explains why, and what the tool does.

## Why not re-export

1. **The skin would tear.** OBJ8 cannot skin. `rigidify_skinned_meshes` moves only faces whose vertices all
   belong ≥ 99.99 % to one bone; blended faces stay frozen.
   - `tools/wing_flex/flex_skin_census.py` on FBW's `A380_WINGS.gltf` (5f07408) gives 66,005 wing triangles rigid on
     one bone, 84,360 blended across flex sections and 15,528 blended within one, out of 172,395 with flex
     influence.
   - About 58 % of the skin would therefore stay frozen while the rest bends, which shows as cracks along the
     whole wing in flight. `bake_static_skin_pose` only avoided this because flex was static.
2. **Hand fixes would be lost.** The installed wing OBJs carry edits made after export:
   - the slat skin repair (PR #16, `docs/SLAT_SKIN_REPAIR.md`), which has no tool to redo it;
   - the spoiler and aileron v2 rewiring from 2026-11-14.

## What the tool does

**Rig.** FBW bends each wing with a chain of bones: `WING_FLEX_3` to `WING_FLEX_15`, root to tip. Four clips per side
drive them:

| Clip | Bones | Dataref |
|---|---|---|
| inboard | 3-5 | `flex_*_inboard` |
| inner midboard | 6-9 | `flex_*_inboard_mid` |
| outer midboard | 10-13 | `flex_*_outboard_mid` |
| outboard | 14-15 | `flex_*_outboard` |

- Each bone only rotates, with no translation above 4 µm. It turns about one fixed axis (within 0.003°), roughly
  along the local chord, by an angle that is close to linear in the dataref.
- The tool samples each clip at 9 dataref values across `value_min`..`value_max` from `config/animation_datarefs.json`.
  It takes the change of the bone's local rotation relative to the zero-flex frame, expressed in its parent's
  zero-flex world frame. That zero-flex frame is the one the installed objects were frozen at (the old
  `static_fraction`), and it is also what the datarefs give at 0.

**Coordinates.** The exterior objects use the glTF frame as is (x left, y up, z forward; the ACF turns them by 180°),
so FBW's pivots and axes are used unchanged.

**Segments.** The wing is cut into one rigid segment per bone:
- Each cut is a vertical plane through the bone's pivot that contains its bend axis, so the two sides of a cut move
  together near the axis.
- Nothing within 6 m of the centreline moves. That is where FBW's own skin weights start bending.
- A segment on bone k nests the rotations of bones 3..k, outermost first:
  `T(p3) R3 T(p4-p3) R4 ... Rk T(-pk)`, which is exactly the rig's change of world transform for bone k.

**Assignment.**

| Geometry | How it is assigned |
|---|---|
| Engines, cores, fans and pylons | The bone FBW attaches them to: inboard `WING_FLEX_6`, outboard `WING_FLEX_11`, split at \|x\| = 20 m. |
| Wing skin, rigid fragments, moving panels, wing fence | Cut exactly along the planes; each piece goes to the segment that contains it. Panels are cut at their retracted position. |

- New vertices interpolate position, normal and UV on the cut edge. Existing vertices are untouched.
- Skin within 10 cm of a cut is drawn on both sides of it. This covers the opening under or over the bend axis; at
  zero flex the two copies coincide exactly.

**Draw calls.** Every draw call becomes one ANIM block per segment: the flex chain outermost, then the call's
original animation stack, unchanged. The slat, spoiler and aileron animations ride inside.

| | Before | After |
|---|---|---|
| Draw calls (11 objects) | 525 | 776 |
| VT added | — | 50,616 |
| Size of the 11 objects | 91.6 MB | 99.6 MB |

## Checks (`tools/wing_flex/verify_flex.py`)

**At zero flex:**
- Every new triangle lies in its source triangle (within 0.4 µm).
- Each source triangle's area is preserved (within 2e-7 m²).
- All triangles are kept.

**Against FBW's own rig at the same flex:** with every dataref at +1.6, the engines, cores, fans, pylons and wing
fence follow FBW within 1.5 cm. The skin follows within about 5 cm at p95; FBW's skin is blended, ours bends in
segments.

**Realistic values from the plugin's model** (`test_wing_flex`, values for links 1-4):

| Case | Link values (deg) | Wingtip | Largest opening at a cut |
|---|---|---|---|
| Parked, full fuel | -0.33, -0.52, -0.65, +0.07 | about 1.0 m lower | 0.8 cm |
| 1 g cruise | +0.13, +0.25, +0.36, +0.88 | about 0.65 m higher | 0.6 cm |
| 2.5 g | +0.51, +1.50, +1.85, +1.92 | about 3 m higher | 2.0 cm |

The 10 cm band covers every opening.

**Format:** `tools/obj8_numbers.py --check` reports 0 numbers in exponent notation.

**Renders:**
- Front view: both wings bend up and the engines move with them.
- Close-up below the inboard trailing edge: slits along the cuts without the band, none with it (at +1.6 and +2.3).

## Limits

- The bend is piecewise: 13 rigid segments per wing, not FBW's smooth skin.
- At large flex, a step of 1-2 cm can be seen at the cuts on the thick root underside.
- Wing lights defined in the ACF do not move with the wing.
- A future wing re-export replaces these objects. Run `wing_flex_obj.py` again on its output, or teach the converter
  the same segmenting.

## Usage

```bash
python3 tools/wing_flex/wing_flex_obj.py --gltf <FBW>/Part_Exterior_Wings/model/A380_WINGS.gltf \
  --config config/animation_datarefs.json --out <dir> --report <dir>/report.json --perm <dir>/src \
  <aircraft>/objects/exterior/wings/*.obj
python3 tools/wing_flex/verify_flex.py <A380_WINGS.gltf> config/animation_datarefs.json <old static-flex mapping> \
  <installed wings dir> <dir> <dir>/src
```

Run it on wing objects that do **not** already have flex (the tool adds the chain; it does not detect an existing one).
