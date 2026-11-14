# Complete slat skin animation

The installed wing objects previously left mixed-weight slat triangles in fixed
draw groups. Deploying the slats separated their moving and stationary surfaces.
The four repaired wing objects move each complete slat assembly under one fitted
transform shared by its two material objects.

The correction rehomes 46 existing TRIS draw ranges covering 57,504 triangles.
Of those triangles, 42,604 previously remained stationary. Vertex positions,
normals, UVs, index tables and texture directives retain their original bytes.
The existing `a380x/xp12/flaps/slat_anim_ratio` drives the repaired panels.

## Reference and accuracy

The reference mesh and its authored slat clips come from FlyByWire's A380 wing
model at commit `01847d956061b90ec13fe99c6668a7d377708bf5` in
[aircraft-large-files](https://github.com/flybywiresim/aircraft-large-files/tree/01847d956061b90ec13fe99c6668a7d377708bf5/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/attachments/flybywire/Part_Exterior_Wings/model).

An offline comparison covered 41 authored ratios plus the CONF 1 ratio
`286.48 / 327.39`. Maximum deployment mismatch fell from 762.6 mm to 15.2 mm.
One rigid transform per assembly approximates the original weighted deformation.
The existing fixed wing-flex pose and FPPU input mapping are retained.

## Validation

The staged OBJ hashes match the candidate used in the manual X-Plane test.
The user confirmed slat movement and supplied retracted and fully extended views.
The visible wing section kept its complete skin together during extension and
seated on retraction. Clearance at the pylons, the full slot and the remaining
wing sections still need visual inspection.

Static verification checks preserved geometry payloads, draw-range coverage,
neutral geometry, slat binding, and unrelated control transforms. The source
pose comparison and runtime screenshots establish the scope of this repair.
Exact Airbus AMM slat transit-time acceptance limits remain unverified.

## Manual check

Park with the parking brake set, engines idle and green hydraulic pressure
available. At simulation speed 1x, compare CONF 0, 1+F, 3 or FULL, and 0 again.
Wait for movement to finish at each setting. Observe both leading edges from
the same camera and inspect panel seams, pylon clearances, wingtip ends,
flickering and final seating. The default `2` key extends the flap lever one
detent and `1` retracts it one detent. See [FLAPS_LEVER.md](FLAPS_LEVER.md)
for the existing control logic and modeled timing.
