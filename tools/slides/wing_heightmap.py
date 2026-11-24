"""Top and bottom height maps of the wings, engines, pylons and belly fairing beside the fuselage (Blender ray casts,
0.25 m grid). Usage: blender -b -P wing_heightmap.py -- out.json <dir of Wavefront conversions>"""
import bpy, sys, json, glob, bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
argv = sys.argv[sys.argv.index('--') + 1:]
out, d = argv[0], argv[1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bm = bmesh.new()
for f in sorted(glob.glob(d + '/*.obj')):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=f, forward_axis='Y', up_axis='Z')
    for o in [o for o in bpy.data.objects if o not in before]: bm.from_mesh(o.data)
t = BVHTree.FromBMesh(bm)
xs = [round(2.0 + 0.25 * i, 2) for i in range(97)]
xs = [-x for x in reversed(xs)] + xs
zs = [round(-32 + 0.25 * i, 2) for i in range(265)]
H, L = [], []
for z in zs:
    row, low = [], []
    for x in xs:
        loc, n, i, dist = t.ray_cast(Vector((x, -z, 3.6)), Vector((0, 0, -1)), 12.0)
        row.append(None if loc is None else round(loc.z, 3))
        loc, n, i, dist = t.ray_cast(Vector((x, -z, -7.0)), Vector((0, 0, 1)), 10.6)
        low.append(None if loc is None else round(loc.z, 3))
    H.append(row); L.append(low)
json.dump({'x': xs, 'z': zs, 'top': H, 'bottom': L}, open(out, 'w'))
print('faces', len(bm.faces), 'done', flush=True)
