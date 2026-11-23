import bpy, sys, math, json, glob, bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
argv = sys.argv[sys.argv.index('--') + 1:]
out, ind, exd = argv[0], argv[1], argv[2]
def tree(files):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bm = bmesh.new()
    for f in files:
        before = set(bpy.data.objects)
        bpy.ops.wm.obj_import(filepath=f, forward_axis='Y', up_axis='Z')
        for o in [o for o in bpy.data.objects if o not in before]:
            bm.from_mesh(o.data)
    return BVHTree.FromBMesh(bm), len(bm.faces)
ti, ni = tree(sorted(glob.glob(ind + '/*.obj'))); te, ne = tree(sorted(glob.glob(exd + '/*.obj')))
print('faces', ni, ne, flush=True)
Y0 = 5.0
zs = [round(-22 + 0.1 * i, 2) for i in range(531)]
As = list(range(-112, 113, 2))
res = {'y0': Y0, 'z': zs, 'a': As, 'din': [], 'kind': [], 'dex': []}
for z in zs:
    rd, rk, re_ = [], [], []
    for a in As:
        ar = math.radians(a)
        o = Vector((0.0, -z, Y0)); dv = Vector((math.sin(ar), 0.0, math.cos(ar)))
        start, travelled, kind, d = o, 0.0, 0, -1
        for _ in range(20):          # X-Plane culls back faces: look through them
            loc, nrm, idx, dist = ti.ray_cast(start, dv, 8.0 - travelled)
            if loc is None: break
            travelled += dist
            if nrm.dot(dv) < 0: kind, d = 1, travelled; break
            start = loc + dv * 0.001; travelled += 0.001
        rd.append(round(d, 4)); rk.append(kind)
        loc, nrm, idx, dist = te.ray_cast(o, dv, 8.0)
        re_.append(-1 if loc is None else round(dist, 4))
    res['din'].append(rd); res['kind'].append(rk); res['dex'].append(re_)
json.dump(res, open(out, 'w'))
print('done', flush=True)
