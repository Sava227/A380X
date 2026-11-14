import bpy, sys
from mathutils import Vector
argv = sys.argv[sys.argv.index('--') + 1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
for f in argv:
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=f)
    new = [o for o in bpy.data.objects if o not in before]
    pts = [o.matrix_world @ Vector(c) for o in new for c in o.bound_box]
    mn = [min(p[i] for p in pts) for i in range(3)]; mx = [max(p[i] for p in pts) for i in range(3)]
    print('BBOX', f.split('/')[-1], [round(x, 2) for x in mn], [round(x, 2) for x in mx], 'objects', len(new))
    if 'surrounds' in f:
        for o in new[:3] + new[12:17] + new[42:44]:
            ps = [o.matrix_world @ Vector(c) for c in o.bound_box]
            print('   ', o.name, [round(min(p[i] for p in ps), 2) for i in range(3)], [round(max(p[i] for p in ps), 2) for i in range(3)])
