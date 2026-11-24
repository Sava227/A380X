import bpy, sys, os, math, colorsys
argv = sys.argv[sys.argv.index('--') + 1:]
prefix = argv[0]; cams = []; files = []
for a in argv[1:]:
    if a.startswith('cam='): n, loc, rot = a[4:].split(':'); cams.append((n, [float(x) for x in loc.split(',')], [float(x) for x in rot.split(',')]))
    else: files.append(a)
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
legend = []
for i, f in enumerate(files):
    col_name = None
    if '@' in f: f, col_name = f.split('@')
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=f, forward_axis='Y', up_axis='Z')
    new = [o for o in bpy.data.objects if o not in before]
    h = (i * 0.618) % 1.0
    col = colorsys.hsv_to_rgb(h, 0.55, 0.9) + (1,)
    if col_name == 'grey': col = (0.6, 0.6, 0.6, 1)
    mat = bpy.data.materials.new(os.path.basename(f)); mat.diffuse_color = col
    mat.use_backface_culling = True
    for o in new:
        o.data.materials.clear(); o.data.materials.append(mat)
    legend.append((os.path.basename(f), [round(c, 2) for c in col[:3]]))
print('LEGEND', legend, flush=True)
scene.render.engine = 'BLENDER_WORKBENCH'
scene.display.shading.light = 'STUDIO'; scene.display.shading.color_type = 'MATERIAL'
scene.display.shading.show_backface_culling = True
scene.render.resolution_x = 960; scene.render.resolution_y = 600
w = bpy.data.worlds.new('w'); scene.world = w; w.color = (0.55, 0.75, 1.0)
scene.display.shading.background_type = 'WORLD' if hasattr(scene.display.shading, 'background_type') else None
cam = bpy.data.cameras.new('cam'); cam.lens = float(os.environ.get("LENS", "14"))
co = bpy.data.objects.new('cam', cam); scene.collection.objects.link(co); scene.camera = co
for n, loc, rot in cams:
    co.location = loc; co.rotation_euler = [math.radians(a) for a in rot]
    scene.render.filepath = f'{prefix}_{n}.png'
    bpy.ops.render.render(write_still=True)
