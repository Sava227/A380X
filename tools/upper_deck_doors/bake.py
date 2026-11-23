import bpy, bmesh, sys, json, math, time
argv = sys.argv[sys.argv.index('--') + 1:]
src, out_img, out_uv, size, samples = argv[0], argv[1], argv[2], int(argv[3]), int(argv[4])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=src, forward_axis='Y', up_axis='Z')
objs = sorted([o for o in bpy.data.objects if o.type == 'MESH'], key=lambda o: o.name)
# tag faces with their original index, then weld each part's split corners
for o in objs:
    me = o.data
    attr = me.attributes.new('orig', 'INT', 'FACE')
    attr.data.foreach_set('value', list(range(len(me.polygons))))
    bm = bmesh.new(); bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bm.to_mesh(me); bm.free()
    for p in me.polygons: p.use_smooth = True
# unwrap and pack all parts together
for o in objs: o.select_set(True)
bpy.context.view_layer.objects.active = objs[0]
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.uv.smart_project(angle_limit=math.radians(60), island_margin=0.0015, area_weight=0.0, correct_aspect=True)
bpy.ops.uv.pack_islands(rotate=True, margin=0.0015)
bpy.ops.object.mode_set(mode='OBJECT')
# bake ambient occlusion
img = bpy.data.images.new('ao', size, size, alpha=False, float_buffer=False)
mat = bpy.data.materials.new('bake'); mat.use_nodes = True
node = mat.node_tree.nodes.new('ShaderNodeTexImage'); node.image = img
mat.node_tree.nodes.active = node
for o in objs: o.data.materials.clear(); o.data.materials.append(mat)
scene = bpy.context.scene
scene.render.engine = 'CYCLES'
try:
    prefs = bpy.context.preferences.addons['cycles'].preferences
    prefs.compute_device_type = 'METAL'; prefs.get_devices()
    for d in prefs.devices: d.use = True
    scene.cycles.device = 'GPU'
except Exception as e:
    print('GPU unavailable, CPU bake:', e)
scene.cycles.samples = samples
scene.world = bpy.data.worlds.new('w'); scene.world.light_settings.distance = 0.25
scene.render.bake.margin = 8
t0 = time.time()
bpy.ops.object.bake(type='AO')
print('bake seconds', round(time.time() - t0, 1))
img.filepath_raw = out_img; img.file_format = 'PNG'; img.save()
# per-face UVs for the X-Plane file: call -> {orig face: [[u,v] x 3 in Blender loop order]}
res = {}
for o in objs:
    me = o.data; orig = [0] * len(me.polygons); me.attributes['orig'].data.foreach_get('value', orig)
    uv = me.uv_layers.active.data
    res[o.name] = {str(orig[p.index]): [[uv[l].uv[0], uv[l].uv[1]] for l in p.loop_indices] for p in me.polygons}
json.dump(res, open(out_uv, 'w'))
print('faces exported', sum(len(v) for v in res.values()))
