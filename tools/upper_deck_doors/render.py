import bpy, sys, os, math
argv = sys.argv[sys.argv.index('--') + 1:]
out, cam_loc, cam_rot = argv[0], [float(x) for x in argv[1].split(',')], [float(x) for x in argv[2].split(',')]
items = [a.split('=', 1) for a in argv[3:]]     # objfile=albedo|none, '@nocull' suffix = double-sided
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
for objfile, tex in items:
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=objfile, forward_axis='Y', up_axis='Z')
    new = [o for o in bpy.data.objects if o not in before]
    mat = bpy.data.materials.new(os.path.basename(objfile))
    mat.use_nodes = True
    nocull = tex.endswith('@nocull'); tex = tex[:-7] if nocull else tex
    mat.use_backface_culling = os.environ.get('CULL') == '1' and not nocull
    bsdf = mat.node_tree.nodes['Principled BSDF']
    if tex not in ('none', 'magenta', 'red', 'green', 'blue', 'yellow', 'cyan', 'orange'):
        img = mat.node_tree.nodes.new('ShaderNodeTexImage')
        img.image = bpy.data.images.load(tex)
        mat.node_tree.links.new(img.outputs['Color'], bsdf.inputs['Base Color'])
        mat.diffuse_color = (1, 1, 1, 1)
    else:
        col = {'magenta': (1, 0, 1, 1), 'red': (0.9, 0.1, 0.1, 1), 'green': (0.1, 0.8, 0.1, 1), 'blue': (0.15, 0.3, 0.95, 1), 'yellow': (0.95, 0.85, 0.1, 1), 'cyan': (0.1, 0.85, 0.9, 1), 'orange': (1, 0.5, 0.05, 1)}.get(tex, (0.55, 0.55, 0.55, 1))
        bsdf.inputs['Base Color'].default_value = col
        mat.diffuse_color = col
    for o in new:
        o.data.materials.clear(); o.data.materials.append(mat)
cam = bpy.data.cameras.new('cam'); cam.lens = 18
co = bpy.data.objects.new('cam', cam); scene.collection.objects.link(co)
co.location = cam_loc; co.rotation_euler = [math.radians(a) for a in cam_rot]
scene.camera = co
scene.render.engine = 'BLENDER_WORKBENCH'
scene.world = bpy.data.worlds.new('sky'); scene.world.color = (0.45, 0.65, 0.95)
sh = scene.display.shading
sh.light = 'STUDIO'; sh.color_type = 'TEXTURE'
for o in bpy.data.objects:
    if o.type == 'MESH' and o.active_material and o.active_material.diffuse_color[1] == 0.0: pass; sh.show_backface_culling = os.environ.get('CULL') == '1'
scene.render.resolution_x, scene.render.resolution_y = 1280, 800
scene.render.filepath = out
bpy.ops.render.render(write_still=True)
print('rendered', out)
