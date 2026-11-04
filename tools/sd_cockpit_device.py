"""Append ECAM screen/ECP controls without rebuilding existing cockpit objects."""
import re
from pathlib import Path
from avionics_panel_layout import SD_SCREEN_SURFACE
from build_cockpit_controls import Control, build_obj, mfd_surface_normal
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

SD_DEVICE = 'a380x.xp12.ecam.sd'
SD_CHANNEL = 18
MARKER = '# A380X NATIVE ECAM SD V2'
PAGE_NODES = ('ENG','APU','BLEED','COND','PRESS','DOOR','ELAC','ELDC','FUEL','WHEEL','HYD','FCTL','CB','STS','VIDEO')
PAGE_ACTIONS = ('engine','apu','bleed','cond','press','door','elec_ac','elec_dc','fuel','wheel','hyd','fctl','cb','status','video')
ACTION_NODES = {'ALL':'all','TOCONF':'to_config','CLR_LH':'clear_left','CLR_RH':'clear_right','RCLLAST':'recall_last','RCL':'recall','CL':'checklist',
                'CHECK_LH':'check_left','CHECK_RH':'check_right','UP':'up','DOWN':'down','ABNPROC':'abnormal_procedures','EMERCANC':'emergency_cancel','MORE':'more'}


def controls_patch(source: Path):
    controls = [*(Control('PUSH_ECAM_'+node,'a380x/xp12/ecam/sd/'+action,'ECAM - '+node,command_cursor='button') for node,action in zip(PAGE_NODES,PAGE_ACTIONS)),
        *(Control('PUSH_ECAM_'+node,'a380x/xp12/ecam/sd/'+action,'ECAM - '+node,command_cursor='button') for node,action in ACTION_NODES.items()),
        Control('KNOB_EFIS_SD', None, 'ECAM SD - brightness and OFF',
                command_up='a380x/xp12/ecam/sd/brightness_up',
                command_down='a380x/xp12/ecam/sd/brightness_down'),
        Control('KNOB_EFIS_EWD', None, 'ECAM EWD - brightness and OFF',
                command_up='a380x/xp12/ecam/sd/ewd_brightness_up',command_down='a380x/xp12/ecam/sd/ewd_brightness_down'),
    ]
    return compact_targets(build_obj(source, controls, fcu_overlay=False)[0])


def compact_targets(text):
    """12 triangles per invisible control instead of its detailed button mesh.

    Preserve authored world-space bounds and every manipulator command. Only
    this new ECAM patch is compacted; existing aircraft geometry is untouched.
    """
    lines=text.splitlines()
    points=[tuple(map(float,line.split()[1:4])) for line in lines if line.startswith('VT\t')]
    old_indices=[int(n) for line in lines if re.match(r'^IDX(?:10)?\s',line) for n in line.split()[1:]]
    start=lines.index('ATTR_draw_disable')
    vertices=[];indices=[];commands=[]
    faces=(((0,3,2,1),(0,0,-1)),((4,5,6,7),(0,0,1)),((0,1,5,4),(0,-1,0)),
           ((3,7,6,2),(0,1,0)),((0,4,7,3),(-1,0,0)),((1,2,6,5),(1,0,0)))
    for line in lines[start:]:
        if not line.startswith('TRIS\t'):
            commands.append(line);continue
        _,at,count=line.split();used=[points[k] for k in old_indices[int(at):int(at)+int(count)]]
        lo=[min(p[k] for p in used) for k in range(3)];hi=[max(p[k] for p in used) for k in range(3)]
        x,y,z=lo;X,Y,Z=hi
        box=((x,y,z),(X,y,z),(X,Y,z),(x,Y,z),(x,y,Z),(X,y,Z),(X,Y,Z),(x,Y,Z))
        begin=len(indices)
        for face,normal in faces:
            base=len(vertices)
            vertices.extend((*box[k],*normal,0,0) for k in face)
            indices.extend(base+k for k in (0,2,1,0,3,2)) # OBJ8 clockwise fronts
        commands.append(f'TRIS\t{begin}\t36')
    if not vertices:raise ValueError('ECAM patch contains no physical controls')
    return '\n'.join(['I','800','OBJ',f'POINT_COUNTS\t{len(vertices)}\t0\t0\t{len(indices)}',
        *('VT\t'+'\t'.join(f'{obj8_fmt(v)}' for v in vertex) for vertex in vertices),
        *('IDX\t'+str(i) for i in indices),*commands,''])


def append_geometry(text, vertices, indices, commands):
    lines = text.splitlines()
    row = next(i for i,line in enumerate(lines) if line.startswith('POINT_COUNTS'))
    count = list(map(int,lines[row].split()[1:]))
    if len(count)!=4:
        raise ValueError('Invalid OBJ point counts')
    previous_vertices = sum(line.startswith('VT\t') for line in lines)
    previous_indices = sum(len(line.split())-1 for line in lines if re.match(r'^IDX(?:10)?\s',line))
    if previous_vertices!=count[0] or previous_indices!=count[3]:
        raise ValueError('Existing cockpit geometry is inconsistent; refusing to rewrite it')
    at=max(i for i,line in enumerate(lines) if line.startswith('VT\t'))+1
    lines[at:at]=vertices
    at=max(i for i,line in enumerate(lines) if re.match(r'^IDX(?:10)?\s',line))+1
    lines[at:at]=['IDX\t'+str(count[0]+n) for n in indices]
    lines[row]='POINT_COUNTS\t'+'\t'.join(map(str,(count[0]+len(vertices),count[1],count[2],count[3]+len(indices))))
    adjusted=[]
    for line in commands:
        words=line.split()
        if words and words[0]=='TRIS':
            line=f'TRIS\t{int(words[1])+count[3]}\t{words[2]}'
        adjusted.append(line)
    return '\n'.join([*lines,'',MARKER,*adjusted,''])


def augment_controls(text, patch):
    if MARKER in text:
        return text
    if '# A380X NATIVE ECAM SD V1' in text:
        raise ValueError('Old ECAM V1 geometry must be explicitly upgraded, not duplicated')
    lines=patch.splitlines()
    vertices=[line for line in lines if line.startswith('VT\t')]
    indices=[int(n) for line in lines if re.match(r'^IDX(?:10)?\s',line) for n in line.split()[1:]]
    start=next(i for i,line in enumerate(lines) if line=='ATTR_draw_disable')
    if not vertices or not indices:
        raise ValueError('Empty ECAM control patch')
    return append_geometry(text,vertices,indices,
        ['ATTR_no_cockpit','ATTR_manip_none','ATTR_poly_os\t0',*lines[start:]])


def augment_screen(text):
    if MARKER in text:
        return text
    x0,x1,y0,z0,y1,z1=SD_SCREEN_SURFACE
    corners=((x1,y0,z0),(x1,y1,z1),(x0,y1,z1),(x0,y0,z0))
    normal=mfd_surface_normal(corners)
    vertices=[]
    for p,uv in zip(corners,((0,0),(0,1),(1,1),(1,0))):
        v=(*(p[i]+normal[i]*.002 for i in range(3)),*normal,*uv)
        vertices.append('VT\t'+'\t'.join(f'{obj8_fmt(n)}' for n in v))
    return append_geometry(text,vertices,[0,1,2,0,2,3],
        ['ATTR_no_cockpit','ATTR_draw_enable','ATTR_poly_os\t2',
         f'ATTR_cockpit_device\t{SD_DEVICE}\t1\t{SD_CHANNEL}\t1',
         'TRIS\t0\t6','ATTR_no_cockpit','ATTR_poly_os\t0'])
