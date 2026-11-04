#!/usr/bin/env python3
"""Stage trim-only changes to existing cockpit OBJs, without rebuilding avionics.

Uses the pinned FBW switch/knob/reset clips and exact mesh selection. Original
visible VT/IDX/UV/texture payload remains identical; click and display faces are
append-only. Never writes to the input aircraft. All edits are staged separately.
"""
import argparse
import hashlib
import json
from pathlib import Path

from build_cockpit_controls import (accessor_values, load_buffer, local_matrix,
                                    matrix_multiply, transform_point)
from isis_cockpit_device import _append, _box_rows
from patch_center_right_animations import source_targets, wrap_geometry
from patch_efis_animations import vertex_payload
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

PREFIX='a380x/xp12/trim/'
MARKER='# A380X TRIM PANEL V1'
DEVICE='a380x.xp12.trim.rudder'
CONTROLS='objects/interior/cockpit-controls/a380x_cockpit_controls.obj'
SCREENS='objects/interior/cockpit-panels/a380x_mfd_screens.obj'
PANEL='objects/interior/cockpit-panels/panel_a380_cockpit_pedestal_knobs02_4k.obj'
MOTIONS=tuple((n,n,'trim/'+s,1) for n,s in (
    ('SWITCH_TRIM_PITCH','pitch_switch'),('KNOB_RUDDERTRIM','rudder_knob'),('PUSH_TRIM_RESET','reset_pressed')))

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def group_motion_meshes(targets):
    """One animation owns the knob, its cap and emissive child mesh together.

    FBW shares vertices across these primitives, so selecting them independently
    makes identical boundary triangles ambiguous. Their union has one owner.
    """
    groups={}
    for target in targets:
        if target['animation']=='SWITCH_TRIM_PITCH':
            # Pinned pedestal.xml explicitly sets INVERT_ANIM=1: switch UP
            # commands nose DOWN; its DOWN position commands nose UP.
            target=dict(target,poses=list(reversed(target['poses'])))
        key=(target['animation'],target['material'])
        if key not in groups:
            groups[key]=dict(target,points=list(target['points']))
        else:
            g=groups[key]
            g['points'].extend(target['points']);g['count']+=target['count']
            g['low']=tuple(min(a,b) for a,b in zip(g['low'],target['low']))
            g['high']=tuple(max(a,b) for a,b in zip(g['high'],target['high']))
    return list(groups.values())

def screen_geometry(source):
    d=json.loads(source.read_text());buffers=[load_buffer(source,b['uri']) for b in d['buffers']]
    nodes=d['nodes'];parents={c:i for i,n in enumerate(nodes) for c in n.get('children',[])}
    def world(i):
        return matrix_multiply(world(parents[i]),local_matrix(nodes[i])) if i in parents else local_matrix(nodes[i])
    rows=[]
    for ni,n in enumerate(nodes):
        for p in d['meshes'][n['mesh']]['primitives'] if 'mesh' in n else ():
            if d['materials'][p['material']]['name']=='RTPI':rows.append((ni,p))
    if len(rows)!=1:raise ValueError('Expected exactly one authored RTPI screen')
    ni,p=rows[0]
    positions=accessor_values(d,buffers,p['attributes']['POSITION'])
    indices=[int(r[0]) for r in accessor_values(d,buffers,p['indices'])]
    packed=p.get('extras',{}).get('ASOBO_primitive',{})
    start=packed.get('StartIndex',0);count=packed.get('PrimitiveCount',len(indices)//3)*3
    indices=[i+packed.get('BaseVertexIndex',0) for i in indices[start:start+count]]
    used=sorted(set(indices));remap={v:i for i,v in enumerate(used)}
    points=[transform_point(world(ni),positions[i]) for i in used]
    lo=[min(p[a] for p in points) for a in range(3)]
    hi=[max(p[a] for p in points) for a in range(3)]
    if count!=18 or len(points)!=8 or hi[1]-lo[1]>.0001:
        raise ValueError('RTPI shape is not the expected eight-corner pedestal face')
    vertices=[]
    for x,y,z in points:
        # Seated crew: +X is left, +Z is forward/up on the pedestal.
        values=(x,y+.0005,z,0,1,0,(hi[0]-x)/(hi[0]-lo[0]),(z-lo[2])/(hi[2]-lo[2]))
        vertices.append('VT\t'+'\t'.join(f'{obj8_fmt(v)}' for v in values))
    # FBW/MSFS stores the opposite front-face winding from X-Plane OBJ8.
    # Reverse each triangle so the display is visible from ABOVE the pedestal.
    exported=[]
    for start in range(0,len(indices),3):
        a,b,c=indices[start:start+3];exported.extend((remap[a],remap[c],remap[b]))
    return vertices,exported,dict(low=lo,high=hi,vertices=8,indices=count)

def control_boxes(targets):
    def bounds(node):
        group=[t for t in targets if t['name']==node or node=='KNOB_RUDDERTRIM' and t['animation']==node]
        if not group:raise ValueError('Missing authored control '+node)
        low=[min(t['low'][a] for t in group)-.001 for a in range(3)]
        high=[max(t['high'][a] for t in group)+.001 for a in range(3)]
        return low,high
    result=[]
    low,high=bounds('SWITCH_TRIM_PITCH');mid=targets[0]['pivot'][2]
    # Ordinary left click/hold on each half; no drag manipulator.
    a,b=low.copy(),high.copy();b[2]=mid-.0002
    result.append((a,b,'pitch_up','PITCH TRIM - hold NOSE UP'))
    a,b=low.copy(),high.copy();a[2]=mid+.0002
    result.append((a,b,'pitch_down','PITCH TRIM - hold NOSE DN'))
    low,high=bounds('KNOB_RUDDERTRIM');mid=next(t['pivot'][0] for t in targets if t['name']=='KNOB_RUDDERTRIM')
    a,b=low.copy(),high.copy();a[0]=mid+.0002
    result.append((a,b,'rudder_left','RUDDER TRIM - hold NOSE L'))
    a,b=low.copy(),high.copy();b[0]=mid-.0002
    result.append((a,b,'rudder_right','RUDDER TRIM - hold NOSE R'))
    low,high=bounds('PUSH_TRIM_RESET');result.append((low,high,'rudder_reset','RUDDER TRIM - RESET to neutral'))
    return result

def augment_controls(text,boxes):
    if MARKER in text:return text
    vertices=[];indices=[]
    commands=['ATTR_manip_none','ATTR_no_cockpit','ATTR_draw_disable']
    for low,high,action,label in boxes:
        vs,ix=_box_rows(low,high);start=len(indices)
        indices.extend(len(vertices)+i for i in ix);vertices.extend(vs)
        commands += [f'ATTR_manip_command\tbutton\t{PREFIX}{action}\t{label}',
                     f'TRIS\t{start}\t{len(ix)}','ATTR_manip_none']
    commands+=['ATTR_draw_enable']
    return _append(text,vertices,indices,commands,MARKER)

def augment_screen(text,vertices,indices):
    if MARKER in text:return text
    commands=['ATTR_no_cockpit','ATTR_draw_enable','ATTR_poly_os\t2','ANIM_begin',
        'ANIM_hide\t-0.5\t0.5\t'+PREFIX+'device_active',
        f'ATTR_cockpit_device\t{DEVICE}\t1\t23\t1',f'TRIS\t0\t{len(indices)}',
        'ATTR_no_cockpit','ANIM_end','ATTR_poly_os\t0']
    return _append(text,vertices,indices,commands,MARKER)

def stage(aircraft,source,destination):
    if destination.exists():raise ValueError('Stage directory must not exist')
    targets=source_targets(source,motions=MOTIONS,include_child_points=True)
    text=(aircraft/PANEL).read_text()
    animated,matched,_=wrap_geometry(text,PANEL,group_motion_meshes(targets),MARKER+' MOTION ')
    if vertex_payload(text)!=vertex_payload(animated):raise ValueError('Visible geometry payload changed')
    vs,ix,screen=screen_geometry(source);boxes=control_boxes(targets)
    staged={PANEL:animated,CONTROLS:augment_controls((aircraft/CONTROLS).read_text(),boxes),
            SCREENS:augment_screen((aircraft/SCREENS).read_text(),vs,ix)}
    destination.mkdir(parents=True)
    files=[]
    for name,changed in staged.items():
        target=destination/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_text(changed)
        files.append(dict(path=name,before_sha256=sha(aircraft/name),after_sha256=sha(target)))
    report=dict(format=1,aircraft=str(aircraft),source=str(source),source_sha256=sha(source),
                files=files,screen=screen,controls=boxes,motion=matched,
                visible_vertex_index_uv_payload_preserved=True,simulator_tested=False)
    (destination/'staging-report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--aircraft',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--stage',type=Path,required=True);a=p.parse_args()
    r=stage(a.aircraft.resolve(),a.source.resolve(),a.stage.resolve())
    print(json.dumps(r,indent=2))

if __name__=='__main__':main()
