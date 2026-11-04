#!/usr/bin/env python3
"""Stage authored cargo holds and native cargo-door animations, offline.

Existing fuselage vertex/index/UV/texture payloads are preserved. Cargo texture
and normalized glTF inputs come from the established source conversion tools.
No source asset or installed aircraft is modified by this builder.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re

from build_cockpit_controls import (accessor_values, load_buffer, local_matrix,
    matrix_multiply, transform_point, transform_normal)
from patch_center_right_animations import source_targets, wrap_geometry, primitive_points, animation_block
from patch_efis_animations import vertex_payload
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

ROOT = Path(__file__).resolve().parents[1]
MARKER = '# CARGO DOOR MOTION '
MATERIAL_ALIASES = {}


def door_specs():
    rows = json.loads((ROOT/'config/cargo_doors.json').read_text())['doors']
    if [r['index'] for r in rows] != [17, 18, 19] or [r['id'] for r in rows] != ['FWD', 'AFT', 'BULK']:
        raise ValueError('Cargo doors must use the three native channels after cockpit door 17')
    return rows


def merged_targets(source, normalized):
    specs = door_specs()
    motions = tuple((r['clip'], r['node'], 'unused', 1) for r in specs if 'clip' in r)
    raw = source_targets(source, motions=motions, include_child_points=True)
    groups = {}
    for target in raw:
        spec = next(r for r in specs if r.get('clip') == target['animation'])
        target['dataref'] = f"sim/flightmodel2/misc/door_open_ratio[{spec['index']}]"
        target['material'] = MATERIAL_ALIASES.get(target['material'], target['material'])
        key = (target['dataref'], target['material'])
        if key not in groups:
            groups[key] = dict(target, name=spec['id']+'_'+target['material'], points=[], count=0, source_nodes=[])
        group = groups[key]
        group['points'].extend(target['points'])
        group['count'] += target['count']
        group['source_nodes'].append(target['name'])
    for group in groups.values():
        group['low'] = tuple(min(p[k] for p in group['points']) for k in range(3))
        group['high'] = tuple(max(p[k] for p in group['points']) for k in range(3))

    document = json.loads(normalized.read_text())
    buffers = [load_buffer(normalized, b['uri']) for b in document['buffers']]
    bulk = next(n for n in document['nodes'] if n.get('name') == 'BULK_CARGO_DOOR')
    points, count = [], 0
    for primitive in document['meshes'][bulk['mesh']]['primitives']:
        ps, n = primitive_points(document, buffers, primitive)
        points += [transform_point(local_matrix(bulk), p) for p in ps]
        count += n
    low = tuple(min(p[k] for p in points) for k in range(3))
    high = tuple(max(p[k] for p in points) for k in range(3))
    top = [p for p in points if p[1] >= high[1]-.003]
    if not top or not (0.5 < high[1]-low[1] < 1.5 and .8 < high[2]-low[2] < 1.5):
        raise ValueError('Unexpected authored bulk-door bounds')
    groups['bulk'] = dict(name='BULK_DOOR', material='a380x_fuse3', points=points, count=count,
        low=low, high=high, kind='rotation', pivot=(min(p[0] for p in top), high[1], (low[2]+high[2])/2),
        axis=(0,0,1), values=[0,1], poses=[0,specs[2]['open_angle_degrees']],
        dataref='sim/flightmodel2/misc/door_open_ratio[19]', source_nodes=['BULK_CARGO_DOOR'])
    return list(groups.values())


def hold_objects(normalized, manifest, targets):
    document = json.loads(normalized.read_text())
    buffers = [load_buffer(normalized, b['uri']) for b in document['buffers']]
    parents = {c:i for i,n in enumerate(document['nodes']) for c in n.get('children',[])}
    cache = {}
    def world(i):
        if i not in cache:
            local = local_matrix(document['nodes'][i])
            cache[i] = matrix_multiply(world(parents[i]),local) if i in parents else local
        return cache[i]
    variants = {r['materials'][0]:r for r in json.loads(manifest.read_text())['exports']}
    batches = defaultdict(lambda: dict(vertices=[], indices=[], nodes=[], draws=[], station_triangles=defaultdict(int)))
    for i,node in enumerate(document['nodes']):
        for primitive in document['meshes'][node['mesh']]['primitives'] if 'mesh' in node else ():
            material = document['materials'][primitive['material']]['name']
            if material == 'A380_EXTERIOR_MISC' and node.get('name') not in ('MAIN FUSELAGE.117','MAIN FUSELAGE.102'):
                continue
            if material not in variants:
                continue
            if 'skin' in node:
                raise ValueError('Cargo hold must be rigid authored geometry')
            mesh = batches[material]
            draw_start = len(mesh['indices'])
            positions = accessor_values(document,buffers,primitive['attributes']['POSITION'])
            normals = accessor_values(document,buffers,primitive['attributes']['NORMAL'])
            uv = accessor_values(document,buffers,primitive['attributes']['TEXCOORD_0'])
            ids = [int(v[0]) for v in accessor_values(document,buffers,primitive['indices'])]
            transformed = {}
            for index in set(ids):
                p = transform_point(world(i),positions[index])
                n = transform_normal(world(i),normals[index])
                transformed[index] = (*p,*n,uv[index][0],1-uv[index][1])
            remap = {}
            for index in sorted(transformed):
                remap[index] = len(mesh['vertices'])
                mesh['vertices'].append(transformed[index])
            for offset in range(0,len(ids),3):
                a,b,c = ids[offset:offset+3]
                pa,pb,pc = (transformed[j] for j in (a,b,c))
                ab = [pb[k]-pa[k] for k in range(3)]
                ac = [pc[k]-pa[k] for k in range(3)]
                cross = (ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0])
                if sum(cross[k]*(pa[3+k]+pb[3+k]+pc[3+k]) for k in range(3)) > 0:
                    b,c = c,b
                mesh['indices'].extend((remap[a],remap[b],remap[c]))
                z = (pa[2]+pb[2]+pc[2])/3
                station = 'FWD' if z > 3 else 'AFT' if z > -15 else 'BULK'
                mesh['station_triangles'][station] += 1
            mesh['nodes'].append(node['name'])
            mesh['draws'].append((node['name'],draw_start,len(mesh['indices'])-draw_start))
    if set(batches) != {'Cargo_FWD','Cargo_FWD_DECAL','A380_EXTERIOR_MISC'}:
        raise ValueError('Both authored cargo material layers are required')
    output = []
    for material,mesh in batches.items():
        variant = variants[material]
        lines = ['I','800','OBJ','', '# Authored FBW cargo compartments: FWD, AFT and BULK']
        for key,directive in (('texture','TEXTURE'),('texture_map_normal','TEXTURE_MAP normal'),
                              ('texture_map_material_gloss','TEXTURE_MAP material_gloss')):
            if key in variant:
                lines.append(directive+' ../../../textures/interior/cargo/'+Path(variant[key]).name)
        lines.append(f"POINT_COUNTS\t{len(mesh['vertices'])}\t0\t0\t{len(mesh['indices'])}")
        lines += ['VT\t'+'\t'.join(f'{obj8_fmt(v)}' for v in row) for row in mesh['vertices']]
        for offset in range(0,len(mesh['indices']),10):
            row = mesh['indices'][offset:offset+10]
            lines += ['IDX10\t'+'\t'.join(map(str,row))] if len(row)==10 else ['IDX\t'+str(v) for v in row]
        lines += ['ATTR_shiny_rat\t1','ATTR_no_cull']
        if material.endswith('DECAL'):
            lines += ['ATTR_blend','ATTR_poly_os\t1']
        if material == 'A380_EXTERIOR_MISC':
            for node,start,count in mesh['draws']:
                target = next(t for t in targets if node in t['source_nodes'])
                lines += animation_block(target,start,count,MARKER)
        else:
            lines += [f"TRIS\t0\t{len(mesh['indices'])}"]
        lines.append('')
        name = ('a380x_cargo_hardware' if material == 'A380_EXTERIOR_MISC' else 'a380x_cargo_holds'+('_decals' if material.endswith('DECAL') else ''))+'.obj'
        report = dict(material=material,vertices=len(mesh['vertices']),triangles=len(mesh['indices'])//3,
            station_triangles=dict(mesh['station_triangles']),source_nodes=mesh['nodes'],
            bounds=[tuple(min(v[k] for v in mesh['vertices']) for k in range(3)),
                    tuple(max(v[k] for v in mesh['vertices']) for k in range(3))])
        output.append((name,'\n'.join(lines),report))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--normalized',type=Path,required=True)
    parser.add_argument('--materials',type=Path,required=True)
    parser.add_argument('--aircraft',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    aircraft,output = args.aircraft.resolve(),args.output.resolve()
    if output.is_relative_to(aircraft):
        parser.error('Stage outside the installed aircraft')
    report = dict(scope='Offline cargo staging; no simulator launched',files=[],holds=[],doors=[])
    def stage(relative,data,checks):
        destination = output/'aircraft'/relative
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(data)
        current = aircraft/relative
        report['files'].append(dict(path=relative,staged=str(destination),
            before_sha256=hashlib.sha256(current.read_bytes()).hexdigest() if current.exists() else None,
            sha256=hashlib.sha256(data).hexdigest(),checks=checks))
    targets = merged_targets(args.source,args.normalized)
    patched_names = set()
    for path in sorted((aircraft/'objects/exterior/fuselage').glob('*.obj')):
        if path.name.startswith('._'):
            continue
        before = path.read_text()
        applicable = [t for t in targets if path.name.endswith(t['material']+'.obj')]
        pending = [t for t in applicable if t['dataref'] not in before]
        after,rows,gear_changes = wrap_geometry(before,path.name,pending,marker=MARKER)
        if gear_changes or vertex_payload(after) != vertex_payload(before):
            raise ValueError('Cargo patch changed existing geometry or gear binding')
        rows += [dict(name=t['name'],dataref=t['dataref'],already_animated=True)
                 for t in applicable if t['dataref'] in before]
        if rows:
            patched_names.update(r['name'] for r in rows)
            stage(str(path.relative_to(aircraft)),after.encode(),dict(unchanged_vertex_index_uv_textures=True,animations=rows))
    expected = {t['name'] for t in targets if t['material'] != 'a380_exterior_misc'}
    if patched_names != expected:
        raise ValueError(f'Cargo source nodes not fully matched: {expected-patched_names}')
    for name,text,checks in hold_objects(args.normalized,args.materials,targets):
        stage('objects/interior/cargo/'+name,text.encode(),checks)
        report['holds'].append(dict(file=name,**checks))
    textures = {Path(r[k]) for r in json.loads(args.materials.read_text())['exports']
                for k in ('texture','texture_map_normal','texture_map_material_gloss') if k in r}
    for texture in sorted(textures):
        stage('textures/interior/cargo/'+texture.name,texture.read_bytes(),dict(authored_material_conversion=True))
    for target in targets:
        report['doors'].append({k:v for k,v in target.items() if k not in ('points',)})
    output.mkdir(parents=True,exist_ok=True)
    (output/'cargo-report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(files=len(report['files']),holds=report['holds'],report=str(output/'cargo-report.json'))))


if __name__ == '__main__':
    main()
