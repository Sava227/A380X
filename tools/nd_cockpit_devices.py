"""Add native ND device quads without rebuilding or shifting existing cockpit geometry."""
import re
from avionics_panel_layout import ND_SCREEN_SURFACES
from build_cockpit_controls import mfd_surface_normal
from obj8_numbers import fmt as obj8_fmt  # X-Plane cannot read exponent notation

ND_DEVICES = ('a380x.xp12.nd.captain','a380x.xp12.nd.first_officer')
ND_CHANNELS = (16,17)
ND_COMMANDS = tuple(f'a380x/xp12/nd/{side}/{action}' for side in ('captain','first_officer')
                    for action in ('popup','map_data','center','zoom_in','zoom_out'))
ND_DATAREFS = ('a380x/xp12/xpkit/nd_devices_active',*(f'a380x/xp12/nd/{side}/cursor_{axis}'
                for side in ('captain','first_officer') for axis in ('x','y','visible')))
MARKER = '# A380X NATIVE ND DEVICE QUADS V1'


def augment(text, touch):
    if MARKER in text:
        return text
    lines = text.splitlines()
    row = next(i for i,line in enumerate(lines) if line.startswith('POINT_COUNTS'))
    counts = list(map(int,lines[row].split()[1:]))
    vertices, indices, draws = [], [], []
    for side,(x0,x1,y0,z0,y1,z1) in enumerate(ND_SCREEN_SURFACES):
        corners = ((x1,y0,z0),(x1,y1,z1),(x0,y1,z1),(x0,y0,z0))
        normal = mfd_surface_normal(corners)
        base = counts[0]+len(vertices)
        draws.append(counts[3]+len(indices))
        for point,uv in zip(corners,((0,0),(0,1),(1,1),(1,0))):
            vertex = (*(point[i]+normal[i]*(.006 if touch else .005) for i in range(3)),*normal,*uv)
            vertices.append('VT\t'+'\t'.join(f'{obj8_fmt(v)}' for v in vertex))
        indices.extend((base,base+1,base+2,base,base+2,base+3))
    lines[row]='POINT_COUNTS\t'+ '\t'.join(map(str,(counts[0]+8,counts[1],counts[2],counts[3]+12)))
    at=max(i for i,line in enumerate(lines) if line.startswith('VT\t'))+1
    lines[at:at]=vertices
    at=max(i for i,line in enumerate(lines) if re.match(r'^IDX(?:10)?\s',line))+1
    lines[at:at]=['IDX\t'+str(n) for n in indices]
    lines.extend(['',MARKER,'ATTR_manip_none','ATTR_no_cockpit','ATTR_poly_os\t2',
                  'ATTR_draw_disable' if touch else 'ATTR_draw_enable'])
    for side,device in enumerate(ND_DEVICES):
        lines.extend(['ANIM_begin','ANIM_hide\t-0.5\t0.5\ta380x/xp12/xpkit/nd_devices_active',
                      f'ATTR_cockpit_device\t{device}\t1\t{ND_CHANNELS[side]}\t1'])
        if touch:
            lines.append(f'ATTR_manip_device\tarrow\t{device}\tA380X ND / OANS map')
        lines.extend([f'TRIS\t{draws[side]}\t6'])
        if touch: lines.append('ATTR_manip_none')
        lines.extend(['ATTR_no_cockpit','ANIM_end'])
    lines.extend(['ATTR_draw_enable','ATTR_poly_os\t0',''])
    # Visible screen object must remain non-manipulated (including ATTR_manip_none).
    if not touch:
        lines=[line for line in lines if line!='ATTR_manip_none']
    return '\n'.join(lines)
