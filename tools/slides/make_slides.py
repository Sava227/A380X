"""A380X escape slides: one exterior object with the 16 passenger-door slides (main deck 0-9, upper deck 10-15).

Each slide is driven by a380x/xp12/doors/slide_ratio[k] (0 stowed .. 1 deployed; A380XDoors moves it when a door
whose slide was armed opens). Object frame = the fuselage objects' frame (attached at psi 180): +x is the side of
the even doors, +y up, +z forward.

Shape (per door): a flat girt platform at the sill, then a ramp of hinged segments down to the ground and a
run-out on the ground. The ramp has two side tubes, a centre lane divider, a head tube across its top and a toe
tube across its end. The path follows the estimated ground and drapes over the wing / belly fairing height map
(wing_heightmap.py); for each door the yaw (0 = straight out, positive = aft) with the shortest path is used, so the
over-wing door 3 slides run aft along the wing and off its trailing edge.

Deployment (slide_ratio 0 -> 1, about 6 s): stowed, the ramp is folded accordion-style below the platform; the
hinges open one after the other from the top, and each segment's tubes inflate while it unfolds (every tube is
made of strips that move out from its centre line to the full radius).

Usage: make_slides.py <aircraft dir> <wing_heightmap.json> <out dir>
Writes <out>/a380x_slides.obj, <out>/a380x_slides.png, <out>/slides.json.
"""
import json, math, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xp2wave3 as xw

REF = 'a380x/xp12/doors/slide_ratio'
DOOR_REF = 'sim/flightmodel2/misc/door_open_ratio['
MAIN = dict(W=1.85, R=0.27, Lp=0.9, slope=34.0, run=1.4, seg=1.35, drop=0.08)
UPPER = dict(W=2.05, R=0.32, Lp=1.0, slope=31.0, run=1.6, seg=1.5, drop=0.08)
DIVIDER_R = 0.11
SIDES = 12            # tube cross-section
STRIPS = 4            # inflation strips per tube (SIDES / STRIPS sides each)
DEFLATED = 0.25       # stowed tube radius fraction
MAX_SEGS = 10
YAWS = range(0, 45, 5)
# Main deck door 3 is over the wing: fuselage-mounted over-wing slide that runs aft alongside the fuselage over the
# wing root and drops off behind the trailing edge, steeper than the door slides.
OVERWING = (4, 5)
OVERWING_YAWS = range(60, 91, 5)
OVERWING_SPEC = dict(slope=42.0, run=1.2, Lp=0.4)


def ground_y(z):
    """Ground under the aircraft at rest: through the tyre contact points, raised for strut compression."""
    return -4.625 + 0.0166 * (z + 1.0)


# ---------------------------------------------------------------- door openings (closed outer panels)
def door_openings(acf):
    out = {}
    for f in ('a380x_fuse1', 'a380x_fuse2', 'a380x_fuse3'):
        path = os.path.join(acf, 'objects/exterior/fuselage', f + '.obj')
        V, I, calls = [], [], []
        stack, refs, pend = [np.eye(4)], [set()], None
        for line in open(path, errors='replace'):
            p = line.split()
            if not p:
                continue
            k = p[0]
            if k == 'VT': V.append([float(x) for x in p[1:4]])
            elif k in ('IDX10', 'IDX'): I.extend(int(x) for x in p[1:])
            elif k == 'ANIM_begin': stack.append(stack[-1].copy()); refs.append(set(refs[-1]))
            elif k == 'ANIM_end': stack.pop(); refs.pop()
            elif k in ('ANIM_trans_begin', 'ANIM_rotate_begin'):
                pend = ([float(x) for x in p[1:4]] if 'rotate' in k else None, [], p[-1]); refs[-1].add(p[-1])
            elif k == 'ANIM_trans_key': pend[1].append((float(p[1]), [float(x) for x in p[2:5]]))
            elif k == 'ANIM_rotate_key': pend[1].append((float(p[1]), [float(p[2])]))
            elif k == 'ANIM_trans_end': stack[-1] = stack[-1] @ xw.tr(xw.interp(pend[1], 0.0)); pend = None
            elif k == 'ANIM_rotate_end':
                stack[-1] = stack[-1] @ xw.rot(pend[0], float(xw.interp(pend[1], 0.0)[0])); pend = None
            elif k == 'ANIM_trans': stack[-1] = stack[-1] @ xw.tr(np.array(p[1:4], float))
            elif k == 'ANIM_rotate': stack[-1] = stack[-1] @ xw.rot([float(x) for x in p[1:4]], float(p[4]))
            elif k == 'TRIS': calls.append((int(p[1]), int(p[2]), stack[-1].copy(), set(refs[-1])))
        V, I = np.array(V), np.array(I)
        for off, cnt, M, rs in calls:
            ds = {int(r[len(DOOR_REF):-1]) for r in rs if r.startswith(DOOR_REF)}
            if len(ds) != 1:
                continue
            d = ds.pop()
            if d > 15:
                continue
            P = (np.c_[V[I[off:off + cnt]], np.ones(cnt)] @ M.T)[:, :3]
            lo, hi = P.min(0), P.max(0)
            # the outer door panel: the call that spans the full door height
            if hi[1] - lo[1] > 1.8 and (d not in out or cnt > out[d]['n']):
                out[d] = dict(n=cnt, lo=lo.tolist(), hi=hi.tolist())
    doors = {}
    for d, o in sorted(out.items()):
        lo, hi = np.array(o['lo']), np.array(o['hi'])
        s = 1.0 if d % 2 == 0 else -1.0
        doors[d] = dict(side=s, sill_y=float(lo[1]), x=float(max(abs(lo[0]), abs(hi[0]))),
                        z0=float(lo[2]), z1=float(hi[2]), upper=d >= 10)
    assert len(doors) == 16, sorted(doors)
    return doors


# ---------------------------------------------------------------- obstacles
class HeightMap:
    """Obstacles beside the fuselage: per 0.25 m cell the top and bottom surface heights (None = nothing)."""
    def __init__(self, path):
        r = json.load(open(path))
        self.x, self.z = np.array(r['x']), np.array(r['z'])
        self.top = np.array([[np.nan if v is None else v for v in row] for row in r['top']])
        self.bot = np.array([[np.nan if v is None else v for v in row] for row in r['bottom']])

    def __call__(self, x, z):
        if x < self.x[0] or x > self.x[-1] or z < self.z[0] or z > self.z[-1]:
            return None
        i = min(int(round((z - self.z[0]) / 0.25)), len(self.z) - 1); j = int(np.argmin(abs(self.x - x)))
        t, b = self.top[i, j], self.bot[i, j]
        if np.isnan(t):
            return None
        return t, (b if not np.isnan(b) else -1e9)


def plan_path(door, yaw, hm, S):
    """Bed centre-line height along the slide (horizontal distance s). Returns (s, y) points, total length.
    The bed descends at the slide slope; where it would run into an obstacle from above it rests on top of it
    (wing, fairing); it may pass under an obstacle with CLEAR to spare."""
    s_sign, zc = door['side'], (door['z0'] + door['z1']) / 2
    d = np.array([s_sign * math.cos(math.radians(yaw)), 0.0, -math.sin(math.radians(yaw))])
    n = np.array([-d[2], 0.0, d[0]])
    start = np.array([s_sign * door['x'], door['sill_y'] - S['drop'], zc])
    # a yawed slide starts outboard of the door by its half width, so it runs alongside the fuselage
    start[0] += s_sign * (S['W'] / 2 + 0.1) * math.sin(math.radians(yaw))
    lift = 0.45 * S['R']
    clear = 2 * S['R'] + 0.3
    ds, tan = 0.1, math.tan(math.radians(S['slope']))
    pts = [(0.0, start[1])]
    y, s, on_ground, run = start[1], 0.0, 0.0, False
    hits = 0
    while s < 30:
        s += ds
        p = start + d * s
        floor = ground_y(p[2]) + lift
        above = y
        y = y if s <= S['Lp'] else y - tan * ds
        for w in (-S['W'] / 2, 0, S['W'] / 2):
            o = hm(*(p + n * w)[[0, 2]])
            if o is not None and y < o[0] + lift + 0.06 and y > o[1] - clear:
                if above >= o[0] - 0.3:
                    y = max(y, o[0] + lift + 0.06)      # rest on top of it
                else:
                    hits += 1                            # would cut through it from below
        y = max(y, floor)
        pts.append((s, y))
        if y <= floor + 1e-6 and s > S['Lp']:
            on_ground += ds
            if on_ground >= S['run']:
                break
        else:
            on_ground = 0.0
    P = np.array(pts)
    length = float(np.sum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))))
    return P, length, start, d, n, hits


def footprint(P, start, d, n, W):
    """Plan-view sample points (x, z) of the slide's centre line and edges, every 0.25 m."""
    out = []
    for s in np.arange(0.0, P[-1, 0], 0.25):
        c = start + d * s
        for w in (-W / 2, 0.0, W / 2):
            q = c + n * w
            out.append((q[0], q[2]))
    return np.array(out)


def choose_plan(k, door, hm, planned):
    """Shortest path over the allowed yaws that cuts through nothing and keeps clear of the slides already planned."""
    S = dict(UPPER if door['upper'] else MAIN)
    if k in OVERWING:
        S.update(OVERWING_SPEC)
    best = None
    for yaw in (OVERWING_YAWS if k in OVERWING else YAWS):
        P, L, start, d, n, hits = plan_path(door, yaw, hm, S)
        fp = footprint(P, start, d, n, S['W'])
        overlap = 0
        for other in planned.values():
            dist = np.min(np.hypot(fp[:, None, 0] - other[None, :, 0], fp[:, None, 1] - other[None, :, 1]), axis=1)
            overlap += int(np.sum(dist < 0.6))
        score = L + 0.02 * yaw + 5.0 * hits + 5.0 * overlap
        if best is None or score < best[0] - 1e-9:
            best = (score, yaw, P, L, start, d, n, hits, overlap, S)
    planned[k] = footprint(*best[2:3], *best[4:7], best[9]['W'])
    return best


# ---------------------------------------------------------------- geometry
class Mesh:
    def __init__(self):
        self.vt, self.idx = [], []

    def tris(self, verts, faces):
        """verts: [(pos, normal, uv)], faces: index triples into verts. Returns (start, count) in idx."""
        base = len(self.vt)
        for p, nn, uv in verts:
            self.vt.append('VT\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f' % (*p, *nn, *uv))
        start = len(self.idx)
        for a, b, c in faces:
            self.idx += [base + a, base + b, base + c]
        return start, len(self.idx) - start


UV = dict(tube=(0.125, 0.5), bed=(0.375, 0.5), lane=(0.375, 0.9), girt=(0.625, 0.5), dark=(0.875, 0.5),
          head=(0.125, 0.85))


def unit(v):
    return v / np.linalg.norm(v)


def tube_strips(mesh, a, b, axis_u, axis_v, R, uv, ext=0.0):
    """Tube from a to b, cross-section in the (axis_u, axis_v) plane. Returns [(start, count, mid_dir)] per strip."""
    t = unit(b - a)
    a, b = a - t * ext, b + t * ext
    per = SIDES // STRIPS
    out = []
    for k in range(STRIPS):
        verts, faces = [], []
        for j in range(per + 1):
            ang = 2 * math.pi * (k * per + j) / SIDES
            r = math.cos(ang) * axis_u + math.sin(ang) * axis_v
            verts += [(a + r * R, r, uv), (b + r * R, r, uv)]
        for j in range(per):
            i0 = 2 * j
            faces += [(i0, i0 + 2, i0 + 1), (i0 + 1, i0 + 2, i0 + 3)]
        # outward facing: (b-a) x (dr/dang) direction check, flip if needed
        p0, p1, p2 = verts[0][0], verts[2][0], verts[1][0]
        nrm = np.cross(p1 - p0, p2 - p0)
        if np.dot(nrm, verts[0][1]) < 0:
            faces = [(x, z, y) for x, y, z in faces]
        mid = 2 * math.pi * (k * per + per / 2) / SIDES
        s, c = mesh.tris(verts, faces)
        out.append((s, c, math.cos(mid) * axis_u + math.sin(mid) * axis_v))
    return out


def slab(mesh, a, b, n, up, half_w, thick, uv):
    """Flat bed from a to b (centre line), width 2*half_w across n, top face up, bottom face down."""
    verts, faces = [], []
    for sgn, nn in ((1, up), (-1, -up)):
        o = up * (thick / 2 if sgn > 0 else -thick / 2)
        base = len(verts)
        verts += [(a + o - n * half_w, nn, uv), (a + o + n * half_w, nn, uv),
                  (b + o + n * half_w, nn, uv), (b + o - n * half_w, nn, uv)]
        f = [(base, base + 1, base + 2), (base, base + 2, base + 3)]
        q0, q1, q2 = verts[base][0], verts[base + 1][0], verts[base + 2][0]
        if np.dot(np.cross(q1 - q0, q2 - q0), nn) < 0:
            f = [(x, z, y) for x, y, z in f]
        faces += f
    return mesh.tris(verts, faces)


def fmt(v):
    return '\t'.join('%.4f' % x for x in v)


def build_slide(mesh, k, door, plan):
    _, yaw, P, L, start, d, n, hits, overlap, S = plan
    up_world = np.array([0.0, 1.0, 0.0])

    def world(s, y):
        q = start + d * s
        return np.array([q[0], y, q[2]])

    # hinge points: platform end, then equal arc-length steps along the path
    seg_s = [0.0, S['Lp']]
    arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))])
    rest = arc[-1] - np.interp(S['Lp'], P[:, 0], arc)
    nseg = int(min(MAX_SEGS, max(3, math.ceil(rest / S['seg']))))
    a_lp = np.interp(S['Lp'], P[:, 0], arc)
    for i in range(1, nseg + 1):
        seg_s.append(float(np.interp(a_lp + rest * i / nseg, arc, P[:, 0])))
    pts = [world(s, float(np.interp(s, P[:, 0], P[:, 1]))) for s in seg_s]

    lines = []
    W, R = S['W'], S['R']
    door_w = door['z1'] - door['z0']
    # platform (static): girt slab across the door width, from the sill to the platform end
    s0, c0 = slab(mesh, pts[0], pts[1], n, up_world, door_w / 2 - 0.05, 0.06, UV['girt'])
    lines += ['ANIM_begin', f'\tANIM_hide\t0\t0.004\t{REF}[{k}]', f'\tTRIS\t{s0}\t{c0}']
    sill = np.array([door['side'] * door['x'], door['sill_y'] - S['drop'], (door['z0'] + door['z1']) / 2])
    if abs(pts[0][0] - sill[0]) > 0.05:      # girt bridge from the door sill out to a yawed slide
        out = np.array([door['side'], 0.0, 0.0])
        nb = np.array([0.0, 0.0, 1.0])
        sb0, cb0 = slab(mesh, sill, pts[0] + out * 0.3, nb, up_world, door_w / 2 - 0.05, 0.06, UV['girt'])
        lines.append(f'\tTRIS\t{sb0}\t{cb0}')

    # deployed in-plane angle of every segment (degrees, + toward up)
    def ang(a, b):
        v = b - a
        return math.degrees(math.atan2(v[1], float(np.dot(v, d))))
    theta = [ang(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]     # theta[0] = platform
    fold = [theta[0]]
    for i in range(1, len(theta)):
        fold.append((-82.0 if i % 2 == 1 else 98.0))                   # accordion: down, back up, down ...
    rel = [0.0] + [(fold[i] - theta[i]) - (fold[i - 1] - theta[i - 1]) for i in range(1, len(theta))]
    nr = len(theta) - 1
    depth = 1
    for i in range(1, len(theta)):
        a0 = 0.05 + 0.75 * (i - 1) / nr
        a1 = min(1.0, a0 + 0.22)
        f0, f1 = max(0.0, a0 - 0.03), min(1.0, a1 + 0.05)
        ind = '\t' * depth
        A, Bp = pts[i], pts[i + 1]
        lines += [ind + 'ANIM_begin',
                  ind + '\tANIM_trans\t%s\t%s\t0\t0\tnone' % (fmt(A), fmt(A)),
                  ind + f'\tANIM_rotate_begin\t{fmt(n)}\t{REF}[{k}]',
                  ind + '\t\tANIM_rotate_key\t0\t%.3f' % rel[i],
                  ind + '\t\tANIM_rotate_key\t%.4f\t%.3f' % (a0, rel[i]),
                  ind + '\t\tANIM_rotate_key\t%.4f\t0' % a1,
                  ind + '\t\tANIM_rotate_key\t1\t0',
                  ind + '\tANIM_rotate_end',
                  ind + '\tANIM_trans\t%s\t%s\t0\t0\tnone' % (fmt(-A), fmt(-A))]
        t = unit(Bp - A)
        bn = unit(np.cross(t, n))
        if bn[1] < 0:
            bn = -bn
        # bed + lanes
        sb, cb = slab(mesh, A, Bp, n, bn, W / 2 - R * 0.7, 0.05, UV['bed'])
        lines.append(ind + f'\tTRIS\t{sb}\t{cb}')
        strips = []
        for side in (-1, 1):
            c = n * side * (W / 2 - R)
            strips += [(st, ct, md, R) for st, ct, md in
                       tube_strips(mesh, A + c + bn * 0.55 * R, Bp + c + bn * 0.55 * R, n, bn, R, UV['tube'], ext=0.12)]
        strips += [(st, ct, md, DIVIDER_R) for st, ct, md in
                   tube_strips(mesh, A + bn * 0.3 * R, Bp + bn * 0.3 * R, n, bn, DIVIDER_R, UV['lane'], ext=0.05)]
        if i == 1:      # head tube across the top of the ramp
            strips += [(st, ct, md, R * 1.1) for st, ct, md in
                       tube_strips(mesh, A - n * (W / 2) + bn * 0.7 * R, A + n * (W / 2) + bn * 0.7 * R, t, bn,
                                   R * 1.1, UV['head'], ext=0.0)]
        if i == nr:     # toe tube across the end
            strips += [(st, ct, md, R) for st, ct, md in
                       tube_strips(mesh, Bp - n * (W / 2) + bn * 0.55 * R, Bp + n * (W / 2) + bn * 0.55 * R, t, bn,
                                   R, UV['tube'], ext=0.0)]
        for st, ct, md, rr in strips:
            off = -md * rr * (1 - DEFLATED)
            lines += [ind + '\tANIM_begin',
                      ind + f'\t\tANIM_trans_begin\t{REF}[{k}]',
                      ind + '\t\t\tANIM_trans_key\t0\t%s' % fmt(off),
                      ind + '\t\t\tANIM_trans_key\t%.4f\t%s' % (f0, fmt(off)),
                      ind + '\t\t\tANIM_trans_key\t%.4f\t0\t0\t0' % f1,
                      ind + '\t\t\tANIM_trans_key\t1\t0\t0\t0',
                      ind + '\t\tANIM_trans_end',
                      ind + f'\t\tTRIS\t{st}\t{ct}',
                      ind + '\tANIM_end']
        depth += 1
    for i in range(depth - 1, 0, -1):
        lines.append('\t' * i + 'ANIM_end')
    lines.append('ANIM_end')
    info = dict(door=k, yaw=yaw, length=round(L, 2), segments=nr, cuts=hits, overlap=overlap,
                sill=[round(float(v), 3) for v in pts[0]],
                toe=[round(float(v), 3) for v in pts[-1]], height=round(float(pts[0][1] - pts[-1][1]), 2))
    return lines, info


def texture(path):
    from PIL import Image, ImageDraw
    im = Image.new('RGB', (128, 128))
    dr = ImageDraw.Draw(im)
    dr.rectangle([0, 0, 31, 127], fill=(240, 182, 18))      # tubes: slide yellow
    dr.rectangle([0, 0, 31, 31], fill=(232, 168, 14))       # head tube, slightly deeper
    dr.rectangle([32, 0, 63, 127], fill=(214, 160, 30))     # bed
    dr.rectangle([32, 0, 63, 23], fill=(70, 70, 72))        # lane divider: dark grey
    dr.rectangle([64, 0, 95, 127], fill=(118, 120, 122))    # girt / platform: grey
    dr.rectangle([96, 0, 127, 127], fill=(28, 28, 30))
    im.save(path)


def main(acf, hm_path, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    doors = door_openings(acf)
    hm = HeightMap(hm_path)
    mesh, body, infos, planned, plans = Mesh(), [], [], {}, {}
    for k in sorted(range(16), key=lambda k: k in OVERWING):     # the over-wing slides fit around the others
        plans[k] = choose_plan(k, doors[k], hm, planned)
    for k in range(16):
        lines, info = build_slide(mesh, k, doors[k], plans[k])
        body += lines
        infos.append(info)
    I = mesh.idx
    head = ['I', '800', 'OBJ', '', 'TEXTURE\ta380x_slides.png', '',
            f'POINT_COUNTS\t{len(mesh.vt)}\t0\t0\t{len(I)}', '']
    idx = ['IDX10\t' + '\t'.join(str(x) for x in I[s:s + 10]) for s in range(0, len(I) - len(I) % 10, 10)]
    idx += [f'IDX\t{x}' for x in I[len(I) - len(I) % 10:]]
    text = '\n'.join(head + mesh.vt + [''] + idx + ['', '# A380X escape slides (make_slides.py); ' + REF,
                                                     'ATTR_cull'] + body) + '\n'
    open(os.path.join(out_dir, 'a380x_slides.obj'), 'w').write(text)
    texture(os.path.join(out_dir, 'a380x_slides.png'))
    json.dump(dict(doors={str(k): v for k, v in doors.items()}, slides=infos),
              open(os.path.join(out_dir, 'slides.json'), 'w'), indent=1)
    for i in infos:
        print(i)
    print(f'vertices {len(mesh.vt)}, indices {len(I)}')


if __name__ == '__main__':
    main(*sys.argv[1:4])
