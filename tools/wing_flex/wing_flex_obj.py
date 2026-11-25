#!/usr/bin/env python3
"""Add FBW A380 wing flex to the installed exterior wing objects (OBJ8), without re-exporting them.

FBW bends each wing with a chain of bones (WING_FLEX_3 .. WING_FLEX_15 per side, root to tip). Four clips per side
drive them (inboard: 3-5, inner midboard: 6-9, outer midboard: 10-13, outboard: 14-15); the avionics plugin publishes
the clip angles as a380x/xp12/wing/flex_<left|right>_<inboard|inboard_mid|outboard_mid|outboard> (config/
animation_datarefs.json maps each clip linearly from value_min to value_max). The installed objects hold the wing
frozen at the zero-flex frame (the old static_fraction), which is the frame these datarefs give at 0.

OBJ8 cannot skin, so the wing is bent as rigid segments, one per bone: a segment is everything between the cut plane
through its bone's pivot and the next one (planes perpendicular to the span, containing the bone's bend axis, so the
two sides of a cut move together near the axis). Each bone k gets, about its zero-flex pivot p_k and axis a_k (from
the glTF: the change of its local rotation relative to the zero-flex frame, in its parent's zero-flex world frame), an
ANIM_rotate keyed on its clip's dataref. A segment on bone k nests the rotations of bones 3..k, outermost first:
    T(p3) R3 T(p4 - p3) R4 ... Rk T(-pk)
which is exactly the rig's world transform change for bone k (G_k = G_parent * D_k).

Assignment:
  * engines, nacelle cores, fans and pylons: the bone FBW attaches them to (inboard engine WING_FLEX_6, outboard
    WING_FLEX_11; split at |x| = 20 m);
  * wing skin, rigid fragments and the wing fence: each triangle by the segment containing its centroid; nothing
    within 6 m of the centreline moves (where FBW's skin weights start bending);
  * moving parts (slats, flaps, spoilers, ailerons) are cut like the skin, at their retracted position.
Triangles that cross a cut are split exactly along it (new vertices interpolate position, normal and UV on the cut
edge), so the two sides share their edge on the bend axis and open by only (bend change) x (distance from the axis).
Skin within BAND (10 cm) of a cut is drawn on both sides of it, so the opening under or over the bend axis (up to a few
cm at FBW's cruise flex) is always covered; at zero flex the two copies coincide exactly.
Every draw call is flattened into one ANIM block per segment: the flex chain outermost, then the call's original
animation stack, unchanged. Existing vertices are untouched; new ones are appended; the IDX table is rebuilt.
Coordinates: these objects are in the glTF's own frame (x left, y up, z forward; the ACF attaches them turned 180).

    python3 wing_flex_obj.py --gltf A380_WINGS.gltf --config animation_datarefs.json --out OUT_DIR OBJ...
"""
from __future__ import annotations
import argparse, json, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gltfrig import Rig, rot_axis_angle

CLIPS = {'inboard_flex': 'inboard', 'inner_midboard_flex': 'inboard_mid', 'outer_midboard_flex': 'outboard_mid',
         'outboard_flex': 'outboard'}
ENGINE_OBJECTS = re.compile(r'a380x_(eng_|fans|pylons)')
ENGINE_SPLIT_X = 20.0
X_ROOT = 6.0          # FBW's flexing skin starts ~5.5-6.3 m from the centreline at every chord station
KEYS = 9
BAND = 0.10          # m: skin within this distance of a cut is drawn on both sides, covering the opening


def fmt(x):
    s = f'{x:.7f}'.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


# ---------------------------------------------------------------- rig
class Flex:
    def __init__(self, gltf, config):
        r = Rig(gltf); rules = json.load(open(config))['mappings']
        self.bones = {}          # (side, k) -> dict(pivot, axis, dataref, keys, normal)
        self.report = []
        for side, sname in (('L', 'left'), ('R', 'right')):
            for clip, short in CLIPS.items():
                name = f'{sname}_{clip}'
                rule = next(x for x in rules if re.fullmatch(x['pattern'], name))
                dr, vmin, vmax = rule['dataref'], float(rule['value_min']), float(rule['value_max'])
                if not dr.endswith(f'flex_{sname}_{short}'):
                    raise ValueError(f'{name}: unexpected dataref {dr}')
                dur = r.clip_duration(name)
                t_of = lambda v: dur * (v - vmin) / (vmax - vmin)
                ov0 = r.sample(name, t_of(0.0)); W0 = r.world(ov0)
                vs = sorted(set(np.round(np.linspace(vmin, vmax, KEYS), 6)) | {0.0})
                for node in ov0:
                    m = re.fullmatch(r'WING_FLEX_(\d+)\.([LR])', r.nodes[node]['name'])
                    assert m and m.group(2) == side, r.nodes[node]['name']
                    k = int(m.group(1)); L0 = r.local(node, ov0); P0 = W0[r.parent[node]][:3, :3]
                    rots = []
                    for v in vs:
                        L = r.local(node, r.sample(name, t_of(v)))
                        dt = float(np.abs(L[:3, 3] - L0[:3, 3]).max()); self.max_trans = max(getattr(self, 'max_trans', 0.0), dt)
                        if dt > 1e-4: raise ValueError(f'{name}: bone {k} translates by {dt} m')
                        ax, ang = rot_axis_angle(L[:3, :3] @ np.linalg.inv(L0[:3, :3]))
                        axw = P0 @ ax; axw /= np.linalg.norm(axw); rots.append((v, axw, ang))
                    ref = max(rots, key=lambda x: x[2])[1]           # axis of the largest rotation
                    keys, dev = [], 0.0
                    for v, axw, ang in rots:
                        c = float(np.dot(axw, ref))
                        if ang > 1e-6: dev = max(dev, np.degrees(np.arccos(min(1.0, abs(c)))))
                        keys.append((v, ang * (1 if c >= 0 else -1)))
                    self.bones[(side, k)] = dict(pivot=W0[node][:3, 3].copy(), axis=ref, dataref=dr, keys=keys,
                                                 axis_dev=dev, clip=name)
        for side in 'LR':
            ks = sorted(k for s, k in self.bones if s == side)
            assert ks == list(range(3, 16)), ks
            for k in ks:
                b = self.bones[(side, k)]; n = np.cross(b['axis'], [0.0, 1.0, 0.0]); n /= np.linalg.norm(n)
                outward = (self.bones[(side, k + 1)]['pivot'] - b['pivot'] if k < 15
                           else b['pivot'] - self.bones[(side, k - 1)]['pivot'])
                b['normal'] = n if np.dot(n, outward) > 0 else -n
                self.report.append(dict(side=side, bone=k, clip=b['clip'], dataref=b['dataref'],
                                        pivot=[round(float(x), 4) for x in b['pivot']],
                                        axis=[round(float(x), 5) for x in b['axis']], axis_dev_deg=round(b['axis_dev'], 4),
                                        angle_range_deg=[round(b['keys'][0][1], 4), round(b['keys'][-1][1], 4)]))

    def segment(self, p):
        """bone (side, k) whose segment contains point p, or None (inboard of WING_FLEX_3)"""
        side = 'L' if p[0] > 0 else 'R'; best = None
        if abs(p[0]) < X_ROOT: return None
        for k in range(3, 16):
            b = self.bones[(side, k)]
            if np.dot(p - b['pivot'], b['normal']) >= 0: best = (side, k)
        return best

    def segments(self, P):
        res = np.full(len(P), -1, int); sides = np.where(P[:, 0] > 0, 0, 1)
        for si, side in enumerate('LR'):
            for k in range(3, 16):
                b = self.bones[(side, k)]
                inside = (sides == si) & (np.abs(P[:, 0]) >= X_ROOT) & ((P - b['pivot']) @ b['normal'] >= 0)
                res[inside] = k
        return [None if k < 0 else ('LR'[s], int(k)) for s, k in zip(sides, res)]

    def chain_lines(self, bone, indent):
        if bone is None: return []
        side, k = bone; out = []; prev = np.zeros(3)
        for j in range(3, k + 1):
            b = self.bones[(side, j)]; d = b['pivot'] - prev; prev = b['pivot']
            t = '\t'.join(fmt(x) for x in d)
            out.append(f'{indent}ANIM_trans\t{t}\t{t}')
            out.append(f'{indent}ANIM_rotate_begin\t' + '\t'.join(fmt(x) for x in b['axis']) + f'\t{b["dataref"]}')
            for v, a in b['keys']: out.append(f'{indent}ANIM_rotate_key\t{fmt(v)}\t{fmt(a)}')
            out.append(f'{indent}ANIM_rotate_end')
        t = '\t'.join(fmt(-x) for x in prev)
        out.append(f'{indent}ANIM_trans\t{t}\t{t}')
        return out


# ---------------------------------------------------------------- OBJ8
GEOMETRY = {'TRIS'}
UNSUPPORTED = re.compile(r'^(LINES|LIGHT|LIGHTS|SMOKE|ANIM_hide|ANIM_show|ATTR_LOD|ANIM_keyframe_loop)')


class Obj8:
    def __init__(self, path):
        self.path = path
        lines = open(path, encoding='latin1').read().split('\n')
        self.head, self.vt, self.idx, self.cmds = [], [], [], []
        stage = 0
        for line in lines:
            s = line.strip(); w = s.split()[0] if s else ''
            if w == 'VT': self.vt.append(line); stage = max(stage, 1)
            elif w in ('IDX', 'IDX10'): self.idx.extend(int(x) for x in s.split()[1:]); stage = 2
            elif stage == 0: self.head.append(line)
            else: self.cmds.append(line)
        while self.cmds and not self.cmds[-1].strip(): self.cmds.pop()
        self.V = np.array([l.split()[1:4] for l in self.vt], float)
        self.I = np.array(self.idx, np.int64)
        for c in self.cmds:
            if UNSUPPORTED.match(c.strip()): raise ValueError(f'{path}: unsupported command {c.strip()}')

    def walk(self):
        """-> list of (kind, payload) in order: ('line', text) for state lines, ('tris', dict) for draw calls with
        the anim lines of all enclosing blocks (outermost first) and whether any of them moves."""
        out = []; stack = [[]]; pending = None
        for line in self.cmds:
            s = line.strip(); p = s.split(); w = p[0] if p else ''
            if w == 'ANIM_begin': stack.append([])
            elif w == 'ANIM_end': stack.pop()
            elif w in ('ANIM_trans_begin', 'ANIM_rotate_begin'): pending = [s]
            elif w in ('ANIM_trans_key', 'ANIM_rotate_key'): pending.append(s)
            elif w in ('ANIM_trans_end', 'ANIM_rotate_end'): pending.append(s); stack[-1].append(pending); pending = None
            elif w in ('ANIM_trans', 'ANIM_rotate'): stack[-1].append([s])
            elif w == 'TRIS':
                anims = [g for level in stack for g in level]
                out.append(('tris', dict(off=int(p[1]), cnt=int(p[2]), anims=anims, moving=any(moving(g) for g in anims))))
            else: out.append(('line', line))
        assert len(stack) == 1 and pending is None, self.path
        return out


def moving(group):
    p = group[0].split(); w = p[0]
    if w == 'ANIM_trans': return len(p) >= 10 and p[9] not in ('none', 'no_ref') and p[1:4] != p[4:7]
    if w == 'ANIM_rotate': return len(p) >= 9 and p[8] not in ('none', 'no_ref') and float(p[4]) != float(p[5])
    vals = [tuple(x.split()[2:]) for x in group[1:-1]]
    return len(set(vals)) > 1


def rot(axis, deg):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a); x, y, z = a
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg)); C = 1 - c; m = np.eye(4)
    m[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return m


def trans(v):
    m = np.eye(4); m[:3, 3] = v; return m


def interp(keys, v):
    keys = sorted(keys, key=lambda k: k[0])
    if v <= keys[0][0]: return np.asarray(keys[0][1], float)
    if v >= keys[-1][0]: return np.asarray(keys[-1][1], float)
    for (v0, a0), (v1, a1) in zip(keys, keys[1:]):
        if v0 <= v <= v1:
            t = 0 if v1 == v0 else (v - v0) / (v1 - v0)
            return np.asarray(a0, float) * (1 - t) + np.asarray(a1, float) * t


def matrix(anims, values):
    """world matrix of an animation stack (list of line groups) with dataref values (default 0)"""
    M = np.eye(4)
    def val(dr): return values.get(dr, 0.0)
    for g in anims:
        p = g[0].split(); w = p[0]
        if w == 'ANIM_trans':
            a = np.array(p[1:4], float); b = np.array(p[4:7], float)
            if len(p) >= 10 and p[9] not in ('none', 'no_ref'): a = interp([(float(p[7]), a), (float(p[8]), b)], val(p[9]))
            M = M @ trans(a)
        elif w == 'ANIM_rotate':
            ax = [float(x) for x in p[1:4]]; r0 = float(p[4]); r1 = float(p[5]) if len(p) > 5 else r0
            if len(p) >= 9 and p[8] not in ('none', 'no_ref'): r0 = float(interp([(float(p[6]), [r0]), (float(p[7]), [r1])], val(p[8]))[0])
            M = M @ rot(ax, r0)
        elif w == 'ANIM_trans_begin':
            keys = [(float(x.split()[1]), [float(y) for y in x.split()[2:5]]) for x in g[1:-1]]
            M = M @ trans(interp(keys, val(p[1])))
        elif w == 'ANIM_rotate_begin':
            keys = [(float(x.split()[1]), [float(x.split()[2])]) for x in g[1:-1]]
            M = M @ rot([float(x) for x in p[1:4]], float(interp(keys, val(p[4]))[0]))
    return M


def evaluate(obj, values, items=None):
    """world position of every IDX slot (by slot) for an Obj8 at dataref values"""
    items = items or obj.walk(); W = np.full((len(obj.I), 3), np.nan)
    for kind, t in items:
        if kind != 'tris': continue
        sl = slice(t['off'], t['off'] + t['cnt']); M = matrix(t['anims'], values)
        W[sl] = (np.c_[obj.V[obj.I[sl]], np.ones(t['cnt'])] @ M.T)[:, :3]
    return W


def clip_call(tri, verts, extra, M, planes, eps=1e-6):
    """Split triangles (rows of VT ids) exactly along world planes [(n, p, side_sign)] given the call's rest frame M.
    New vertices (position, normal, uv interpolated on the cut edge) are appended to `extra` (rows of 8 floats) with
    ids len(verts) + i. Returns (triangles, source triangle index)."""
    nv = len(verts)
    def rows(ids):
        ids = np.asarray(ids); out = np.empty((len(ids), 8))
        lo = ids < nv; out[lo] = verts[ids[lo]]
        if (~lo).any(): out[~lo] = np.array([extra[i - nv] for i in ids[~lo]])
        return out
    Minv = np.linalg.inv(M); T = np.asarray(tri); src = np.arange(len(T))
    for pi, (n_w, p_w, side) in enumerate(planes):
        n_l = M[:3, :3].T @ n_w; p_l = (Minv @ np.r_[p_w, 1.0])[:3]
        R = rows(T.reshape(-1)); W = (np.c_[R[:, :3], np.ones(len(R))] @ M.T)[:, :3]
        d = ((R[:, :3] - p_l) @ n_l).reshape(-1, 3)
        cx = W.reshape(-1, 3, 3).mean(1)[:, 0]
        relevant = (np.sign(cx) == side) & (np.abs(W[:, 0]).reshape(-1, 3).max(1) >= X_ROOT - 1e-6 if pi >= 0 else True)
        sg = np.where(d > eps, 1, np.where(d < -eps, -1, 0))
        cut = relevant & (sg.max(1) > 0) & (sg.min(1) < 0)
        if not cut.any(): continue
        keepT, keepS = [T[~cut]], [src[~cut]]; cache = {}
        def mid(u, v, du, dv):
            key = (min(u, v), max(u, v))
            if key not in cache:
                ru, rv = rows([u, v]); t = du / (du - dv); r = ru * (1 - t) + rv * t
                nrm = r[3:6]; ln = np.linalg.norm(nrm); r[3:6] = nrm / ln if ln > 0 else ru[3:6]
                extra.append(r); cache[key] = nv + len(extra) - 1
            return cache[key]
        newT, newS = [], []
        for ti in np.flatnonzero(cut):
            ids = list(T[ti]); ds = list(d[ti]); pos, neg = [], []
            for j in range(3):
                u, v, du, dv = ids[j], ids[(j + 1) % 3], ds[j], ds[(j + 1) % 3]
                su = 1 if du > eps else (-1 if du < -eps else 0); sv = 1 if dv > eps else (-1 if dv < -eps else 0)
                if su >= 0: pos.append(u)
                if su <= 0: neg.append(u)
                if su * sv < 0: m = mid(u, v, du, dv); pos.append(m); neg.append(m)
            for poly in (pos, neg):
                for j in range(1, len(poly) - 1): newT.append((poly[0], poly[j], poly[j + 1])); newS.append(src[ti])
        T = np.concatenate(keepT + [np.array(newT, np.int64).reshape(-1, 3)]); src = np.concatenate(keepS + [np.array(newS, np.int64)])
    return T, src


def process(obj, flex, engine_like):
    items = obj.walk(); verts = np.array([l.split()[1:9] for l in obj.vt], float); extra = []
    newI, out_cmds, srcs = [], [], []
    stats = dict(calls=0, split_calls=0, sub_calls=0, cut_triangles=0, new_vertices=0, bones={})
    seen = []
    for kind, t in items:
        if kind == 'tris':
            key = (t['off'], t['cnt'])
            for o in seen:
                if o[0] < key[0] + key[1] and key[0] < o[0] + o[1]: raise ValueError(f'{obj.path}: overlapping TRIS {o} {key}')
            seen.append(key)
    planes, cuts = [], []
    for side, sgn in (('L', 1.0), ('R', -1.0)):
        cuts.append((np.array([sgn, 0.0, 0.0]), np.array([sgn * X_ROOT, 0.0, 0.0]), sgn))
        for k in range(3, 16):
            b = flex.bones[(side, k)]; cuts.append((b['normal'], b['pivot'], sgn))
    for n, p, sgn in cuts:
        for o in (0.0, -BAND, BAND): planes.append((n, p + o * n, sgn))
    for kind, t in items:
        if kind == 'line':
            out_cmds.append(t if not t.strip().startswith('#') else t.strip()); continue
        stats['calls'] += 1
        off, cnt = t['off'], t['cnt']; tri = obj.I[off:off + cnt].reshape(-1, 3); M = matrix(t['anims'], {})
        if engine_like:
            T, src = tri, np.arange(len(tri))
        else:
            T, src = clip_call(tri, verts, extra, M, planes)
            stats['cut_triangles'] += int(len(set(src.tolist())) - (np.bincount(src) == 1).sum())
        R = np.empty((T.size, 8)); flat = T.reshape(-1); lo = flat < len(verts)
        R[lo] = verts[flat[lo]]
        if (~lo).any(): R[~lo] = np.array([extra[i - len(verts)] for i in flat[~lo]])
        C = (np.c_[R[:, :3], np.ones(len(R))] @ M.T)[:, :3].reshape(-1, 3, 3).mean(1)
        if engine_like:
            if t['moving']: C = np.repeat(C.mean(0, keepdims=True), len(C), 0)     # rotating fans: one engine per call
            bones = [(('L' if c[0] > 0 else 'R'), 6 if abs(c[0]) < ENGINE_SPLIT_X else 11) for c in C]
        else:
            # a triangle within BAND of a cut also belongs to the segment on the other side of it
            sets = [{b} for b in flex.segments(C)]
            for n, p, sgn in cuts:
                near = np.flatnonzero((np.sign(C[:, 0]) == sgn) & (np.abs((C - p) @ n) < BAND))
                if not len(near): continue
                for o in (-BAND, BAND):
                    for i, b in zip(near, flex.segments(C[near] + o * n)): sets[i].add(b)
            stats['band_triangles'] = stats.get('band_triangles', 0) + sum(len(x) > 1 for x in sets)
            bones = sets
        if engine_like: bones = [{b} for b in bones]
        order = sorted(set().union(*bones), key=lambda b: (b is not None, b or ('', 0)))
        if len(order) > 1: stats['split_calls'] += 1
        for ob in order:
            g = [i for i, b in enumerate(bones) if ob in b]; pos = len(newI); n = 3 * len(g)
            newI.extend(T[g].reshape(-1).tolist()); srcs.extend((off + 3 * src[g]).tolist())
            stats['sub_calls'] += 1; stats['bones'][str(ob)] = stats['bones'].get(str(ob), 0) + len(g)
            out_cmds.append('ANIM_begin')
            out_cmds += flex.chain_lines(ob, '\t')
            for grp in t['anims']: out_cmds += ['\t' + x for x in grp]
            out_cmds.append(f'\tTRIS\t{pos} {n}')
            out_cmds.append('ANIM_end')
    stats['new_vertices'] = len(extra)
    return np.array(newI, np.int64), out_cmds, stats, np.array(srcs, np.int64), extra


def write(obj, I, cmds, path, extra):
    nvt = len(obj.vt) + len(extra)
    with open(path, 'w', encoding='latin1', newline='\n') as f:
        for line in obj.head:
            if line.strip().startswith('POINT_COUNTS'):
                p = line.split(); line = f'POINT_COUNTS\t{nvt}\t{p[2]}\t{p[3]}\t{len(I)}'
            f.write(line + '\n')
        for line in obj.vt: f.write(line + '\n')
        for r in extra: f.write('VT\t' + '\t'.join(fmt(x) for x in r) + '\n')
        n10 = len(I) // 10 * 10
        for i in range(0, n10, 10): f.write('IDX10\t' + ' '.join(map(str, I[i:i + 10])) + '\n')
        for i in range(n10, len(I)): f.write(f'IDX\t{I[i]}\n')
        f.write('\n')
        for line in cmds: f.write(line + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--gltf', required=True); ap.add_argument('--config', required=True)
    ap.add_argument('--out', required=True); ap.add_argument('--report'); ap.add_argument('--perm'); ap.add_argument('objs', nargs='+')
    a = ap.parse_args()
    flex = Flex(a.gltf, a.config); os.makedirs(a.out, exist_ok=True); rep = dict(bones=flex.report, max_bone_translation_m=flex.max_trans, objects={}); print('max bone translation in the clips (ignored):', flex.max_trans)
    for path in a.objs:
        obj = Obj8(path); I, cmds, stats, src, extra = process(obj, flex, bool(ENGINE_OBJECTS.search(os.path.basename(path))))
        write(obj, I, cmds, os.path.join(a.out, os.path.basename(path)), extra)
        if a.perm: os.makedirs(a.perm, exist_ok=True); np.save(os.path.join(a.perm, os.path.basename(path) + '.src.npy'), src)
        rep['objects'][os.path.basename(path)] = stats; print(os.path.basename(path), json.dumps(stats))
    if a.report: json.dump(rep, open(a.report, 'w'), indent=1)


if __name__ == '__main__':
    main()
