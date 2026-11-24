"""A380X landing gear: spinning wheels and steering (2026-11-24).

Adds animation to the gear objects (objects/exterior/gear-*), each moving draw call wrapped in its own ANIM block in
its own local frame (so it still follows the gear extension):
  * spin: every tyre/wheel about its axle with sim/flightmodel2/gear/tire_rotation_angle_deg[n] (0..360, + rolling
    forward = + about object +x, the objects being attached yawed 180 deg);
  * nose steering: the lower strut (steering collar, torque links, sliding piston, axle, parts below it) and both
    wheels about the strut axis (fitted from the piston / main fitting rings) with a380x/xp12/gear/steer_vis_deg[0]
    (degrees + right, from the A380XDoors plugin: X-Plane's enacted steering on the ground, centred in the air);
  * body wheel steering: the aft axle (axle beam + its two wheels) of each body gear about the vertical through the
    axle centre with steer_vis_deg[3] / [4]. FlyByWire's model rests at full body wheel steering (aft axle turned
    ~15 deg right); the steering keys include that offset, so the axle is straight at 0.
  * main gear bogie tilt: each wing / body gear bogie (the node holding its tyres and that node's children, minus
    the sliding piston, which shares the wing gear's bogie node) pitches about its pivot pin (fitted
    from the piston's pin rings) with a380x/xp12/gear/bogie_tilt_deg[n] (+ = rear wheels down, from the A380XDoors
    plugin); a bogie that FlyByWire left unlevel (wing RH, 1.8 deg front down) is levelled at 0 through the keys.
Nesting: tilt outside, then steering, spin inside. Geometry rules (no hard-coded line numbers except the nose strut parts, given
by draw-call offset/count): see SPEC below.
Usage: gear_wheels.py <out dir> <source objects/exterior dir> [more source dirs; the first that has a file wins]
"""
import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gearobj import Obj
from gtree import tree, subtree

SPIN = 'sim/flightmodel2/gear/tire_rotation_angle_deg[%d]'
STEER = 'a380x/xp12/gear/steer_vis_deg[%d]'
TILT = 'a380x/xp12/gear/bogie_tilt_deg[%d]'
REST_LEVEL_MIN_DEG = 0.5
# nose strut draw calls (offset, count) that turn with the steering: collar, gland, lower / upper torque link, axle
# housing, parts below the axle, sliding piston
NOSE_STEER_CALLS = {(67482, 29547), (20718, 11274), (34956, 9732), (154551, 5472), (44688, 6990), (51678, 15144),
                    (66822, 660), (101355, 1152)}
NOSE_RING_CALLS = {(101355, 1152), (102507, 18636)}   # sliding piston, main fitting: strut axis
FILES = {
    'nose': ('gear-nose/gear_nose_a380_exterior_tyres.obj', 'gear-nose/gear_nose_a380_exterior_nlg.obj'),
    'wing_lh': ('gear-wing-lh/gear_wing_lh_a380_exterior_tyres.obj', 'gear-wing-lh/gear_wing_lh_a380x_wlg.obj',
                'gear-wing-lh/gear_wing_lh_a380_exterior_wing1.obj'),
    'wing_rh': ('gear-wing-rh/gear_wing_rh_a380_exterior_tyres.obj', 'gear-wing-rh/gear_wing_rh_a380x_wlg.obj',
                'gear-wing-rh/gear_wing_rh_a380_exterior_wing2.obj'),
    'body_lh': ('gear-body-lh/gear_body_lh_a380_exterior_tyres.obj', 'gear-body-lh/gear_body_lh_a380x_blg.obj'),
    'body_rh': ('gear-body-rh/gear_body_rh_a380_exterior_tyres.obj', 'gear-body-rh/gear_body_rh_a380x_blg.obj'),
}
GEAR_INDEX = {'nose': 0, 'wing_lh': 1, 'wing_rh': 2, 'body_lh': 3, 'body_rh': 4}


def wheels(P):
    """Split a tyre draw call into wheels (gap in x) -> list of (centre, points)."""
    xs = np.sort(P[:, 0]); gaps = np.diff(xs); k = int(np.argmax(gaps))
    parts = [P] if gaps[k] < 0.08 else [P[P[:, 0] <= xs[k]], P[P[:, 0] > xs[k]]]
    return [((Q.min(0) + Q.max(0)) / 2, Q) for Q in parts]


def circle_fit(Q):
    A = np.c_[2 * Q, np.ones(len(Q))]; b = (Q ** 2).sum(1); s = np.linalg.lstsq(A, b, rcond=None)[0]
    return s[:2], np.sqrt(s[2] + s[:2] @ s[:2])


def nose_axis(nlg):
    pts = []
    for c in nlg.calls(1.0):
        if (c['off'], c['cnt']) not in NOSE_RING_CALLS: continue
        P = nlg.world(c)
        for y0 in np.arange(P[:, 1].min(), P[:, 1].max(), 0.1):
            Q = P[(P[:, 1] >= y0) & (P[:, 1] < y0 + 0.1)]
            if len(Q) < 40: continue
            cc, r = circle_fit(Q[:, [0, 2]])
            if r > 0.05 and np.abs(np.hypot(*(Q[:, [0, 2]] - cc).T) - r).mean() < 0.01:
                pts.append([cc[0], y0 + 0.05, cc[1]])
    pts = np.array(pts); m = pts.mean(0); d = np.linalg.svd(pts - m)[2][0]
    d = d if d[1] > 0 else -d                                   # up
    return m, d, len(pts)


def pin_pivot(objs):
    """The sliding piston (reaches from above y -3.4 to below -4.1, narrow) and its pivot pin ring centre."""
    for o in objs:
        for c in o.calls(1.0):
            P = o.world(c)
            if P[:, 1].max() > -3.4 and P[:, 1].min() < -4.1 and np.ptp(P[:, 2]) < 0.7 and np.ptp(P[:, 0]) < 0.8:
                Q = P[P[:, 1] < -3.85]; cc, r = circle_fit(Q[:, [2, 1]])
                for _ in range(5):
                    d = np.hypot(Q[:, 2] - cc[0], Q[:, 1] - cc[1]); sel = Q[(d > 0.03) & (d < 0.08)]
                    cc, r = circle_fit(sel[:, [2, 1]])
                return np.array([np.median(Q[:, 0]), cc[1], cc[0]]), (id(o), c['off'], c['cnt'])
    raise RuntimeError('no piston found')


def tilt_plan(objs, tyre_obj, n, axles):
    """-> ({id(obj): {line: world tilt op}}, report) for a main gear."""
    nodes_t = tree(tyre_obj)
    bog = {nd['i'] for nd in nodes_t for (off, cnt, ln) in nd['calls']}
    assert len(bog) == 1, bog
    S = subtree(nodes_t, bog.pop())
    pivot, piston = pin_pivot(objs)
    straight = sorted(axles, key=lambda a: a[2])                     # (x, y, z) centres of straight axles, by z
    rear, front = straight[0], straight[-1]
    rest = float(np.degrees(np.arctan2(front[1] - rear[1], front[2] - rear[2])))   # + = front up
    corr = -rest if abs(rest) >= REST_LEVEL_MIN_DEG else 0.0
    op = ('tilt', pivot, np.array([-1.0, 0.0, 0.0]), TILT % n, [(-30, -30.0 + corr), (30, 30.0 + corr)])
    out = {}; count = 0
    for o in objs:
        nodes = tree(o); assert len(nodes) == len(nodes_t)
        lines = {ln for nd in nodes if nd['i'] in S for (off, cnt, ln) in nd['calls']}
        for c in o.calls(1.0):
            if c['line'] in lines and (id(o), c['off'], c['cnt']) != piston:
                out.setdefault(id(o), {})[c['line']] = (c, op); count += 1
    return out, dict(pivot=np.round(pivot, 4).tolist(), piston_call=list(piston[1:]), rest_front_up_deg=round(rest, 2),
                     level_correction_deg=round(corr, 2), tilted_calls=count, bogie_nodes=sorted(S))


def block(indent, ops):
    """ops: list of ('steer'|'spin', point, axis, dataref, keys) in local coordinates."""
    t = indent
    out = [t + 'ANIM_begin']
    for kind, p, a, ref, keys in ops:
        out.append(t + 'ANIM_trans\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f' % (*p, *p))
        out.append(t + 'ANIM_rotate_begin\t%.6f\t%.6f\t%.6f\t%s' % (*a, ref))
        out += [t + 'ANIM_rotate_key\t%g\t%.6f' % kv for kv in keys]
        out.append(t + 'ANIM_rotate_end')
        out.append(t + 'ANIM_trans\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f' % (*(-p), *(-p)))
    return out


def local(M, p=None, d=None):
    Mi = np.linalg.inv(M)
    if p is not None: return (Mi @ np.r_[p, 1.0])[:3]
    v = Mi[:3, :3] @ d; return v / np.linalg.norm(v)


def resolver(roots):
    def path(rel):
        for r in roots:
            if os.path.exists(os.path.join(r, rel)): return os.path.join(r, rel)
        raise FileNotFoundError(rel)
    return path


def plan(path):
    """-> {rel: {tris_line: (call, [ops in world])}}, report. path(rel) -> source file."""
    rep = {}; todo = {}; objs = {}
    def load(rel):
        if rel not in objs: objs[rel] = Obj(path(rel))
        return objs[rel]
    def add(rel, c, ops, front=False):
        cur = todo.setdefault(rel, {}).get(c['line'])
        if cur is None: todo[rel][c['line']] = (c, list(ops))
        else: todo[rel][c['line']] = (c, list(ops) + cur[1] if front else cur[1] + list(ops))
    spin_keys = [(0, 0.0), (360, 360.0)]
    for gear, files in FILES.items():
        n = GEAR_INDEX[gear]
        tyres = load(files[0]); tcalls = tyres.calls(1.0)
        if gear == 'nose':
            nlg = load(files[1])
            m, up, nring = nose_axis(nlg)
            wl = [wheels(tyres.world(c)) for c in tcalls]
            y_axle = np.mean([w[0][1] for ws in wl for w in ws])
            pivot = m + up * (y_axle - m[1]) / up[1]                   # strut axis at axle height
            steer = ('steer', pivot, -up, STEER % n, [(-90, -90.0), (90, 90.0)])
            rep[gear] = dict(axis_up=np.round(up, 4).tolist(), rake_deg=round(float(np.degrees(np.arctan2(-up[2], up[1]))), 2),
                             pivot=np.round(pivot, 4).tolist(), rings=nring, wheels=[])
            for c, ws in zip(tcalls, wl):
                (cen, _), = ws
                add(files[0], c, [steer, ('spin', cen, np.array([1.0, 0, 0]), SPIN % n, spin_keys)])
                rep[gear]['wheels'].append(np.round(cen, 4).tolist())
            k = 0
            for c in nlg.calls(1.0):
                if (c['off'], c['cnt']) in NOSE_STEER_CALLS: add(files[1], c, [steer]); k += 1
            rep[gear]['strut_calls'] = k
            assert k == len(NOSE_STEER_CALLS)
            continue
        rep[gear] = dict(axles=[]); straight = []
        aft = []
        for c in tcalls:
            ws = wheels(tyres.world(c))
            if len(ws) == 1:
                aft.append((c, ws[0][0]))
            else:
                cen = np.mean([w[0] for w in ws], 0); straight.append(cen)
                add(files[0], c, [('spin', cen, np.array([1.0, 0, 0]), SPIN % n, spin_keys)])
                rep[gear]['axles'].append(dict(centre=np.round(cen, 4).tolist(), wheels=len(ws)))
        if gear.startswith('body'):
            assert len(aft) == 2, gear
            (ca, pa), (cb, pb) = sorted(aft, key=lambda t: t[1][0])
            axle = pb - pa; axle[1] = 0.0; axle /= np.linalg.norm(axle)
            baked = float(np.degrees(np.arctan2(axle[2], axle[0])))   # + = turned right (about object -y)
            pivot = (pa + pb) / 2
            steer = ('steer', pivot, np.array([0.0, -1.0, 0.0]), STEER % n, [(-90, -90.0 - baked), (90, 90.0 - baked)])
            spin_axis = (pb - pa) / np.linalg.norm(pb - pa)
            for c, cen in aft:
                add(files[0], c, [steer, ('spin', cen, spin_axis, SPIN % n, spin_keys)])
            blg = load(files[1]); beam = []
            for c in blg.calls(1.0):
                P = blg.world(c); ctr = (P.min(0) + P.max(0)) / 2
                if np.hypot(ctr[0] - pivot[0], ctr[2] - pivot[2]) < 0.1 and np.ptp(P[:, 0]) > 1.4: beam.append(c)
            assert len(beam) == 1, (gear, len(beam))
            add(files[1], beam[0], [steer])
            rep[gear].update(aft_pivot=np.round(pivot, 4).tolist(), aft_baked_right_deg=round(baked, 2),
                             aft_axle_call=[beam[0]['off'], beam[0]['cnt']])
        # bogie tilt, outermost
        gobjs = [load(f) for f in files]
        tilt, trep = tilt_plan(gobjs, tyres, n, straight)
        for rel in files:
            for line, (c, op) in tilt.get(id(load(rel)), {}).items(): add(rel, c, [op], front=True)
        rep[gear]['tilt'] = trep
    return todo, rep, objs


def write(objs, dst, todo):
    for rel, calls in todo.items():
        o = objs[rel]; L = o.L; out = []
        for n, line in enumerate(L):
            if n in calls:
                c, ops = calls[n]; M = c['M']; indent = line[:len(line) - len(line.lstrip())]
                lops = [(k, local(M, p=p), local(M, d=a), ref, keys) for k, p, a, ref, keys in ops]
                out += block(indent, lops); out.append(line); out.append(indent + 'ANIM_end')
            else:
                out.append(line)
        os.makedirs(os.path.dirname(os.path.join(dst, rel)), exist_ok=True)
        open(os.path.join(dst, rel), 'w').write('\n'.join(out))


if __name__ == '__main__':
    dst, roots = sys.argv[1], sys.argv[2:]
    todo, rep, objs = plan(resolver(roots))
    write(objs, dst, todo)
    rep['wrapped_calls'] = {k: len(v) for k, v in todo.items()}
    rep['sources'] = {k: resolver(roots)(k) for k in todo}
    json.dump(rep, open(os.path.join(dst, 'wheels_report.json'), 'w'), indent=1)
    print(json.dumps({g: (v if g in ('wrapped_calls',) else {kk: vv for kk, vv in v.items() if kk in ('tilt', 'aft_baked_right_deg', 'rake_deg')}) for g, v in rep.items() if g != 'sources'}, indent=1))
