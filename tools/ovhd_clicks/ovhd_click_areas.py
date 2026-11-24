"""Overhead pushbuttons: click spots over the whole button field (A380X, 2026-11-15).

The overhead pushbutton click spots in a380x_cockpit_controls.obj are invisible quads of about 17 x 17 mm, smaller
than the button face, so clicking near the edge of a button did nothing. Each one is grown, in its own plane and
about its own centre, to TARGET (the full button field) - never closer than GAP to the half-way line towards a
neighbouring control on the same panel surface (other pushbuttons grow too, so the space between two buttons is
shared), and never smaller than it was. Controls stacked on the button by design (a guard over its button) are not
neighbours. Only vertex positions change; commands, tooltips, draw order and every other control are untouched.

Usage: ovhd_click_areas.py <in cockpit_controls.obj> <out.obj> <report.json>
"""
import json, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xp2wave3 as xw

REGION_LO = np.array([-0.33, 3.30, 31.60])     # overhead panel (object metres)
REGION_HI = np.array([0.33, 3.78, 32.56])
TARGET = 0.0135        # half size: 27 mm button field
GAP = 0.0004           # kept free on each side of the half-way line (0.8 mm between two buttons)
SAME_SURFACE = 0.030   # neighbours within this distance along the normal
STACKED = 0.006        # centres closer than this belong to the same control (guard / button)


def parse(path):
    lines = open(path, errors='replace').read().split('\n')
    vt_rows, idx, calls = [], [], []
    stack, pend, manip, draw = [np.eye(4)], None, None, True
    for n, line in enumerate(lines):
        p = line.split()
        if not p:
            continue
        k = p[0]
        if k == 'VT': vt_rows.append(n)
        elif k in ('IDX10', 'IDX'): idx.extend(int(x) for x in p[1:])
        elif k == 'ANIM_begin': stack.append(stack[-1].copy())
        elif k == 'ANIM_end': stack.pop()
        elif k in ('ANIM_trans_begin', 'ANIM_rotate_begin'):
            pend = ([float(x) for x in p[1:4]] if 'rotate' in k else None, [])
        elif k == 'ANIM_trans_key': pend[1].append((float(p[1]), [float(x) for x in p[2:5]]))
        elif k == 'ANIM_rotate_key': pend[1].append((float(p[1]), [float(p[2])]))
        elif k == 'ANIM_trans_end': stack[-1] = stack[-1] @ xw.tr(xw.interp(pend[1], 0.0))
        elif k == 'ANIM_rotate_end': stack[-1] = stack[-1] @ xw.rot(pend[0], float(xw.interp(pend[1], 0.0)[0]))
        elif k == 'ANIM_trans': stack[-1] = stack[-1] @ xw.tr(np.array(p[1:4], float))
        elif k == 'ANIM_rotate': stack[-1] = stack[-1] @ xw.rot([float(x) for x in p[1:4]], float(p[4]))
        elif k.startswith('ATTR_manip_'): manip = None if k == 'ATTR_manip_none' else line.strip()
        elif k == 'ATTR_draw_disable': draw = False
        elif k == 'ATTR_draw_enable': draw = True
        elif k == 'TRIS' and manip:
            calls.append(dict(manip=manip, off=int(p[1]), cnt=int(p[2]), M=stack[-1].copy(), draw=draw,
                              animated=len(stack) > 1))
    V = np.array([[float(x) for x in lines[r].split()[1:4]] for r in vt_rows])
    return lines, vt_rows, np.array(idx), V, calls


def main(src, dst, report_path):
    lines, vt_rows, I, V, calls = parse(src)
    use = np.bincount(I, minlength=len(V))
    items = []
    for c in calls:
        ids = I[c['off']:c['off'] + c['cnt']]
        P = (np.c_[V[ids], np.ones(len(ids))] @ c['M'].T)[:, :3]
        ctr = (P.min(0) + P.max(0)) / 2
        if not (np.all(ctr >= REGION_LO) and np.all(ctr <= REGION_HI)):
            continue
        t = c['manip'].split('\t')
        quad = c['cnt'] == 6 and len(set(ids.tolist())) == 4 and len(t) > 1 and t[1] == 'button' and not c['draw']
        items.append(dict(call=c, ids=ids, P=P, ctr=ctr, quad=quad, name=t[-1] if len(t) > 2 else t[0]))
    report, changed = [], 0
    for it in items:
        if not it['quad']:
            continue
        ids = sorted(set(it['ids'].tolist()))
        c = it['call']
        if any(use[i] != it['ids'].tolist().count(i) for i in ids):
            report.append(dict(name=it['name'], skipped='vertices shared with other geometry')); continue
        M = c['M']; Minv = np.linalg.inv(M)
        Q = (np.c_[V[ids], np.ones(len(ids))] @ M.T)[:, :3]     # world (rest pose); written back in local space
        ctr = Q.mean(0)
        # quad axes: the two edge directions from the first vertex to its neighbours
        d = Q - Q[0]
        order = np.argsort(np.linalg.norm(d, axis=1))
        u = d[order[1]]; v = d[order[2]]
        a0, b0 = np.linalg.norm(u) / 2, np.linalg.norm(v) / 2
        u, v = u / np.linalg.norm(u), v / np.linalg.norm(v)
        nrm = np.cross(u, v); nrm /= np.linalg.norm(nrm)
        a, b = max(a0, TARGET), max(b0, TARGET)
        by_a = by_b = None
        for other in items:
            if other is it:
                continue
            rel = other['P'] - ctr
            du_all, dv_all, dn_all = rel @ u, rel @ v, rel @ nrm
            if abs(((other['ctr'] - ctr) @ nrm)) > SAME_SURFACE:
                continue
            ou = (du_all.min() + du_all.max()) / 2; hu = (du_all.max() - du_all.min()) / 2
            ov = (dv_all.min() + dv_all.max()) / 2; hv = (dv_all.max() - dv_all.min()) / 2
            if np.hypot(ou, ov) < STACKED:
                continue
            grows = other['quad']
            # limit along u if the neighbour lies beside us (overlaps our v band), and along v likewise
            if abs(ov) < b + hv and abs(ou) > hu:
                lim = abs(ou) / 2 - GAP if grows else abs(ou) - hu - GAP
                if max(a0, lim) < a: a, by_a = max(a0, lim), (other['name'][:40], round(1000 * float(np.hypot(ou, ov)), 1))
            if abs(ou) < a + hu and abs(ov) > hv:
                lim = abs(ov) / 2 - GAP if grows else abs(ov) - hv - GAP
                if max(b0, lim) < b: b, by_b = max(b0, lim), (other['name'][:40], round(1000 * float(np.hypot(ou, ov)), 1))
        for k, i in enumerate(ids):
            r = Q[k] - ctr
            su, sv = np.sign(r @ u), np.sign(r @ v)
            p = ctr + su * a * u + sv * b * v + (r @ nrm) * nrm
            p = (Minv @ np.r_[p, 1.0])[:3]
            q = lines[vt_rows[i]].split('\t') if '\t' in lines[vt_rows[i]] else lines[vt_rows[i]].split()
            q[1:4] = ['%.6f' % x for x in p]
            lines[vt_rows[i]] = '\t'.join(q)
        changed += 1
        report.append(dict(name=it['name'], before_mm=[round(2000 * a0, 1), round(2000 * b0, 1)],
                           after_mm=[round(2000 * a, 1), round(2000 * b, 1)], limited_by=[by_a, by_b]))
    open(dst, 'w').write('\n'.join(lines))
    json.dump(report, open(report_path, 'w'), indent=1)
    sizes = np.array([r['after_mm'] for r in report if 'after_mm' in r])
    print(f'overhead controls {len(items)}, pushbutton click spots enlarged {changed}, skipped '
          f'{sum(1 for r in report if "skipped" in r)}; new size mm min {sizes.min(0)} median {np.median(sizes, 0)} max {sizes.max(0)}')


if __name__ == '__main__':
    main(*sys.argv[1:4])
