"""Numerical checks of the gear_wheels.py output against the original gear objects.
Usage: check_wheels.py <new objects dir> <original source dirs...>"""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gearobj import Obj
from gear_wheels import FILES, GEAR_INDEX, SPIN, STEER, TILT, wheels, resolver
dst, roots = sys.argv[1], sys.argv[2:]
src = resolver(roots)
rep = json.load(open(os.path.join(dst, 'wheels_report.json'))); ok = True
def say(cond, msg):
    global ok; ok &= bool(cond); print(('OK   ' if cond else 'FAIL ') + msg)
for gear, files in FILES.items():
    n = GEAR_INDEX[gear]; tr = rep[gear].get('tilt')
    for rel in files:
        if not os.path.exists(os.path.join(dst, rel)): continue
        a, b = Obj(src(rel)), Obj(os.path.join(dst, rel))
        for ratio in (1.0, 0.5, 0.0):
            ca, cb = a.calls(ratio), b.calls(ratio); assert len(ca) == len(cb)
            moved = [(x['off'], round(float(np.abs(a.world(x) - b.world(y)).max()), 3)) for x, y in zip(ca, cb)
                     if np.abs(a.world(x) - b.world(y)).max() > 1e-4]
            if gear.startswith('body'): exp = 2 if 'tyres' in rel else 1
            elif tr and tr['level_correction_deg']: exp = None          # whole bogie levelled
            else: exp = 0
            if exp is None:
                say(all(d < 0.06 for _, d in moved), f'{rel} deploy {ratio}: bogie levelled ({len(moved)} calls moved, max {max([d for _, d in moved] or [0])} m)')
            else:
                say(len(moved) == exp, f'{rel} deploy {ratio}: neutral pose unchanged except {exp} aft-axle calls (moved {moved})')
        if 'tyres' in rel:
            vals = {SPIN % n: 137.0}
            for x, y in zip(b.calls(1.0), b.calls(1.0, vals)):
                P0, P1 = b.world(x), b.world(y)
                dc = np.abs((P0.min(0) + P0.max(0)) / 2 - (P1.min(0) + P1.max(0)) / 2).max()
                say(dc < 0.004 and np.abs(P0 - P1).max() > 0.3, f'{rel} call {x["off"]}: spins in place (centre moved {dc*1000:.1f} mm)')
    if gear == 'nose' or gear.startswith('body'):
        t = Obj(os.path.join(dst, files[0]))
        for s in ((0.0, 8.0, 30.0) if gear == 'nose' else (0.0, 8.0, -8.0)):
            C = np.array([cen for c in t.calls(1.0, {STEER % n: s}) for cen, _ in wheels(t.world(c))])
            if gear == 'nose': L, R = C[np.argmax(C[:, 0])], C[np.argmin(C[:, 0])]
            else:
                aft = C[C[:, 2] < -3.0]; L, R = aft[np.argmax(aft[:, 0])], aft[np.argmin(aft[:, 0])]
            yaw = np.degrees(np.arctan2(L[2] - R[2], L[0] - R[0]))
            say(abs(yaw - s) < 0.3, f'{gear} steer {s:+.0f}: axle turned {yaw:+.2f} deg (+ = right)')
    if tr:
        piv = np.array(tr['pivot']); t = Obj(os.path.join(dst, files[0]))
        def axles(vals):
            out = []
            for c in t.calls(1.0, vals):
                ws = wheels(t.world(c)); out.append(np.mean([w[0] for w in ws], 0))
            return np.array(sorted(out, key=lambda q: q[2]))
        A0 = axles({}); A10 = axles({TILT % n: 10.0})
        level = np.degrees(np.arctan2(A0[-1][1] - A0[0][1], A0[-1][2] - A0[0][2]))
        say(abs(level) < 0.5, f'{gear} tilt 0: bogie level within the 0.5 deg levelling threshold ({level:+.2f} deg front up)')
        for q0, q1 in zip(A0, A10):
            dz, dy = q0[2] - piv[2], q0[1] - piv[1]
            exp = dz * np.sin(np.radians(10)) + dy * (np.cos(np.radians(10)) - 1)       # rise (front +, rear -)
            say(abs((q1[1] - q0[1]) - exp) < 0.01, f'{gear} tilt 10: axle at z {q0[2]:+.2f} moves {q1[1]-q0[1]:+.3f} m (expected {exp:+.3f})')
        # strut parts (not on the bogie) stay put
        strut = Obj(os.path.join(dst, files[1])); sa = strut.calls(1.0); sb = strut.calls(1.0, {TILT % n: 10.0})
        still = sum(1 for x, y in zip(sa, sb) if np.abs(strut.world(x) - strut.world(y)).max() < 1e-4)
        moved = len(sa) - still
        say(moved == tr['tilted_calls'] - len(t.calls(1.0)), f'{gear} tilt 10: {moved} strut-object calls move (bogie), {still} stay (piston, links, leg)')
        if gear.startswith('body'):
            v = {TILT % n: 10.0, STEER % n: 8.0}
            C = np.array([cen for c in t.calls(1.0, v) for cen, _ in wheels(t.world(c))]); aft = C[C[:, 2] < -3.0]
            beam = [c for c in strut.calls(1.0, v) if (c['off'], c['cnt']) == tuple(rep[gear]['aft_axle_call'])][0]
            P = strut.world(beam); bc = (P.min(0) + P.max(0)) / 2; mid = aft.mean(0)
            say(np.linalg.norm(bc[[0, 2]] - mid[[0, 2]]) < 0.03 and abs(bc[1] - mid[1]) < 0.06,
                f'{gear} tilt 10 + steer 8: aft wheels stay on the aft axle (offset {np.linalg.norm(bc - mid)*1000:.0f} mm)')
print('ALL OK' if ok else 'SOME CHECKS FAILED')
