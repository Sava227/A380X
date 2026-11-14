"""A380X wing spoiler panels and ground-spoiler ailerons (on the GitHub c84f813 wing objects).

Spoilers: every spoiler segment rotation (a380x/xp12/hydraulics/spoiler_{l|r}_{group}_deg) is re-pointed to its
A380 panel, a380x/xp12/surfaces/spoiler_deg[i] (left 0-7, right 8-15 = panels 1-8 inboard to outboard, from the
hinge position: two segments per panel), which the A380XDoors plugin computes from the PRIM panel orders and the roll
spoilers. The key tables (degrees in, degrees out) are unchanged.

Ailerons: after each aileron's hinge rotation (ANIM_rotate_begin 0 0 -1 a380x/xp12/hydraulics/aileron_*_ratio) an
extra rotation about the same hinge is inserted, driven by a380x/xp12/surfaces/aileron_ground_up (0..1): 40 deg
trailing edge up (the ACF's ground speed-brake aileron deflection; left +40, right -40 in the hinge frame, from the
existing ratio -1 = left trailing edge up / +1 = right trailing edge up). Lift dumping on landing.

Idempotent. Usage: wing_surfaces.py <in.obj> <out.obj>
"""
import re, sys

PANEL_EDGES = [8.9, 11.0, 13.1, 15.15, 17.05, 19.4, 21.6]     # |x| of the hinge between panels 1|2 ... 7|8 (m)
AIL_UP_DEG = 40.0
SPOILER = re.compile(r'^(\s*ANIM_rotate_begin\s+\S+\s+\S+\s+\S+\s+)a380x/xp12/hydraulics/spoiler_([lr])_[a-z]+_deg(\s*)$')
AILERON = re.compile(r'^(\s*)ANIM_rotate_begin\s+0\s+0\s+-1\s+a380x/xp12/hydraulics/aileron_([lr])_[a-z]+_ratio\s*$')


def panel(x):
    ax = abs(x)
    for i, e in enumerate(PANEL_EDGES):
        if ax < e:
            return i + 1
    return 8


def main(src, dst):
    lines = open(src, errors='replace').read().split('\n')
    if any('a380x/xp12/surfaces/' in l for l in lines):
        print('already converted:', src)
        open(dst, 'w').write('\n'.join(lines))
        return
    out, stack, pending_ail = [], [[]], None
    n_sp = n_ail = 0
    counts = {}
    for l in lines:
        p = l.split()
        if p:
            if p[0] == 'ANIM_begin':
                stack.append([])
            elif p[0] == 'ANIM_end':
                stack.pop()
            elif p[0] == 'ANIM_trans' and len(p) >= 4:
                stack[-1].append(float(p[1]))
        m = SPOILER.match(l)
        if m:
            x = [v for lv in stack for v in lv][-1]
            side = m.group(2)
            if (side == 'l') != (x > 0):
                raise ValueError(f'spoiler side {side} with hinge x {x}')
            idx = (8 if side == 'r' else 0) + panel(x) - 1
            counts[idx] = counts.get(idx, 0) + 1
            out.append(f'{m.group(1)}a380x/xp12/surfaces/spoiler_deg[{idx}]{m.group(3)}')
            n_sp += 1
            continue
        m = AILERON.match(l)
        if m:
            pending_ail = (m.group(1), AIL_UP_DEG if m.group(2) == 'l' else -AIL_UP_DEG)
        out.append(l)
        if pending_ail and p and p[0] == 'ANIM_rotate_end':
            ind, deg = pending_ail
            out += [f'{ind}ANIM_rotate_begin\t0\t0\t-1\ta380x/xp12/surfaces/aileron_ground_up',
                    f'{ind}ANIM_rotate_key\t0\t0', f'{ind}ANIM_rotate_key\t1\t{deg:.1f}', f'{ind}ANIM_rotate_end']
            pending_ail = None
            n_ail += 1
    open(dst, 'w').write('\n'.join(out))
    per_panel = ' '.join(f'{"L" if i < 8 else "R"}{i % 8 + 1}:{counts[i]}' for i in sorted(counts))
    print(f'{src.split("/")[-1]}: spoiler rotations {n_sp} ({per_panel}), ailerons with ground-up {n_ail}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
