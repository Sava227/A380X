"""Structural check of X-Plane OBJ8 files: counts, index/TRIS ranges, ANIM nesting, key tables."""
import sys
def check(path):
    nv = ni = 0; pc = None; depth = 0; maxd = 0; errs = []; tris = []; idx_max = -1; inkeys = None; nk = 0
    for n, l in enumerate(open(path, errors='replace'), 1):
        p = l.split()
        if not p: continue
        k = p[0]
        if k == 'VT': nv += 1
        elif k == 'IDX10': ni += 10; idx_max = max(idx_max, max(int(x) for x in p[1:]))
        elif k == 'IDX': ni += 1; idx_max = max(idx_max, int(p[1]))
        elif k == 'POINT_COUNTS': pc = [int(x) for x in p[1:5]]
        elif k == 'ANIM_begin': depth += 1; maxd = max(maxd, depth)
        elif k == 'ANIM_end':
            depth -= 1
            if depth < 0: errs.append(f'line {n}: ANIM_end without begin')
        elif k in ('ANIM_trans_begin', 'ANIM_rotate_begin'):
            if depth == 0: errs.append(f'line {n}: {k} outside ANIM_begin')
            inkeys = k; nk = 0
        elif k in ('ANIM_trans_key', 'ANIM_rotate_key'): nk += 1
        elif k in ('ANIM_trans_end', 'ANIM_rotate_end'):
            if nk < 2: errs.append(f'line {n}: key table with {nk} keys')
            inkeys = None
        elif k == 'TRIS': tris.append((n, int(p[1]), int(p[2])))
    if pc is None: errs.append('no POINT_COUNTS')
    else:
        if pc[0] != nv: errs.append(f'VT count {nv} != POINT_COUNTS {pc[0]}')
        if pc[3] != ni: errs.append(f'IDX count {ni} != POINT_COUNTS {pc[3]}')
    if idx_max >= nv: errs.append(f'index {idx_max} >= vertex count {nv}')
    for n, s, c in tris:
        if s < 0 or s + c > ni or c % 3: errs.append(f'line {n}: TRIS {s} {c} out of range ({ni})')
    if depth != 0: errs.append(f'ANIM depth {depth} at end')
    print(('OK  ' if not errs else 'FAIL') + f' {path.split("/")[-1]}: VT {nv} IDX {ni} TRIS {len(tris)} max ANIM depth {maxd}')
    for e in errs[:10]: print('     ', e)
    return not errs
ok = all([check(p) for p in sys.argv[1:]])
sys.exit(0 if ok else 1)
