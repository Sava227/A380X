"""Node tree of an X-Plane gear OBJ: per ANIM block (in file order) its parent, frame after its own transforms
(at a deploy ratio) and the draw calls directly inside it."""
import numpy as np, xp2wave3 as xw
from gearobj import Obj
def tree(o, ratio=1.0, values=None):
    values = values or {}
    val = lambda dr: ratio if 'deploy_ratio' in dr else values.get(dr, 0.0)
    nodes = []; stack = [np.eye(4)]; open_ = []; pend = None
    for n, l in enumerate(o.L):
        p = l.split()
        if not p or p[0] in ('VT', 'IDX', 'IDX10'): continue
        k = p[0]
        if k == 'ANIM_begin':
            nodes.append(dict(i=len(nodes), line=n, parent=open_[-1] if open_ else None, depth=len(open_), calls=[], T=None, own_end=n))
            open_.append(len(nodes) - 1); stack.append(stack[-1].copy())
        elif k == 'ANIM_end': open_.pop(); stack.pop()
        elif k == 'ANIM_trans':
            a = np.array(p[1:4], float); stack[-1] = stack[-1] @ xw.tr(a)
            if open_ and nodes[open_[-1]]['T'] is None: nodes[open_[-1]]['T'] = a
            if open_: nodes[open_[-1]]['own_end'] = n
        elif k == 'ANIM_trans_begin': pend = ('t', None, [], p[1])
        elif k == 'ANIM_rotate_begin': pend = ('r', [float(x) for x in p[1:4]], [], p[4])
        elif k == 'ANIM_trans_key': pend[2].append((float(p[1]), [float(x) for x in p[2:5]]))
        elif k == 'ANIM_rotate_key': pend[2].append((float(p[1]), [float(p[2])]))
        elif k in ('ANIM_trans_end', 'ANIM_rotate_end'):
            stack[-1] = stack[-1] @ (xw.tr(xw.interp(pend[2], val(pend[3]))) if k == 'ANIM_trans_end'
                                     else xw.rot(pend[1], float(xw.interp(pend[2], val(pend[3]))[0])))
            if open_: nodes[open_[-1]]['own_end'] = n
        elif k == 'TRIS':
            if open_: nodes[open_[-1]]['calls'].append((int(p[1]), int(p[2]), n))
        if open_ and k in ('ANIM_trans_end', 'ANIM_rotate_end', 'ANIM_trans'): nodes[open_[-1]]['M'] = stack[-1].copy()
    for nd in nodes:
        if 'M' not in nd: nd['M'] = None
    return nodes
def subtree(nodes, i):
    out = {i}; ch = True
    while ch:
        ch = False
        for nd in nodes:
            if nd['parent'] in out and nd['i'] not in out: out.add(nd['i']); ch = True
    return out
