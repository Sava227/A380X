"""Parse an X-Plane OBJ8 gear object; evaluate every TRIS call's frame at a deploy ratio."""
import numpy as np, xp2wave3 as xw
class Obj:
    def __init__(s, path):
        s.path=path; s.L=open(path,errors='replace').read().split('\n'); V=[]; I=[]; s.vt_rows=[]
        for n,l in enumerate(s.L):
            if l.startswith('VT'): V.append([float(x) for x in l.split()[1:9]]); s.vt_rows.append(n)
            elif l.startswith('IDX'): I.extend(int(x) for x in l.split()[1:])
        s.V=np.array(V); s.I=np.array(I)
    def calls(s, ratio=1.0, values=None):
        """values: dict dataref->value for other datarefs (default 0). Returns list of dicts per TRIS."""
        values=values or {}
        def val(dr): return ratio if 'deploy_ratio' in dr else values.get(dr,0.0)
        stack=[np.eye(4)]; pend=None; out=[]; depth=0; blk=[]
        for n,l in enumerate(s.L):
            p=l.split()
            if not p: continue
            k=p[0]
            if k in ('VT','IDX','IDX10'): continue
            if k=='ANIM_begin': stack.append(stack[-1].copy()); depth+=1; blk.append(n)
            elif k=='ANIM_end': stack.pop(); depth-=1; blk.pop()
            elif k=='ANIM_trans': 
                a=np.array(p[1:4],float); b=np.array(p[4:7],float)
                if len(p)>=10 and p[9] not in ('none','no_ref') and not np.allclose(a,b):
                    v0,v1=float(p[7]),float(p[8]); t=0 if v1==v0 else np.clip((val(p[9])-v0)/(v1-v0),0,1); a=a*(1-t)+b*t
                stack[-1]=stack[-1]@xw.tr(a)
            elif k=='ANIM_trans_begin': pend=('t',None,[],p[1])
            elif k=='ANIM_rotate_begin': pend=('r',[float(x) for x in p[1:4]],[],p[4])
            elif k=='ANIM_trans_key': pend[2].append((float(p[1]),[float(x) for x in p[2:5]]))
            elif k=='ANIM_rotate_key': pend[2].append((float(p[1]),[float(p[2])]))
            elif k=='ANIM_trans_end': stack[-1]=stack[-1]@xw.tr(xw.interp(pend[2],val(pend[3])))
            elif k=='ANIM_rotate_end': stack[-1]=stack[-1]@xw.rot(pend[1],float(xw.interp(pend[2],val(pend[3]))[0]))
            elif k=='ANIM_rotate':
                ang=float(p[4]) if len(p)<8 or p[8] in ('none','no_ref') else float(p[4])
                stack[-1]=stack[-1]@xw.rot([float(x) for x in p[1:4]],ang)
            elif k=='TRIS':
                o,c=int(p[1]),int(p[2]); out.append(dict(line=n,off=o,cnt=c,M=stack[-1].copy(),depth=depth,block=blk[-1] if blk else None))
        return out
    def world(s, call, ids=None):
        if ids is None: ids=np.unique(s.I[call['off']:call['off']+call['cnt']])
        P=np.c_[s.V[ids,:3],np.ones(len(ids))]@call['M'].T
        return P[:,:3]
