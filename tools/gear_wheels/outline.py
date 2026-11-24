import sys, numpy as np
def parse(path):
    L=open(path,errors='replace').read().split('\n'); V=[]; I=[]
    for l in L:
        if l.startswith('VT'): V.append([float(x) for x in l.split()[1:9]])
        elif l.startswith('IDX'): I.extend(int(x) for x in l.split()[1:])
    return L,np.array(V),np.array(I)
def outline(path):
    L,V,I=parse(path); d=0; out=[]; pend=None
    for n,l in enumerate(L):
        p=l.split()
        if not p or p[0] in ('VT','IDX','IDX10') or p[0].endswith('_key'): continue
        k=p[0]
        if k=='ANIM_begin': out.append(f'{n:7d} '+'  '*d+'{'); d+=1
        elif k=='ANIM_end': d-=1; out.append(f'{n:7d} '+'  '*d+'}')
        elif k=='ANIM_trans': out.append(f'{n:7d} '+'  '*d+'T '+' '.join(p[1:4]) + ('' if p[1:4]==p[4:7] else ' -> '+' '.join(p[4:7])+' '+' '.join(p[7:])))
        elif k=='ANIM_trans_begin': out.append(f'{n:7d} '+'  '*d+'Tkeyed '+p[1])
        elif k=='ANIM_rotate_begin': out.append(f'{n:7d} '+'  '*d+'R ['+' '.join(p[1:4])+'] '+p[4])
        elif k=='ANIM_rotate': out.append(f'{n:7d} '+'  '*d+'Rstatic '+' '.join(p[1:]))
        elif k=='TRIS':
            o,c=int(p[1]),int(p[2]); ids=np.unique(I[o:o+c]); P=V[ids,:3]
            out.append(f'{n:7d} '+'  '*d+f'TRIS {o} {c}  verts {len(ids)} bbox min {np.round(P.min(0),3)} max {np.round(P.max(0),3)}')
        elif k.startswith(('ATTR','ANIM_hide','ANIM_show')): out.append(f'{n:7d} '+'  '*d+' '.join(p))
        elif k.endswith('_end'): pass
    return out
if __name__=='__main__':
    for l in outline(sys.argv[1]): print(l)
