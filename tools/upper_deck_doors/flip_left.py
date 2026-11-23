"""Turn the mirrored left upper-deck door panels (door_open_ratio 11/13/15) to face the cabin: negate the
normals of their vertices and swap two corners of each triangle. Vertices of those draw calls are not shared
with other calls (checked). in.obj out.obj"""
import sys, re
src, dst = sys.argv[1], sys.argv[2]
lines = open(src, errors='replace').read().split('\n')
vt_rows = [i for i, l in enumerate(lines) if l.startswith('VT')]
idx_rows = [i for i, l in enumerate(lines) if l.startswith('IDX')]
IDX = [int(x) for i in idx_rows for x in lines[i].split()[1:]]
flip_ranges, anim, call_no = [], [], -1
STATIC_LEFT = {2, 3, 6, 7, 10, 11}     # left side linings and hoods (mirrored copies of 0/1, 4/5, 8/9)
for l in lines:
    s = l.strip()
    if s.startswith('ANIM_begin'): anim.append([])
    elif s.startswith('ANIM_end'): anim.pop()
    elif s.startswith('ANIM_trans_begin') or s.startswith('ANIM_rotate_begin'): anim[-1].append(s.split()[-1])
    elif s.startswith('TRIS'):
        call_no += 1
        doors = re.findall(r'door_open_ratio\[(\d+)\]', ' '.join(d for lv in anim for d in lv))
        if (doors and int(doors[0]) in (11, 13, 15)) or (not doors and call_no in STATIC_LEFT):
            off, n = (int(x) for x in s.split()[1:3]); flip_ranges.append((off, n))
flip_verts, other_verts = set(), set()
inflip = [False] * len(IDX)
for off, n in flip_ranges:
    for p in range(off, off + n): inflip[p] = True
for p, v in enumerate(IDX): (flip_verts if inflip[p] else other_verts).add(v)
assert not (flip_verts & other_verts), 'shared vertices'
for off, n in flip_ranges:
    for t in range(off, off + n, 3): IDX[t + 1], IDX[t + 2] = IDX[t + 2], IDX[t + 1]
for v in flip_verts:
    f = lines[vt_rows[v]].split()
    f[4:7] = [f'{-float(x):.8g}' for x in f[4:7]]
    lines[vt_rows[v]] = '\t'.join(f)
# rewrite the IDX lines with the same layout
pos = 0
for i in idx_rows:
    k = len(lines[i].split()) - 1
    lines[i] = lines[i].split()[0] + '\t' + '\t'.join(str(x) for x in IDX[pos:pos + k]); pos += k
open(dst, 'w').write('\n'.join(lines))
print('flipped draw calls', len(flip_ranges), 'triangles', sum(n for _, n in flip_ranges) // 3, 'vertices', len(flip_verts))
