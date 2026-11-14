"""Final upper-deck door object: baked UVs (one vertex per triangle corner; TRIS ranges and animations
untouched), door-edge seal strips inside each door's animation block, and the albedo / normal / gloss
textures from the AO bake. Usage: build_final.py fixed_src.obj uv.json ao.png out_dir"""
import sys, json, re, numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
import seal
src, uvj, ao_png, out = sys.argv[1:5]
lines = open(src, errors='replace').read().split('\n')
VT = [[float(x) for x in l.split()[1:9]] for l in lines if l.startswith('VT')]
IDX = [int(x) for l in lines if l.startswith('IDX') for x in l.split()[1:]]
tris_rows, calls, anim = [], [], []
for i, l in enumerate(lines):
    s = l.strip()
    if s.startswith('ANIM_begin'): anim.append([])
    elif s.startswith('ANIM_end'): anim.pop()
    elif s.startswith('ANIM_trans_begin'): anim[-1].append(s.split()[-1])
    elif s.startswith('TRIS'):
        off, n = (int(x) for x in s.split()[1:3])
        door = re.findall(r'door_open_ratio\[(\d+)\]', ' '.join(d for lv in anim for d in lv))
        calls.append((off, n, int(door[0]) if door else None)); tris_rows.append(i)
uv = json.load(open(uvj)); names = sorted(uv); assert len(names) == len(calls)
corner = [None] * len(IDX)
for name, (off, n, _) in zip(names, calls):
    for j in range(n // 3):
        L = uv[name].get(str(j))
        if L: a, c, b = L; corner[off + 3*j], corner[off + 3*j + 1], corner[off + 3*j + 2] = a, b, c
ao = np.asarray(Image.open(ao_png).convert('L')).astype(float) / 255.0; H = ao.shape[0]
# free texture block for the seal colour: rasterize all UV triangles (with margin), find an empty 24 px block
mask = Image.new('L', (H, H), 0); dr = ImageDraw.Draw(mask)
for p in range(0, len(IDX), 3):
    if corner[p] is None: continue
    dr.polygon([(corner[p + k][0] * H, (1 - corner[p + k][1]) * H) for k in range(3)], fill=255, outline=255)
m = np.asarray(mask) > 0
B = 24; spot = None
cs = np.pad(m, 4).cumsum(0).cumsum(1)
for y in range(H - B - 8, 8, -4):
    for x in range(H - B - 8, 8, -4):
        y0, x0 = y + 4, x + 4
        s = cs[y0 + B + 4, x0 + B + 4] - cs[y0 - 4, x0 + B + 4] - cs[y0 + B + 4, x0 - 4] + cs[y0 - 4, x0 - 4]
        if s == 0: spot = (x, y); break
    if spot: break
assert spot, 'no free texture block'
seal_uv = ((spot[0] + B / 2) / H, 1 - (spot[1] + B / 2) / H)
# seal strips per door panel (the largest draw call of each door), in that door's local frame
new_vt, new_idx = [], []
for p, i in enumerate(IDX):
    u, v = corner[p] if corner[p] is not None else (0.0, 0.0)
    new_vt.append(VT[i][:6] + [u, v]); new_idx.append(p)
inserts = {}
by_door = {}
for k, (off, n, door) in enumerate(calls):
    if door is not None and (door not in by_door or n > calls[by_door[door]][1]): by_door[door] = k
for door, k in sorted(by_door.items()):
    off, n, _ = calls[k]
    t = np.array(IDX[off:off + n]).reshape(-1, 3)
    V = np.array(VT)
    st, n_avg, nloop = seal.seal(V[t][:, :, :3], V[t][:, :, 3:6], width=0.03, back=0.005)
    start = len(new_idx)
    for tri in st:
        for q in tri:
            new_idx.append(len(new_vt)); new_vt.append(list(q) + list(n_avg) + list(seal_uv))
    inserts[tris_rows[k]] = f'\t\tATTR_no_cull\n\t\tTRIS\t{start} {len(st) * 3}'
N, M = len(new_vt), len(new_idx)
idx_lines = [('IDX10\t' + '\t'.join(str(x) for x in new_idx[s:s + 10])) for s in range(0, M - M % 10, 10)]
idx_lines += [f'IDX\t{x}' for x in new_idx[M - M % 10:]]
outl, vt_done, idx_done = [], False, False
for i, l in enumerate(lines):
    if l.startswith('POINT_COUNTS'): outl.append(f'POINT_COUNTS\t{N}\t0\t0\t{M}'); continue
    if l.startswith('VT'):
        if not vt_done:
            outl.extend('VT\t' + '\t'.join(f'{x:.6f}' for x in v) for v in new_vt); vt_done = True
        continue
    if l.startswith('IDX'):
        if not idx_done: outl.extend(idx_lines); idx_done = True
        continue
    outl.append(l)
    if i in inserts: outl.append(inserts[i])
open(f'{out}/cabin_a380x_upper_deck_door_surrounds.obj', 'w').write('\n'.join(outl))
# textures
rng = np.random.default_rng(380)
def noise(sigma):
    f = np.fft.fft2(rng.standard_normal((H, H))); k = np.fft.fftfreq(H)
    f *= np.exp(-2 * (np.pi * sigma) ** 2 * (k[:, None] ** 2 + k[None, :] ** 2)); r = np.real(np.fft.ifft2(f)); return r / r.std()
grain = 1.4 * noise(0.8 * H / 1024) + 1.2 * noise(8 * H / 1024)
shade = 0.42 + 0.58 * np.clip(ao, 0, 1) ** 0.8
alb = np.clip(np.array([237.0, 243.0, 252.0])[None, None, :] * shade[..., None] + grain[..., None], 0, 255)
gl = np.clip(70 + 60 * ao, 0, 255)
alb[spot[1]:spot[1] + B, spot[0]:spot[0] + B] = [52, 54, 58]; gl[spot[1]:spot[1] + B, spot[0]:spot[0] + B] = 40
one = np.full(ao.shape, 255.0)
Image.fromarray(np.dstack([alb, one]).astype(np.uint8)).save(f'{out}/a380x_upper_deck_doors_albedo.png')
Image.fromarray(np.dstack([np.full(ao.shape, 128.0)] * 3 + [one]).astype(np.uint8)).save(f'{out}/a380x_upper_deck_doors_normal.png')
Image.fromarray(np.dstack([np.zeros(ao.shape), gl, np.zeros(ao.shape), one]).astype(np.uint8)).save(f'{out}/a380x_upper_deck_doors_material_gloss.png')
print('vertices', N, 'indices', M, 'seal strips', len(inserts), 'seal texel block', spot, 'corners without UV', sum(c is None for c in corner))
