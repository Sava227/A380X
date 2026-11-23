"""Rebuild cabin_a380x_upper_deck_door_surrounds.obj with the baked UVs (one vertex per triangle corner, so
the TRIS ranges and animations are untouched) and compose the albedo / normal / gloss textures from the AO
bake. Usage: build_baked.py src.obj uv.json ao.png out_dir"""
import sys, json, numpy as np
from PIL import Image
src, uvj, ao_png, out = sys.argv[1:5]
lines = open(src, errors='replace').read().split('\n')
VT = [l for l in lines if l.startswith('VT')]
IDX = [int(x) for l in lines if l.startswith('IDX') for x in l.split()[1:]]
calls = [(int(l.split()[1]), int(l.split()[2])) for l in lines if l.strip().startswith('TRIS')]
uv = json.load(open(uvj))
names = sorted(uv)                     # call000 .. call047, same order as the TRIS calls
assert len(names) == len(calls), (len(names), len(calls))
corner_uv = [None] * len(IDX)
for name, (off, n) in zip(names, calls):
    faces = uv[name]
    for j in range(n // 3):
        L = faces.get(str(j))
        if L is None: continue
        a, c, b = L                      # wavefront faces were written a, c, b
        corner_uv[off + 3 * j], corner_uv[off + 3 * j + 1], corner_uv[off + 3 * j + 2] = a, b, c
missing = sum(1 for x in corner_uv if x is None)
new_vt = []
for p, i in enumerate(IDX):
    f = VT[i].split()
    u, v = corner_uv[p] if corner_uv[p] is not None else (0.0, 0.0)
    new_vt.append('VT\t' + '\t'.join(f[1:7]) + f'\t{u:.6f}\t{v:.6f}')
N = len(IDX)
idx_lines = []
for s in range(0, N - N % 10, 10): idx_lines.append('IDX10\t' + '\t'.join(str(k) for k in range(s, s + 10)))
for k in range(N - N % 10, N): idx_lines.append(f'IDX\t{k}')
outl, vt_done, idx_done = [], False, False
for l in lines:
    if l.startswith('POINT_COUNTS'): outl.append(f'POINT_COUNTS\t{N}\t0\t0\t{N}'); continue
    if l.startswith('VT'):
        if not vt_done: outl.extend(new_vt); vt_done = True
        continue
    if l.startswith('IDX'):
        if not idx_done: outl.extend(idx_lines); idx_done = True
        continue
    outl.append(l)
open(f'{out}/cabin_a380x_upper_deck_door_surrounds.obj', 'w').write('\n'.join(outl))
# textures from the AO bake
ao = np.asarray(Image.open(ao_png).convert('L')).astype(float) / 255.0
H = ao.shape[0]
rng = np.random.default_rng(380)
def noise(sigma):
    f = np.fft.fft2(rng.standard_normal((H, H))); k = np.fft.fftfreq(H)
    f *= np.exp(-2 * (np.pi * sigma) ** 2 * (k[:, None] ** 2 + k[None, :] ** 2)); n = np.real(np.fft.ifft2(f)); return n / n.std()
grain = 1.4 * noise(0.8 * H / 1024) + 1.2 * noise(8 * H / 1024)
shade = 0.42 + 0.58 * np.clip(ao, 0, 1) ** 0.8          # baked contact shadows, never pitch black
base = np.array([237.0, 243.0, 252.0])
alb = np.clip(base[None, None, :] * shade[..., None] + grain[..., None], 0, 255)
Image.fromarray(np.dstack([alb, np.full(ao.shape, 255.0)]).astype(np.uint8)).save(f'{out}/a380x_upper_deck_doors_albedo.png')
nrm = np.dstack([np.full(ao.shape, 128.0), np.full(ao.shape, 128.0), np.full(ao.shape, 128.0), np.full(ao.shape, 255.0)])
Image.fromarray(nrm.astype(np.uint8)).save(f'{out}/a380x_upper_deck_doors_normal.png')
gloss = np.dstack([np.zeros(ao.shape), np.clip(70 + 60 * ao, 0, 255), np.zeros(ao.shape), np.full(ao.shape, 255.0)])
Image.fromarray(gloss.astype(np.uint8)).save(f'{out}/a380x_upper_deck_doors_material_gloss.png')
print('vertices', N, 'corners without baked UV', missing)
