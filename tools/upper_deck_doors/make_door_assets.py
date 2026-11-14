"""Upper-deck door lining: textures matched to the main-deck door lining (image-006 / image-004 /
a380x_pax_doors_material_gloss sampled on its white panels) and box-projected tiling UVs for
cabin_a380x_upper_deck_door_surrounds.obj, whose exported UVs are unusable and which had no texture."""
import sys, numpy as np
from PIL import Image
src_obj, out_dir = sys.argv[1], sys.argv[2]
N = 1024
rng = np.random.default_rng(380)
def periodic_noise(sigma_px):
    f = np.fft.fft2(rng.standard_normal((N, N)))
    ky = np.fft.fftfreq(N)[:, None]; kx = np.fft.fftfreq(N)[None, :]
    f *= np.exp(-2 * (np.pi * sigma_px) ** 2 * (kx * kx + ky * ky))
    n = np.real(np.fft.ifft2(f)); return n / n.std()
grain = 1.6 * periodic_noise(0.8) + 1.0 * periodic_noise(6) + 0.8 * periodic_noise(60)   # fine grain + soft mottling
base = np.array([237.0, 243.0, 252.0])
albedo = np.clip(base[None, None, :] + grain[..., None] * np.array([1.0, 1.0, 0.8]), 0, 255)
Image.fromarray(np.dstack([albedo, np.full((N, N), 255.0)]).astype(np.uint8), 'RGBA').save(f'{out_dir}/a380x_upper_deck_doors_albedo.png')
# Normal: flat (main-deck encoding, B 128) with a faint micro-texture from the fine grain.
fine = periodic_noise(0.8)
gx = np.roll(fine, -1, 1) - np.roll(fine, 1, 1); gy = np.roll(fine, -1, 0) - np.roll(fine, 1, 0)
normal = np.dstack([128 + 1.5 * gx, 128 + 1.5 * gy, np.full((N, N), 128.0), np.full((N, N), 255.0)])
Image.fromarray(np.clip(normal, 0, 255).astype(np.uint8), 'RGBA').save(f'{out_dir}/a380x_upper_deck_doors_normal.png')
# Material/gloss: non-metallic, the main-deck lining gloss (G 123) with slight variation.
gloss = np.dstack([np.zeros((N, N)), np.clip(123 + 4 * periodic_noise(20), 0, 255), np.zeros((N, N)), np.full((N, N), 255.0)])
Image.fromarray(gloss.astype(np.uint8), 'RGBA').save(f'{out_dir}/a380x_upper_deck_doors_material_gloss.png')

TILE = 0.6   # metres per texture repeat
header = ("TEXTURE ../../../textures/interior/cabin/a380x_upper_deck_doors_albedo.png\n"
          "TEXTURE_MAP normal ../../../textures/interior/cabin/a380x_upper_deck_doors_normal.png\n"
          "TEXTURE_MAP material_gloss ../../../textures/interior/cabin/a380x_upper_deck_doors_material_gloss.png\n")
n_vt = 0
with open(src_obj, 'r', errors='replace') as f, open(f'{out_dir}/cabin_a380x_upper_deck_door_surrounds.obj', 'w') as w:
    inserted = False
    for line in f:
        if not inserted and line.startswith('POINT_COUNTS'):
            w.write(header); inserted = True
        if line.startswith('VT'):
            p = line.split()
            x, y, z, nx, ny, nz = (float(q) for q in p[1:7])
            ax = int(np.argmax(np.abs([nx, ny, nz])))
            u, v = ((z, y), (x, z), (x, y))[ax]
            if ax == 0 and nx < 0: u = -u           # keep the grain unmirrored on opposite faces
            if ax == 2 and nz < 0: u = -u
            w.write(f"VT\t{p[1]}\t{p[2]}\t{p[3]}\t{p[4]}\t{p[5]}\t{p[6]}\t{u / TILE:.6f}\t{v / TILE:.6f}\n")
            n_vt += 1
        else:
            w.write(line)
    assert inserted
print('textures written; VT rewritten:', n_vt)
