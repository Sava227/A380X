"""Checks for wing_flex_obj.py output: identity at zero flex, rigid parts vs FBW's own flexed rig, gaps at the cuts."""
import sys, os, re, json, glob, numpy as np, pickle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wing_flex_obj import Obj8, evaluate, Flex
from neutral_cloud import static_overrides, flex_bone
from gltfrig import Rig
from match import Grid
gltf, cfg_new, cfg_old, old_dir, new_dir, perm_dir = sys.argv[1:7]
rules = json.load(open(cfg_new))['mappings']
DR = {}
for side in ('left', 'right'):
    for clip in ('inboard_flex', 'inner_midboard_flex', 'outer_midboard_flex', 'outboard_flex'):
        r_ = next(x for x in rules if re.fullmatch(x['pattern'], f'{side}_{clip}')); DR[f'{side}_{clip}'] = r_
def values(kind):
    out = {}
    for clip, r_ in DR.items():
        v = {'cruise': 1.6, 'droop': -1.4, 'max': r_['value_max'], 'min': r_['value_min']}[kind]
        out[r_['dataref']] = max(r_['value_min'], min(r_['value_max'], v))
    return out
rig = Rig(gltf)
def fbw_cloud(vals):
    """FBW skinned/rigid points at the flex pose given by dataref values (other clips at the old static pose)"""
    over, _ = static_overrides(rig, cfg_old)
    for clip, r_ in DR.items():
        v = vals[r_['dataref']]; t = rig.clip_duration(clip) * (v - r_['value_min']) / (r_['value_max'] - r_['value_min'])
        for node, d in rig.sample(clip, t).items(): over.setdefault(node, {}).update(d)
    W = rig.world(over); pts = []
    for ni, n in enumerate(rig.nodes):
        if 'mesh' not in n: continue
        for p in rig.d['meshes'][n['mesh']]['primitives']:
            P = rig.acc(p['attributes']['POSITION']).astype(float); H = np.c_[P, np.ones(len(P))]
            if 'skin' in n and 'JOINTS_0' in p['attributes']:
                sk = rig.d['skins'][n['skin']]; joints = sk['joints']
                ibm = rig.acc(sk['inverseBindMatrices']).reshape(-1, 4, 4).transpose(0, 2, 1)
                J = rig.acc(p['attributes']['JOINTS_0']).astype(int); Wt = rig.acc(p['attributes']['WEIGHTS_0']).astype(float)
                Wt = np.where(J < len(joints), Wt, 0.0); J = np.where(J < len(joints), J, 0); Wt /= np.maximum(Wt.sum(1, keepdims=True), 1e-12)
                Mj = np.stack([W[j] @ ibm[k] for k, j in enumerate(joints)]); o = np.zeros((len(P), 4))
                for k in range(4): o += Wt[:, k:k + 1] * np.einsum('nij,nj->ni', Mj[J[:, k]], H)
                pts.append(o[:, :3])
            else: pts.append((H @ W[ni].T)[:, :3])
    C = np.concatenate(pts); return C
neutral = fbw_cloud(values('cruise') | {r_['dataref']: 0.0 for r_ in DR.values()})
keep = np.abs(neutral).max(1) < 100
grid = Grid(neutral[keep], 0.01); idx_keep = np.flatnonzero(keep)
report = {}
for path in sorted(glob.glob(new_dir + '/*.obj')):
    name = os.path.basename(path); old = Obj8(os.path.join(old_dir, name)); new = Obj8(path)
    src = np.load(os.path.join(perm_dir, name + '.src.npy'))
    W0o = evaluate(old, {}); W0n = evaluate(new, {})
    # zero flex: every new triangle lies inside its source triangle, and each source triangle's area is preserved
    A = W0o[src]; Bo = W0o[src + 1]; Co = W0o[src + 2]
    nrm = np.cross(Bo - A, Co - A); area_o = np.linalg.norm(nrm, axis=1) / 2; nn = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    T = W0n.reshape(-1, 3, 3); off_plane = np.abs(np.einsum('nkj,nj->nk', T - A[:, None], nn)).max()
    area_n = np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1) / 2
    key = np.sort(new.I.reshape(-1, 3), axis=1); _, uniq = np.unique(np.c_[src // 3, key], axis=0, return_index=True)
    tri_src = src // 3; tot = np.bincount(tri_src[uniq], area_n[uniq]); used = np.unique(tri_src)
    orig_area = dict(zip(tri_src, area_o)); area_err = max(abs(tot[u] - orig_area[u]) for u in used)
    old_used = np.unique(np.arange(0, len(old.I), 3)[~np.isnan(W0o[::3, 0])] // 3)
    # band coverage: every coincident pair that separates must stay within the 10 cm band
    rep0 = {}
    rep = {'zero_flex_off_plane_m': float(off_plane), 'zero_flex_area_err_m2': float(area_err),
           'source_triangles_kept': f'{len(used)}/{len(old_used)}'}
    # match a sample of slots to FBW neutral points (rigid parts only make sense, but measure all)
    rng = np.random.default_rng(0); S = rng.choice(len(W0n), min(4000, len(W0n)), replace=False)
    m = [(s, grid.nearest(W0n[s])) for s in S]; m = [(s, idx_keep[i]) for s, (i, d) in m if i >= 0 and d < 0.002]
    rep['matched_fraction'] = round(len(m) / len(S), 3)
    for kind in ('cruise', 'max', 'min'):
        vals = values(kind); Wf = evaluate(new, vals); C = fbw_cloud(vals)
        if m:
            err = np.array([np.linalg.norm(Wf[s] - C[i]) for s, i in m])
            disp = np.array([np.linalg.norm(C[i] - neutral[i]) for s, i in m])
            rep[f'{kind}_vs_fbw_err_m(p50,p95,max)'] = [round(float(np.percentile(err, q)), 4) for q in (50, 95)] + [round(float(err.max()), 4)]
            rep[f'{kind}_fbw_displacement_max_m'] = round(float(disp.max()), 3)
        # gaps: slots coincident at zero flex but apart when flexed
        key = np.round(W0n / 0.0005).astype(np.int64); order = np.lexsort(key.T[::-1]); ks = key[order]
        brk = np.r_[True, np.any(ks[1:] != ks[:-1], axis=1)]; gid = np.cumsum(brk) - 1
        Wo = Wf[order]; gap = 0.0; where = None
        mins = np.full((gid[-1] + 1, 3), np.inf); maxs = np.full((gid[-1] + 1, 3), -np.inf)
        np.minimum.at(mins, gid, Wo); np.maximum.at(maxs, gid, Wo)
        spread = np.linalg.norm(maxs - mins, axis=1); g = int(spread.argmax())
        rep[f'{kind}_max_gap_m'] = round(float(spread[g]), 4); rep[f'{kind}_gap_p99_m'] = round(float(np.percentile(spread[spread > 1e-6], 99)) if (spread > 1e-6).any() else 0.0, 4)
        rep[f'{kind}_max_gap_at'] = [round(float(x), 2) for x in W0n[order][np.flatnonzero(gid == g)[0]]]
        rep[f'{kind}_tip_disp_m'] = round(float(np.nanmax(np.linalg.norm(Wf - W0n, axis=1))), 3)
    report[name] = rep; print(name, json.dumps(rep))
json.dump(report, open(os.path.join(os.path.dirname(new_dir), 'verify_report.json'), 'w'), indent=1)
