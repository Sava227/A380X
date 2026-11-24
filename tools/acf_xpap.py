"""A380X ACF: X-Plane autopilot / autothrottle settings as Laminar's A330 has them (Airbus autopilot logic, A/THR
PID gains, ALT always armed, preselected V/S loaded on capture). Usage: acf_xpap.py <in.acf> <out.acf>"""
import sys
SET = {
    'acf/_autopilot_logic': '12',              # Airbus FCU logic (A330: 12)
    'acf/_ott_atr_Kp': '9.000000000',          # autothrottle PID (A330)
    'acf/_ott_atr_Ki': '2.500000000',
    'acf/_ott_atr_Kd': '5.750000000',
    'acf/_ott_asi_thro_per_sec': '0.100000001',
    'acf/_alt_always_armed': '1',
    'acf/_load_presel_on_vvi_hit': '1',
}
src, dst = sys.argv[1], sys.argv[2]
lines = open(src, encoding='utf-8', errors='surrogateescape').read().split('\n')
done = set()
for n, l in enumerate(lines):
    if l.startswith('P '):
        k, _, v = l[2:].partition(' ')
        if k in SET:
            print(f'{k}: {v} -> {SET[k]}'); lines[n] = f'P {k} {SET[k]}'; done.add(k)
missing = set(SET) - done
assert not missing, missing
open(dst, 'w', encoding='utf-8', errors='surrogateescape').write('\n'.join(lines))
