"""SD DOOR page: show "SLIDE" (white, beside the door) for every armed escape slide of a closed door, instead of FBW's
small "S" below it. A black halo keeps the upper-deck labels clear of the main-deck outline they cross. Usage: patch_sd_slide.py <in sd.html> <out sd.html>"""
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding='utf-8').read()
MARK = '/*A380X-SD-SLIDES v2*/'
assert MARK not in s, 'already patched'
old1 = 'c?E&&(b="S"):b="XX"'
new1 = 'c?E&&(b="SLIDE"):b="XX"'
old2 = '{x:r==="L"?-6:23,y:38,className:`${A} F30`},b)'
new2 = ('{x:r==="L"?-6:23,y:b==="SLIDE"?21:38,className:`${A} ${b==="SLIDE"?"F22 LS1":"F30"}`,'
        'style:b==="SLIDE"?{paintOrder:"stroke",stroke:"#000",strokeWidth:"7px",strokeLinejoin:"round"}:void 0},b)')
old3 = '/*A380X-SD-SLIDES v1*/'
for o in (old1, old2, old3):
    assert s.count(o) == 1, o
s = s.replace(old1, new1).replace(old2, new2).replace(old3, old3 + MARK)
open(dst, 'w', encoding='utf-8').write(s)
print('patched', dst)
