"""Attach objects/exterior/slides/a380x_slides.obj to the aircraft: a new _obja entry with the same attachment as the
fuselage objects (a380x_fuse2.obj: psi 180, z 115.108 ft), and _obja/count + 1.
Usage: add_slides_to_acf.py <in.acf> <out.acf>"""
import re, sys
src, dst = sys.argv[1], sys.argv[2]
lines = open(src, encoding='utf-8', errors='surrogateescape').read().split('\n')
FILE = 'exterior/slides/a380x_slides.obj'
assert not any(l.endswith(' ' + FILE) for l in lines), 'already attached'
ci = next(i for i, l in enumerate(lines) if l.startswith('P _obja/count '))
count = int(lines[ci].split()[2])
fi = next(i for i, l in enumerate(lines) if l == 'P _obja/2/_v10_att_file_stl exterior/fuselage/a380x_fuse2.obj')
template = [l for l in lines if l.startswith('P _obja/2/')]
assert len(template) == 13, template
new = [l.replace('P _obja/2/', f'P _obja/{count}/', 1) for l in template]
new = [f'P _obja/{count}/_v10_att_file_stl {FILE}' if '_v10_att_file_stl' in l else l for l in new]
# keep X-Plane's lexical key order: after the last _obja/<n> line that sorts before the new index
keys = [(i, re.match(r'P _obja/(\d+)/', l)) for i, l in enumerate(lines)]
before = [i for i, m in keys if m and m.group(1) < str(count)]
at = max(before) + 1
lines[ci] = f'P _obja/count {count + 1}'
lines[at:at] = new
open(dst, 'w', encoding='utf-8', errors='surrogateescape').write('\n'.join(lines))
print(f'added _obja/{count} at line {at + 1}; _obja/count {count} -> {count + 1}')
print('\n'.join(new))
