import numpy as np
def read_vt(path):
    rows = []
    with open(path, encoding='latin1') as f:
        for line in f:
            s = line.lstrip()
            if s.startswith('VT'): rows.append(s.split()[1:9])
    return np.array(rows, float)
