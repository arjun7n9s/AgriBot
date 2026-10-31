import sys, struct, numpy as np, pathlib
def load(p):
    b = pathlib.Path(p).read_bytes()
    if b[:5].lower()==b'solid' and b'facet' in b[:400]:
        v=[list(map(float,l.split()[1:4])) for l in b.decode(errors='ignore').splitlines() if l.strip().startswith('vertex')]
        return np.array(v)
    n=struct.unpack('<I',b[80:84])[0]
    d=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return d['v'].reshape(-1,3).astype(float)
for p in sys.argv[1:]:
    v=load(p); mn,mx=v.min(0),v.max(0)
    print(f"{pathlib.Path(p).stem:32s} size {np.round(mx-mn,1)}  min {np.round(mn,1)}")
