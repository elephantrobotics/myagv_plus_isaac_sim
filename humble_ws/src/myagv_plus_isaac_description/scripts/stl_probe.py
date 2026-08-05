import struct, sys, math

def load_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:5] == b'solid' and b'facet' in data[:2000]:
        tris = []
        vals = []
        for line in data.decode('utf-8', 'ignore').splitlines():
            s = line.strip()
            if s.startswith('vertex'):
                vals.append(tuple(float(x) for x in s.split()[1:4]))
                if len(vals) == 3:
                    tris.append(tuple(vals))
                    vals = []
        return tris
    n = struct.unpack('<I', data[80:84])[0]
    tris = []
    off = 84
    for _ in range(n):
        f = struct.unpack('<12fH', data[off:off+50])
        tris.append(((f[3], f[4], f[5]), (f[6], f[7], f[8]), (f[9], f[10], f[11])))
        off += 50
    return tris

def volume_props(tris):
    vol = 0.0
    cx = cy = cz = 0.0
    for a, b, c in tris:
        v = (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6.0
        vol += v
        cx += v * (a[0]+b[0]+c[0]) / 4.0
        cy += v * (a[1]+b[1]+c[1]) / 4.0
        cz += v * (a[2]+b[2]+c[2]) / 4.0
    return vol, (cx/vol, cy/vol, cz/vol)

def covariance_about(tris, ctr):
    # area-weighted vertex covariance -> principal axes of the surface
    S = [[0.0]*3 for _ in range(3)]
    tot = 0.0
    for tri in tris:
        ax, ay, az = tri[0]; bx, by, bz = tri[1]; cx_, cy_, cz_ = tri[2]
        ux, uy, uz = bx-ax, by-ay, bz-az
        vx, vy, vz = cx_-ax, cy_-ay, cz_-az
        nx, ny, nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
        area = 0.5*math.sqrt(nx*nx+ny*ny+nz*nz)
        if area <= 0:
            continue
        m = ((ax+bx+cx_)/3.0-ctr[0], (ay+by+cy_)/3.0-ctr[1], (az+bz+cz_)/3.0-ctr[2])
        for i in range(3):
            for j in range(3):
                S[i][j] += area*m[i]*m[j]
        tot += area
    return [[S[i][j]/tot for j in range(3)] for i in range(3)], tot

def jacobi(A):
    import copy
    a = copy.deepcopy(A)
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(100):
        off = sum(a[i][j]**2 for i in range(3) for j in range(3) if i != j)
        if off < 1e-24:
            break
        for p in range(2):
            for q in range(p+1, 3):
                if abs(a[p][q]) < 1e-30:
                    continue
                theta = (a[q][q]-a[p][p])/(2*a[p][q])
                t = (1 if theta >= 0 else -1)/(abs(theta)+math.sqrt(theta*theta+1))
                c = 1/math.sqrt(t*t+1); s = t*c
                for k in range(3):
                    akp = a[k][p]; akq = a[k][q]
                    a[k][p] = c*akp - s*akq
                    a[k][q] = s*akp + c*akq
                for k in range(3):
                    apk = a[p][k]; aqk = a[q][k]
                    a[p][k] = c*apk - s*aqk
                    a[q][k] = s*apk + c*aqk
                for k in range(3):
                    vkp = v[k][p]; vkq = v[k][q]
                    v[k][p] = c*vkp - s*vkq
                    v[k][q] = s*vkp + c*vkq
    eig = [a[i][i] for i in range(3)]
    vecs = [[v[0][i], v[1][i], v[2][i]] for i in range(3)]
    order = sorted(range(3), key=lambda i: eig[i])
    return [eig[i] for i in order], [vecs[i] for i in order]

def main():
  path = sys.argv[1]
  tris = load_stl(path)
  xs = [p[0] for t in tris for p in t]
  ys = [p[1] for t in tris for p in t]
  zs = [p[2] for t in tris for p in t]
  print(f'file      : {path}')
  print(f'triangles : {len(tris)}')
  print(f'bbox min  : [{min(xs):.5f} {min(ys):.5f} {min(zs):.5f}]')
  print(f'bbox max  : [{max(xs):.5f} {max(ys):.5f} {max(zs):.5f}]')
  print(f'bbox size : [{max(xs)-min(xs):.5f} {max(ys)-min(ys):.5f} {max(zs)-min(zs):.5f}]')
  vol, com = volume_props(tris)
  print(f'volume    : {vol:.6f}   (mm^3 if mm)')
  print(f'CoM       : [{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}]')
  cov, area = covariance_about(tris, com)
  eig, vecs = jacobi(cov)
  print(f'area      : {area:.4f}')
  print('principal axes about CoM (ascending spread):')
  for e, v in zip(eig, vecs):
    print(f'   lambda={e:12.6f}  axis=[{v[0]:+.6f} {v[1]:+.6f} {v[2]:+.6f}]')

  # radius profile about the smallest-spread axis (the symmetry axis of a barrel is the LARGEST spread for a long part)
  for label, axis in (('minSpread', vecs[0]), ('midSpread', vecs[1]), ('maxSpread', vecs[2])):
    maxr = 0.0; lo = 1e9; hi = -1e9
    for t in tris:
        for p in t:
            d = [p[i]-com[i] for i in range(3)]
            along = sum(d[i]*axis[i] for i in range(3))
            perp2 = sum(d[i]*d[i] for i in range(3)) - along*along
            maxr = max(maxr, math.sqrt(max(0.0, perp2)))
            lo = min(lo, along); hi = max(hi, along)
    print(f'  about {label:9s}: max_radius={maxr:8.4f}  span=[{lo:8.4f},{hi:8.4f}] len={hi-lo:8.4f}')


if __name__ == '__main__':
    main()
