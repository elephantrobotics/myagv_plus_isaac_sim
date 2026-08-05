import xml.etree.ElementTree as ET, math, sys

def rot(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p); cy, sy = math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ]

def mv(A, v):
    return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]

root = ET.parse(sys.argv[1]).getroot()
links = {l.get('name'): l for l in root.findall('link')}
joints = {j.get('name'): j for j in root.findall('joint')}
print(f'links={len(links)}  joints={len(joints)}')

FDIR = {'fl': (1.0, -1.0), 'fr': (1.0, 1.0), 'rl': (1.0, 1.0), 'rr': (1.0, -1.0)}
ok = True
for side in ('fl', 'fr', 'rl', 'rr'):
    angs, ys, rads = [], [], []
    print(f'--- {side} ---')
    Iyy_tot = 0.0
    hub = links[f'{side}_wheel_link'].find('inertial')
    Iyy_tot += float(hub.find('inertia').get('iyy'))
    for i in range(9):
        j = joints[f'{side}_roller_{i}_joint']
        o = j.find('origin')
        xyz = [float(v) for v in o.get('xyz').split()]
        rpy = [float(v) for v in o.get('rpy').split()]
        ax = [float(v) for v in j.find('axis').get('xyz').split()]
        R = rot(rpy)
        aw = mv(R, ax)
        rad = math.hypot(xyz[0], xyz[2])
        th = math.degrees(math.atan2(xyz[2], xyz[0])) % 360
        rhat = (xyz[0]/rad, 0.0, xyz[2]/rad)
        that = (-rhat[2], 0.0, rhat[0])
        radial = sum(aw[k]*rhat[k] for k in range(3))
        tang = sum(aw[k]*that[k] for k in range(3))
        angY = math.degrees(math.acos(max(-1, min(1, aw[1]))))
        rads.append(rad); ys.append(xyz[1]); angs.append(th)
        m = float(links[f'{side}_roller_{i}_link'].find('inertial/mass').get('value'))
        ixx = float(links[f'{side}_roller_{i}_link'].find('inertial/inertia').get('ixx'))
        izz = float(links[f'{side}_roller_{i}_link'].find('inertial/inertia').get('izz'))
        # roller own inertia about wheel Y + parallel axis
        Iown = izz*aw[1]**2 + ixx*(1-aw[1]**2)
        Iyy_tot += Iown + m*rad**2
        flag = ''
        if abs(rad-0.032) > 1e-9: flag += ' RADIUS!'
        if abs(radial) > 1e-9: flag += ' RADIAL!'
        if abs(abs(tang)-math.sqrt(0.5)) > 1e-9 or tang > 0: flag += ' TANG!'
        if abs(min(angY, 180-angY)-45.0) > 1e-9: flag += ' ANGLE!'
        if flag: ok = False
        print(f'  r{i} th={th:7.2f} R={rad*1000:8.4f}mm y={xyz[1]*1000:+8.3f}mm '
              f'axis=[{aw[0]:+.5f} {aw[1]:+.5f} {aw[2]:+.5f}] angY={angY:8.4f} '
              f'radial={radial:+.2e} tang={tang:+.7f}{flag}')
    d = sorted(a % 360 for a in angs)
    gaps = [round((d[(k+1) % 9]-d[k]) % 360, 6) for k in range(9)]
    print(f'  spacing={set(gaps)}  y_all={set(round(v,9) for v in ys)}')
    # contact-point check vs Gazebo fdir1 (roller at bottom, th=270)
    cy_sign = 1.0 if math.cos(math.radians(45)) else 1.0
    # rebuild axis at th=270 using this side's beta from roller 0 relation
    j0 = joints[f'{side}_roller_0_joint']
    beta = float(j0.find('origin').get('rpy').split()[0])
    th270 = math.radians(270.0)
    R270 = rot([beta, -th270, 0.0])
    a270 = mv(R270, [0, 0, 1])
    fx, fy = FDIR[side]
    n = math.hypot(fx, fy)
    fd = (fx/n, fy/n, 0.0)
    dotv = sum(a270[k]*fd[k] for k in range(3))
    match = abs(abs(dotv)-1.0) < 1e-6
    if not match: ok = False
    print(f'  contact axis={[round(v,6) for v in a270]} vs fdir1={[round(v,6) for v in fd]} '
          f'|dot|={abs(dotv):.6f} {"OK" if match else "MISMATCH!"}')
    print(f'  assembled wheel Iyy = {Iyy_tot:.3e}  (real-car model 6.7e-05)')

print('ALL CHECKS PASSED' if ok else 'FAILURES PRESENT')
