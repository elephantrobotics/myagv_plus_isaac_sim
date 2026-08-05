#!/usr/bin/env python3
"""从 SW2URDF 导出包反解 roller 在轮坐标系下的位姿。

导出器只为配置了 Reference Coordinate System 的 joint 生成正确 origin,
其余 joint 退回零件自身原点。本脚本改用各 roller link 的 <inertial><origin>
经 joint 的 rpy 变换到轮坐标系, 得到真实质心位置与转轴方向。

    python3 analyze_roller.py <导出包目录> [标签]
"""
import glob
import math
import os
import sys
import xml.etree.ElementTree as ET


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def rot(rpy):
    R_, P_, Y_ = rpy
    cr, sr = math.cos(R_), math.sin(R_)
    cp, sp = math.cos(P_), math.sin(P_)
    cy, sy = math.cos(Y_), math.sin(Y_)
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    return matmul(matmul(Rz, Ry), Rx)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def solve(pkgdir):
    """返回 (rollers, wheel_joint)。rollers 为每个 roller 的反解结果字典。"""
    f = glob.glob(os.path.join(pkgdir, 'urdf', '*.urdf'))
    if not f:
        raise SystemExit("目录下没有 urdf/*.urdf: " + pkgdir)
    root = ET.parse(f[0]).getroot()
    links = {l.get('name'): l for l in root.findall('link')}

    rollers = []
    wheel_joint = None
    for j in root.findall('joint'):
        name = j.get('name')
        o = j.find('origin')
        if name == 'wheeljoint':
            wheel_joint = {'xyz': o.get('xyz'), 'rpy': o.get('rpy'),
                           'axis': j.find('axis').get('xyz')}
            continue
        xyz = [float(v) for v in o.get('xyz').split()]
        rpy = [float(v) for v in o.get('rpy').split()]
        R = rot(rpy)
        child = j.find('child').get('link')
        inert = links[child].find('inertial')
        if inert is None:
            continue
        com_local = [float(v) for v in inert.find('origin').get('xyz').split()]
        cw = matvec(R, com_local)
        com = [xyz[i] + cw[i] for i in range(3)]
        rad = math.hypot(com[0], com[2])
        entry = {
            'name': name, 'type': j.get('type'), 'com': com,
            'radius': rad, 'angle': math.degrees(math.atan2(com[2], com[0])) % 360,
            'plane_y': com[1],
            'mass': float(inert.find('mass').get('value')),
            'inertia': {k: float(inert.find('inertia').get(k))
                        for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')},
            'axis': None, 'angle_to_y': None, 'radial': None, 'tangential': None,
        }
        ax = j.find('axis')
        if ax is not None:
            axv = [float(v) for v in ax.get('xyz').split()]
            if norm(axv) > 1e-9 and rad > 1e-9:
                axw = matvec(R, axv)
                rd = [com[0] / rad, 0.0, com[2] / rad]
                tg = [-rd[2], 0.0, rd[0]]
                entry['axis'] = axw
                entry['angle_to_y'] = math.degrees(math.acos(max(-1, min(1, axw[1]))))
                entry['radial'] = dot(axw, rd)
                entry['tangential'] = dot(axw, tg)
        rollers.append(entry)
    rollers.sort(key=lambda e: e['name'])
    return rollers, wheel_joint


def report(pkgdir, label=""):
    rollers, wj = solve(pkgdir)
    print("=" * 96)
    print(label or pkgdir)
    print("=" * 96)
    if wj:
        print("wheeljoint xyz=%s rpy=%s axis=%s" % (wj['xyz'], wj['rpy'], wj['axis']))
    print("")
    for e in rollers:
        line = ("%-10s %-11s R=%8.4fmm  ang=%8.4fdeg  y=%+9.5f  mass=%.5f"
                % (e['name'], e['type'], e['radius'] * 1000, e['angle'], e['plane_y'], e['mass']))
        print(line)
        if e['axis']:
            print("           axis=[%+.6f %+.6f %+.6f] angle_to_Y=%.4f radial=%+.2e tang=%+.6f"
                  % (e['axis'][0], e['axis'][1], e['axis'][2],
                     e['angle_to_y'], e['radial'], e['tangential']))
    return rollers, wj


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    report(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "")
