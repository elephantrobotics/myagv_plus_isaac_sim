#!/usr/bin/env python3
"""测量整轮 mesh 上滚子的周向起始相位。

对外缘顶点做 9 次谐波分析, 谐波幅角除以 9 即为相位。
用于确定 wheel_mecanum_real.urdf.xacro 里各轮的 phase_deg。

    python3 measure_wheel_phase.py <wheel.stl> [<wheel.stl> ...]
"""
import cmath
import math
import sys

from stl_probe import load_stl

OUTER_RADIUS_MIN = 0.034
HARMONIC = 9


def measure(path):
    tris = load_stl(path)
    acc = 0j
    tot = 0.0
    for a, b, c in tris:
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        area = 0.5 * math.sqrt(sum(x * x for x in n))
        g = [(a[i] + b[i] + c[i]) / 3.0 for i in range(3)]
        r = math.hypot(g[0], g[2])
        if r < OUTER_RADIUS_MIN:
            continue
        w = area * r
        acc += w * cmath.exp(HARMONIC * 1j * math.atan2(g[2], g[0]))
        tot += w
    phase = math.degrees(cmath.phase(acc)) / HARMONIC % (360.0 / HARMONIC)
    return phase, abs(acc) / tot if tot else 0.0


if __name__ == '__main__':
    for p in sys.argv[1:]:
        phase, strength = measure(p)
        print(p.split('/')[-1], "phase =", round(phase, 2),
              "deg   harmonic strength =", round(strength, 4))
