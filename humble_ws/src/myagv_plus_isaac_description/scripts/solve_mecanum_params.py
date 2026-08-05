#!/usr/bin/env python3
"""麦轮参数一键反解:从结构交付物直接算出 xacro 需要的全部数值。

输出各参数的反解值、一致性检查、以及可直接粘贴的 xacro 属性块。
纯 Python, 无需 ROS 或 Isaac 环境。

本项目 myAGV Plus 的执行命令:

    cd /home/elephant/humble_ws/src/myagv_plus_description/scripts
    python3 solve_mecanum_params.py \
      --export /home/elephant/workpace/agv_plus/80mm麦克纳姆轮左.SLDASM/80mm麦克纳姆轮左.SLDASM \
      --roller-stl /home/elephant/workpace/agv_plus/80mm麦克纳姆轮左.SLDASM/80mm麦克纳姆轮左.SLDASM/meshes/r1_Link.STL \
      --wheel-mesh fl=/home/elephant/humble_ws/src/myagv_plus_description/meshes/wheel_fl_link.stl \
                   fr=/home/elephant/humble_ws/src/myagv_plus_description/meshes/wheel_fr_link.stl \
                   rl=/home/elephant/humble_ws/src/myagv_plus_description/meshes/wheel_rl_link.stl \
                   rr=/home/elephant/humble_ws/src/myagv_plus_description/meshes/wheel_rr_link.stl \
      --verify /home/elephant/humble_ws/src/myagv_plus_description/urdf/myagv_plus_isaac_rollers.urdf
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from analyze_roller import solve as solve_export
from stl_probe import load_stl, volume_props, covariance_about, jacobi
from measure_wheel_phase import measure as measure_phase

TOL_RADIUS_MM = 0.05
TOL_ANGLE_DEG = 0.05


def check(label, ok, detail):
    print("[%s] %-28s %s" % ("PASS" if ok else "FAIL", label, detail))
    return ok


def solve_roller_geometry(export_dir):
    rollers, _ = solve_export(export_dir)
    radii = [e['radius'] for e in rollers]
    ys = [e['plane_y'] for e in rollers]
    angles = sorted(e['angle'] for e in rollers)
    gaps = [round((angles[(i + 1) % len(angles)] - angles[i]) % 360, 4)
            for i in range(len(angles))]
    axed = [e for e in rollers if e['axis']]

    print("--- roller 位姿反解 (%d 个) ---" % len(rollers))
    ok = True
    rmin, rmax = min(radii), max(radii)
    ok &= check("ring_radius", (rmax - rmin) * 1000 < TOL_RADIUS_MM,
                "%.4f mm  (极差 %.4f mm)" % (rmax * 1000, (rmax - rmin) * 1000))
    ymin, ymax = min(ys), max(ys)
    ok &= check("plane_y", abs(ymax - ymin) * 1000 < TOL_RADIUS_MM,
                "%+.6f m" % ymax)
    ok &= check("周向间隔", len(set(gaps)) == 1,
                "%s deg" % sorted(set(gaps)))
    if axed:
        a2y = [e['angle_to_y'] for e in axed]
        rad = [abs(e['radial']) for e in axed]
        ok &= check("轴与轮轴夹角", all(abs(min(v, 180 - v) - 45) < TOL_ANGLE_DEG for v in a2y),
                    "%s deg" % [round(v, 4) for v in a2y])
        ok &= check("轴径向分量", max(rad) < 1e-4, "max %.2e" % max(rad))
        sign = "+Y" if axed[0]['axis'][1] > 0 else "-Y"
        print("     手性: 轴的 Y 分量为 %s -> beta = %s3*pi/4"
              % (sign, "-" if sign == "+Y" else "+"))
    else:
        print("     WARNING: 没有任何 roller joint 配了 axis, 无法反解手性")

    masses = [e['mass'] for e in rollers]
    inert = axed[0]['inertia'] if axed else rollers[0]['inertia']
    vals = sorted([inert['ixx'] - abs(inert['ixz']), inert['ixx'] + abs(inert['ixz']), inert['iyy']])
    print("     roller mass = %.5f kg" % masses[0])
    print("     roller inertia 对角化 -> 轴向 %.4e, 横向 %.4e" % (vals[0], (vals[1] + vals[2]) / 2))
    return {
        'ring_radius': round(rmax, 6),
        'plane_y_abs': round(abs(ymax), 6),
        'count': len(rollers),
        'spacing': gaps[0] if gaps else None,
        'mass': masses[0],
        'i_axial': vals[0],
        'i_trans': (vals[1] + vals[2]) / 2,
        'ok': ok,
    }


def solve_roller_radius(stl_path):
    tris = load_stl(stl_path)
    _, com = volume_props(tris)
    cov, _ = covariance_about(tris, com)
    _, vecs = jacobi(cov)
    axis = vecs[2]
    maxr = 0.0
    for t in tris:
        for p in t:
            d = [p[i] - com[i] for i in range(3)]
            along = sum(d[i] * axis[i] for i in range(3))
            perp2 = sum(x * x for x in d) - along * along
            maxr = max(maxr, math.sqrt(max(0.0, perp2)))
    unit_mm = maxr > 1.0
    r = maxr / 1000.0 if unit_mm else maxr
    print("")
    print("--- roller_radius 反解 ---")
    print("     mesh 冠部最大半径 %.4f %s -> roller_radius = %.6f m"
          % (maxr, "mm" if unit_mm else "m", r))
    return round(r, 6)


def solve_phases(pairs):
    print("")
    print("--- phase_deg 反解 ---")
    out = {}
    for side, path in pairs:
        ph, strength = measure_phase(path)
        out[side] = round(ph, 2)
        flag = "" if strength > 0.15 else "   WARNING: 谐波强度低, 结果可疑"
        print("     %-3s phase = %6.2f deg   谐波强度 %.4f%s" % (side, ph, strength, flag))
    return out


def emit_xacro(geo, roller_radius, phases):
    print("")
    print("=" * 96)
    print("xacro 属性块 (粘贴进 wheel_mecanum_real.urdf.xacro)")
    print("=" * 96)
    print('    <xacro:property name="ring_radius" value="%s" />' % geo['ring_radius'])
    print('    <xacro:property name="roller_radius" value="%s" />' % roller_radius)
    print('    <xacro:property name="half_hub_width" value="%s" />' % geo['plane_y_abs'])
    print("")
    for side in ("fl", "fr", "rl", "rr"):
        if side in phases:
            print('    <!-- %s -->  <xacro:property name="phase_deg" value="${%s}" />'
                  % (side, phases[side]))
    print("")
    print('    滚子 <mass value="%.5f" />' % geo['mass'])
    print('    滚子 <inertia ixx="%.4e" iyy="%.4e" izz="%.4e" />'
          % (geo['i_trans'], geo['i_trans'], geo['i_axial']))
    print("")
    print("    有效滚动半径 = ring_radius + roller_radius = %.6f m"
          % (geo['ring_radius'] + roller_radius))
    print("")
    print("beta 手性需人工确认: fl 与 rr 一组, fr 与 rl 一组, 两组符号相反。")
    print("依据真车 xacro 的 Gazebo <fdir1>, 或用 verify_rollers.py 的 contact axis 对照。")



# ---------------------------------------------------------------- xacro 生成

SIDES = ("fl", "fr", "rl", "rr")


def parse_realcar_xacro(path):
    """从真车 wheel.urdf.xacro 读整车级参数与各轮的 fdir(用于定手性)。"""
    import re
    t = open(path).read()

    def prop(name):
        m = re.search(r'name="%s"\s+value="([^"]+)"' % name, t)
        return m.group(1) if m else None

    out = {
        'wheel_separation_x': prop('wheel_separation_x'),
        'wheel_separation_y': prop('wheel_separation_y'),
        'sides': {},
    }
    m = re.search(r'<origin xyz="\$\{x\} \$\{y\} ([0-9.eE+-]+)"', t)
    out['joint_z'] = m.group(1) if m else '0.019524'
    m = re.search(r'<limit effort="([0-9.eE+-]+)" velocity="([0-9.eE+-]+)"', t)
    out['effort'], out['velocity'] = (m.group(1), m.group(2)) if m else ('1.35', '40.0')
    m = re.search(r'<dynamics damping="([0-9.eE+-]+)" friction="([0-9.eE+-]+)"', t)
    out['damping'], out['friction'] = (m.group(1), m.group(2)) if m else ('0.001', '0.001')

    for side in SIDES:
        blk = re.search(r"side == '%s'.*?</xacro:if>" % side, t, re.S)
        if not blk:
            continue
        b = blk.group(0)
        mesh = re.search(r'name="mesh"\s+value="([^"]+)"', b)
        fdir = re.search(r'name="fdir"\s+value="([^"]+)"', b)
        ysgn = -1.0 if '${-wheel_separation_y' in b else 1.0
        xsgn = -1.0 if '${-wheel_separation_x' in b else 1.0
        out['sides'][side] = {
            'mesh': mesh.group(1) if mesh else '',
            'fdir': [float(v) for v in fdir.group(1).split()] if fdir else None,
            'x_sign': xsgn, 'y_sign': ysgn,
        }
    return out


def read_hub(export_dir):
    """从导出包读 hub 的质量、质心偏移与惯量。"""
    import glob
    import os
    import xml.etree.ElementTree as ET
    f = glob.glob(os.path.join(export_dir, 'urdf', '*.urdf'))[0]
    root = ET.parse(f).getroot()
    for l in root.findall('link'):
        if not l.get('name').startswith('whee'):
            continue
        i = l.find('inertial')
        it = i.find('inertia')
        com = [float(v) for v in i.find('origin').get('xyz').split()]
        return {
            'mass': float(i.find('mass').get('value')),
            'com_y': abs(com[1]),
            'ixx': float(it.get('ixx')), 'iyy': float(it.get('iyy')), 'izz': float(it.get('izz')),
        }
    raise SystemExit("导出包里没找到 hub link(名字应以 whee 开头)")


def beta_from_fdir(fdir):
    """fdir 的 y 分量决定手性: y>0 -> +3pi/4, y<0 -> -3pi/4。"""
    if not fdir:
        return None
    return "3.0*pi/4.0" if fdir[1] > 0 else "-3.0*pi/4.0"


def emit_xacro_file(path, geo, roller_radius, phases, hub, car):
    L = []
    a = L.append
    a('<?xml version="1.0" encoding="utf-8"?>')
    a('<!-- 由 scripts/solve_mecanum_params.py 的 emit-xacro 模式生成, 勿手工编辑 -->')
    a('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')
    a('')
    a('  <xacro:macro name="mecanum_roller" params="side idx ring_radius roller_radius plane_y phase_deg beta">')
    a('    <xacro:property name="theta" value="${(phase_deg + idx * %s) * pi / 180.0}" />' % geo['spacing'])
    a('')
    a('    <joint name="${side}_roller_${idx}_joint" type="continuous">')
    a('      <parent link="${side}_wheel_link" />')
    a('      <child link="${side}_roller_${idx}_link" />')
    a('      <origin xyz="${ring_radius * cos(theta)} ${plane_y} ${ring_radius * sin(theta)}"')
    a('              rpy="${beta} ${-theta} 0.0" />')
    a('      <axis xyz="0.0 0.0 1.0" />')
    a('      <dynamics damping="0.0" friction="0.0" />')
    a('    </joint>')
    a('')
    a('    <link name="${side}_roller_${idx}_link">')
    a('      <collision>')
    a('        <geometry>')
    a('          <sphere radius="${roller_radius}" />')
    a('        </geometry>')
    a('        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />')
    a('      </collision>')
    a('')
    a('      <inertial>')
    a('        <mass value="%.5f" />' % geo['mass'])
    a('        <inertia ixx="%.4e" ixy="0.0"        ixz="0.0"' % geo['i_trans'])
    a('                                  iyy="%.4e" iyz="0.0"' % geo['i_trans'])
    a('                                                   izz="%.4e" />' % geo['i_axial'])
    a('        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />')
    a('      </inertial>')
    a('    </link>')
    a('  </xacro:macro>')
    a('')
    a('  <xacro:macro name="mecanum_wheel" params="side wheel_radius">')
    a('')
    a('    <xacro:property name="wheel_separation_x" value="%s" />' % car['wheel_separation_x'])
    a('    <xacro:property name="wheel_separation_y" value="%s" />' % car['wheel_separation_y'])
    a('    <xacro:property name="ring_radius" value="%s" />' % geo['ring_radius'])
    a('    <xacro:property name="roller_radius" value="%s" />' % roller_radius)
    a('    <xacro:property name="half_hub_width" value="%s" />' % geo['plane_y_abs'])
    a('    <xacro:property name="hub_com_offset" value="%.6f" />' % hub['com_y'])
    a('')
    a('    <xacro:property name="hub_mass" value="%.7f" />' % hub['mass'])
    a('    <xacro:property name="hub_ixx" value="%.5e" />' % hub['ixx'])
    a('    <xacro:property name="hub_iyy" value="%.5e" />' % hub['iyy'])
    a('    <xacro:property name="hub_izz" value="%.5e" />' % hub['izz'])
    a('')
    for side in SIDES:
        c = car['sides'].get(side)
        if not c:
            continue
        beta = beta_from_fdir(c['fdir'])
        xs = '' if c['x_sign'] > 0 else '-'
        ys = '' if c['y_sign'] > 0 else '-'
        a("    <xacro:if value=\"${side == '%s'}\">" % side)
        a('      <xacro:property name="x" value="${%swheel_separation_x/2.0}" />' % xs)
        a('      <xacro:property name="y" value="${%swheel_separation_y/2.0}" />' % ys)
        a('      <xacro:property name="y_sign" value="${%s1.0}" />' % ('-' if c['y_sign'] > 0 else ''))
        a('      <xacro:property name="mesh" value="%s" />' % c['mesh'])
        a('      <xacro:property name="phase_deg" value="${%s}" />' % phases.get(side, 0.0))
        a('      <xacro:property name="beta" value="${%s}" />' % beta)
        a('    </xacro:if>')
    a('')
    a('    <xacro:property name="plane_y" value="${y_sign * half_hub_width}" />')
    a('')
    a('    <joint name="${side}_wheel_joint" type="continuous">')
    a('      <parent link="base_link" />')
    a('      <child link="${side}_wheel_link" />')
    a('      <origin xyz="${x} ${y} %s" rpy="0.0 0.0 0.0" />' % car['joint_z'])
    a('      <axis xyz="0.0 1.0 0.0" />')
    a('      <limit effort="%s" velocity="%s" />' % (car['effort'], car['velocity']))
    a('      <dynamics damping="%s" friction="%s" />' % (car['damping'], car['friction']))
    a('    </joint>')
    a('')
    a('    <link name="${side}_wheel_link">')
    a('      <visual>')
    a('        <geometry>')
    a('          <mesh filename="${mesh}" />')
    a('        </geometry>')
    a('        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />')
    a('        <material name="">')
    a('          <color rgba="1 1 1 1" />')
    a('        </material>')
    a('      </visual>')
    a('')
    a('      <inertial>')
    a('        <mass value="${hub_mass}" />')
    a('        <inertia ixx="${hub_ixx}" ixy="0.0"        ixz="0.0"')
    a('                                  iyy="${hub_iyy}" iyz="0.0"')
    a('                                                   izz="${hub_izz}" />')
    a('        <origin xyz="0.0 ${y_sign * hub_com_offset} 0.0" rpy="0.0 0.0 0.0" />')
    a('      </inertial>')
    a('    </link>')
    a('')
    for idx in range(geo['count']):
        a('    <xacro:mecanum_roller side="${side}" idx="%d" ring_radius="${ring_radius}" '
          'roller_radius="${roller_radius}" plane_y="${plane_y}" '
          'phase_deg="${phase_deg}" beta="${beta}" />' % idx)
    a('')
    a('  </xacro:macro>')
    a('</robot>')
    open(path, 'w').write('\n'.join(L) + '\n')
    print("")
    print("已生成", path)
    for side in SIDES:
        c = car['sides'].get(side)
        if c:
            print("     %s  fdir=%s -> beta=%s   phase=%s"
                  % (side, c['fdir'], beta_from_fdir(c['fdir']), phases.get(side)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export", required=True, help="SW2URDF 导出包目录")
    ap.add_argument("--roller-stl", help="单个 roller 的 STL")
    ap.add_argument("--wheel-mesh", nargs="*", default=[],
                    help="整轮 mesh, 形如 fl=path.stl")
    ap.add_argument("--verify", help="展开后的 .urdf, 反解完顺带校验")
    ap.add_argument("--realcar-xacro", help="真车 wheel.urdf.xacro, 提供整车尺寸与 fdir 手性")
    ap.add_argument("--emit-xacro", help="生成 wheel_mecanum_real.urdf.xacro 到该路径")
    args = ap.parse_args()

    geo = solve_roller_geometry(args.export)

    roller_radius = None
    if args.roller_stl:
        roller_radius = solve_roller_radius(args.roller_stl)

    phases = {}
    pairs = []
    for item in args.wheel_mesh:
        if "=" not in item:
            print("WARNING: --wheel-mesh 需形如 fl=path.stl, 已跳过", item)
            continue
        side, path = item.split("=", 1)
        pairs.append((side, path))
    if pairs:
        phases = solve_phases(pairs)

    if roller_radius is not None:
        emit_xacro(geo, roller_radius, phases)

    if args.emit_xacro:
        if not (args.realcar_xacro and roller_radius is not None and phases):
            raise SystemExit("--emit-xacro 需同时提供 --realcar-xacro / --roller-stl / --wheel-mesh")
        car = parse_realcar_xacro(args.realcar_xacro)
        hub = read_hub(args.export)
        emit_xacro_file(args.emit_xacro, geo, roller_radius, phases, hub, car)

    if args.verify:
        print("")
        print("=" * 96)
        print("校验展开后的 urdf")
        print("=" * 96)
        os.system("python3 %s %s" % (
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "verify_rollers.py"),
            args.verify))

    if not geo['ok']:
        print("")
        print("反解存在不一致项, 先按 1.1 节要求让结构方重新导出。")


if __name__ == '__main__':
    main()
