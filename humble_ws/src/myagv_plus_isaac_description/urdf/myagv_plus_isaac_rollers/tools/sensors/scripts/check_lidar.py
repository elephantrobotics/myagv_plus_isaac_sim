# RTX Lidar 一键检查。Isaac Script Editor 打开本文件运行。
#
# 只读。Play 状态下跑最完整(渲染产品在 Play 后才建、刚体位姿以 Play 为准)。
# 每项失败会直接给出要改哪个文件的哪个参数。
# ROS 侧的有效点率用 tools/sensors/scripts/scan_stats.py 量。

import math
import omni.usd
from pxr import Usd, UsdGeom, Gf

LIDAR = "/myAGV_plus/Geometry/base_footprint/base_link/laser_link/rtx_lidar"
BASE = "/myAGV_plus/Geometry/base_footprint/base_link"
ROBOT = "/myAGV_plus"
GRAPH = "/Graph/ROS_LidarRTX"

POINTS_PER_REV = 300
NEAR = 0.1
FAR = 12.0
SCAN_HZ = 10
OPEN_MIN = 250

stage = omni.usd.get_context().get_stage()
fails = []


def out(idx, name, ok, info, fix=None):
    print("[%s] %-14s %-4s %s" % (idx, name, "OK" if ok else "FAIL", info))
    if not ok:
        fails.append(name)
        for line in (fix or []):
            print("      -> " + line)


# ---------------- 1 扩展 ----------------
try:
    import omni.kit.app
    mgr = omni.kit.app.get_app().get_extension_manager()
    off = [n for n in ("isaacsim.ros2.bridge", "isaacsim.sensors.rtx",
                       "isaacsim.core.nodes", "omni.replicator.core")
           if not mgr.is_extension_enabled(n)]
    out("1/6", "extensions", not off, "all enabled" if not off else "disabled: " + ", ".join(off),
        ["Window -> Extensions: search and enable the listed extensions"])
except Exception as e:
    out("1/6", "extensions", False, str(e))

# ---------------- 2 雷达参数 ----------------
prim = stage.GetPrimAtPath(LIDAR)
if not prim:
    out("2/6", "lidar params", False, "prim not found: " + LIDAR,
        ["lidar usda should have def OmniLidar \"rtx_lidar\" under laser_link"])
else:
    def g(k):
        a = prim.GetAttribute(k)
        return a.Get() if a and a.IsValid() else None

    emit = g("omni:sensor:Core:numberOfEmitters")
    sr = g("omni:sensor:Core:scanRateBaseHz")
    rr = g("omni:sensor:Core:reportRateBaseHz")
    tk = g("omni:sensor:tickRate")
    nr = g("omni:sensor:Core:nearRangeM")
    fr = g("omni:sensor:Core:farRangeM")
    ppr = (float(rr) / float(sr)) if (rr and sr) else None

    bad = []
    if emit is None:
        bad.append("Example_Rotary_2D reference not loaded (numberOfEmitters unreadable); check network or asset path")
    if sr and tk and abs(float(tk) - float(sr)) > 1e-6:
        bad.append("tickRate(%s) != scanRateBaseHz(%s) -- must be equal, otherwise only partial scans per frame" % (tk, sr))
    if ppr is not None and abs(ppr - POINTS_PER_REV) > 0.5:
        bad.append("points/rev = %g, should be %d (real X2): reportRateBaseHz = %d" %
                   (ppr, POINTS_PER_REV, POINTS_PER_REV * SCAN_HZ))
    if nr is not None and abs(float(nr) - NEAR) > 1e-4:
        bad.append("nearRangeM = %s, should be %s (match real range_min)" % (nr, NEAR))
    if fr is not None and abs(float(fr) - FAR) > 1e-4:
        bad.append("farRangeM = %s, should be %s" % (fr, FAR))

    info = "%s pts/rev  %sHz  near/far %s~%sm" % (ppr, sr, nr, fr)
    out("2/6", "lidar params", not bad, info,
        bad + ["edit rtx_lidar omni:sensor:* in myagv_plus_isaac_rollers_ros2_lidar.usda,",
               "or run tools/sensors/scripts/tune_rtx_lidar_2d_params.py"] if bad else [])

# ---------------- 3 扫描面 ----------------


def build_tris():
    tris, src = [], {}
    for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh) or p.GetPath().pathString.startswith(LIDAR):
            continue
        m = UsdGeom.Mesh(p)
        pts, idx, cnt = (m.GetPointsAttr().Get(), m.GetFaceVertexIndicesAttr().Get(),
                         m.GetFaceVertexCountsAttr().Get())
        if not pts or not idx or not cnt:
            continue
        xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
        wp = [xf.Transform(Gf.Vec3d(q[0], q[1], q[2])) for q in pts]
        o, n0 = 0, len(tris)
        for c in cnt:
            for k in range(1, c - 1):
                tris.append((wp[idx[o]], wp[idx[o + k]], wp[idx[o + k + 1]]))
            o += c
        src[p.GetPath().pathString.split("/")[-2]] = (n0, len(tris))
    return tris, src


def occlusion(tris, Z, ox, oy):
    segs = []
    for i, v in enumerate(tris):
        pt = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            z0, z1 = v[a][2], v[b][2]
            if (z0 - Z) * (z1 - Z) < 0:
                t = (Z - z0) / (z1 - z0)
                pt.append((v[a][0] + t * (v[b][0] - v[a][0]),
                           v[a][1] + t * (v[b][1] - v[a][1]), i))
        if len(pt) >= 2:
            segs.append((pt[0], pt[1]))
    near = far = 0
    who = {}
    for d in range(360):
        ang = math.radians(d)
        dx, dy = math.cos(ang), math.sin(ang)
        best, bi = None, -1
        for p, q in segs:
            ex, ey = q[0] - p[0], q[1] - p[1]
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((p[0] - ox) * ey - (p[1] - oy) * ex) / den
            u = ((p[0] - ox) * dy - (p[1] - oy) * dx) / den
            if t > 1e-6 and -1e-9 <= u <= 1 + 1e-9 and (best is None or t < best):
                best, bi = t, p[2]
        if best is None:
            continue
        if best < NEAR:
            near += 1
            who[bi] = who.get(bi, 0) + 1
        else:
            far += 1
    return near, far, 360 - near - far, who


if prim:
    LW = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    BW = UsdGeom.Xformable(stage.GetPrimAtPath(BASE)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    lz = LW[2] - BW[2]
    tris, src = build_tris()
    near, far, op, who = occlusion(tris, LW[2], LW[0], LW[1])
    ok = op >= OPEN_MIN
    info = "base_link local z=%.5f  open %d deg  self-blocked %d deg" % (lz, op, near)
    fix = []
    if not ok:
        def owner(i):
            for k, v in src.items():
                if v[0] <= i < v[1]:
                    return k
            return "?"
        agg = {}
        for i, c in who.items():
            k = owner(i)
            agg[k] = agg.get(k, 0) + c
        for k, v in sorted(agg.items(), key=lambda x: -x[1])[:3]:
            fix.append("blocked %d deg by %s" % (v, k))
        print("      (over-occluded, sweeping +/-30mm for an open band, please wait)")
        band = []
        for mm in range(-30, 31, 2):
            Z = LW[2] + mm / 1000.0
            o2 = occlusion(tris, Z, LW[0], LW[1])[2]
            if o2 >= OPEN_MIN:
                band.append(Z - BW[2])
        if band:
            fix.append("open band: base_link local z %.4f ~ %.4f, take the midpoint" % (min(band), max(band)))
        else:
            fix.append("no open band within +/-30mm; widen the sweep or check the chassis model")
        fix += ["edit both, keeping the values equal:",
                "  payloads/base.usda            laser_link xformOp:translate 3rd component",
                "  payloads/Physics/physics.usda lidar_joint physics:localPos0 3rd component",
                "laser_link is a rigid body: at Play its pose comes from the joint, editing the Xform alone has no effect",
                "also offset laser_link visual/collision/inertial origin z inversely to keep the shell in place"]
    out("3/6", "scan plane", ok, info, fix)

# ---------------- 4 OmniGraph ----------------
try:
    import omni.graph.core as og
    g = og.get_graph_by_path(GRAPH)
    if g is None:
        out("4/6", "OmniGraph", False, "not found: " + GRAPH,
            ["lidar usda should have def OmniGraph \"ROS_LidarRTX\""])
    else:
        nodes = list(g.get_nodes())
        bad = [n.get_prim_path().split("/")[-1] for n in nodes
               if og.get_node_type(n.get_type_name()) is None]
        out("4/6", "OmniGraph", not bad, "%d nodes" % len(nodes),
            ["unregistered node types: " + ", ".join(bad)])
except Exception as e:
    out("4/6", "OmniGraph", False, str(e))

# ---------------- 5 渲染产品 ----------------
try:
    import omni.graph.core as og
    import omni.timeline
    rp = og.Controller.attribute(GRAPH + "/RenderProduct.outputs:renderProductPath").get()
    cam = stage.GetPrimAtPath(GRAPH + "/RenderProduct").GetRelationship("inputs:cameraPrim").GetTargets()
    playing = omni.timeline.get_timeline_interface().is_playing()
    ok = bool(rp) and bool(cam) and str(cam[0]) == LIDAR
    info = "%s  cameraPrim=%s" % (rp or "<empty>", cam[0] if cam else "<empty>")
    fix = []
    if not rp:
        fix.append("empty at Stop is normal, rerun after Play" if not playing
                   else "RunOnce not triggered; check OnPlaybackTick -> RunOnce -> RenderProduct exec line")
    if cam and str(cam[0]) != LIDAR:
        fix.append("cameraPrim points elsewhere, should be " + LIDAR)
    out("5/6", "render product", ok, info, fix)
except Exception as e:
    out("5/6", "render product", False, str(e))

# ---------------- 6 发布节点 ----------------
rows, bad = [], []
for n, want in (("LaserScanPublish", "laser_scan"), ("PointCloudPublish", "point_cloud")):
    p = stage.GetPrimAtPath(GRAPH + "/" + n)
    if not p:
        bad.append(n + " not found")
        continue

    def ga(k):
        a = p.GetAttribute(k)
        return a.Get() if a and a.IsValid() else None
    t, tp, fid, en = ga("inputs:type"), ga("inputs:topicName"), ga("inputs:frameId"), ga("inputs:enabled")
    rows.append("%s=%s%s" % (tp, t, "" if en else "(disabled)"))
    if t != want:
        bad.append("%s type should be %s, got %s" % (n, want, t))
    if fid != "laser_link":
        bad.append("%s frameId should be laser_link, got %s" % (n, fid))
out("6/6", "publishers", not bad, "  ".join(rows), bad + ["edit inputs:* on the matching node under ROS_LidarRTX"] if bad else [])

print("")
if fails:
    print("FAILED: " + ", ".join(fails))
else:
    print("all passed. measure valid-point ratio on the ROS side:")
    print("  python3 <pkg>/urdf/myagv_plus_isaac_rollers/tools/sensors/scripts/scan_stats.py")
