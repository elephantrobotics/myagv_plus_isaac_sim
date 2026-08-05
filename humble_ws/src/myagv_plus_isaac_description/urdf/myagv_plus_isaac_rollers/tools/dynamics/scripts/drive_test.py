# Script Editor 片段: 运动指令 + 测量
#   每次运行先复位 4 个轮驱动(target=0, maxForce=1.35), 再按 MODE 下指令,
#   然后打印轮速 / 分轮滚子自转 / 车身系速度 / 达成率
#   targetVelocity 是 USD 属性, Stop/Play 不会清, 所以必须显式复位
#
# 用法: Play 状态下跑两次 —— 第一次下指令, 等 2 秒后第二次看稳态读数
#
# MODE: vy 左移 / vx 前进 / wz 自转 / stop 停
#       diag_a 对角 fl+rr / diag_b 对角 fr+rl

MODE = "vy"
SPEED = 0.2
W_DIAG = 5.0

from pxr import UsdPhysics
import omni.usd
import omni.timeline
import math

R = 0.04
LXY = 0.21722
RAD2DEG = 57.29577951308232
MAX_FORCE = 1.35
BASE = "/myAGV_plus/Geometry/base_footprint/base_link"
ORDER = ("fl", "fr", "rl", "rr")

MIX = {
    "vx": (1.0, 1.0, 1.0, 1.0),
    "vy": (-1.0, 1.0, 1.0, -1.0),
    "wz": (-1.0, 1.0, -1.0, 1.0),
    "stop": (0.0, 0.0, 0.0, 0.0),
}
DIAG = {"diag_a": ("fl", "rr"), "diag_b": ("fr", "rl")}

stage = omni.usd.get_context().get_stage()
joints = {}
for prim in stage.Traverse():
    n = prim.GetName()
    if n.endswith("_wheel_joint"):
        joints[n[:2]] = prim


def drive_of(prim):
    d = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not d:
        d = UsdPhysics.DriveAPI.Apply(prim, "angular")
    return d


for side in ORDER:
    if side in joints:
        d = drive_of(joints[side])
        d.CreateTargetVelocityAttr().Set(0.0)
        d.CreateMaxForceAttr().Set(MAX_FORCE)

cmd = {}
if MODE in MIX:
    scale = SPEED * LXY / R if MODE == "wz" else SPEED / R
    for i, side in enumerate(ORDER):
        if side in joints:
            w = MIX[MODE][i] * scale
            drive_of(joints[side]).CreateTargetVelocityAttr().Set(w * RAD2DEG)
            cmd[side] = w
    print("MODE", MODE, " SPEED", SPEED, " cmd wheel speed", round(scale, 3), "rad/s")
elif MODE in DIAG:
    for side in ORDER:
        if side not in joints:
            continue
        d = drive_of(joints[side])
        if side in DIAG[MODE]:
            d.CreateTargetVelocityAttr().Set(W_DIAG * RAD2DEG)
            cmd[side] = W_DIAG
        else:
            d.CreateMaxForceAttr().Set(0.0)
            cmd[side] = 0.0
    print("MODE", MODE, " driving", DIAG[MODE], " others released")
else:
    print("unknown MODE:", MODE)

if not omni.timeline.get_timeline_interface().is_playing():
    print("!!! Stopped, readings invalid; press Play then run twice")
else:
    Art = None
    try:
        from isaacsim.core.prims import SingleArticulation as Art
    except Exception:
        try:
            from omni.isaac.core.articulations import Articulation as Art
        except Exception:
            pass
    if Art is None:
        print("Articulation interface not found")
    else:
        art = Art(BASE)
        try:
            art.initialize()
        except Exception:
            pass
        names = list(art.dof_names)
        vels = art.get_joint_velocities()

        print("")
        for side in ORDER:
            wk = side + "_wheel_joint"
            if wk not in names:
                continue
            wv = float(vels[names.index(wk)])
            rs = sorted(abs(float(vels[i])) for i, n in enumerate(names)
                        if n.startswith(side + "_roller_"))
            c = cmd.get(side)
            line = "  " + side + " wheel " + str(round(wv, 2))
            if c is not None:
                line += " (cmd " + str(round(c, 2)) + ")"
            if rs:
                line += " | roller max " + str(round(rs[-1], 1))
                line += " spinning " + str(sum(1 for v in rs if v > 1.0)) + "/9"
            print(line)

        lin = art.get_linear_velocity()
        angv = art.get_angular_velocity()
        pos, quat = art.get_world_pose()
        qw, qx, qy, qz = (float(quat[0]), float(quat[1]),
                          float(quat[2]), float(quat[3]))
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qw * qz)
        r10 = 2 * (qx * qy + qw * qz)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r20 = 2 * (qx * qz - qw * qy)
        r21 = 2 * (qy * qz + qw * qx)
        wx, wy, wz_ = float(lin[0]), float(lin[1]), float(lin[2])
        bx = r00 * wx + r10 * wy + r20 * wz_
        by = r01 * wx + r11 * wy + r21 * wz_

        print("")
        print("body vx =", round(bx, 4), " vy =", round(by, 4),
              " yaw rate =", round(float(angv[2]), 3))
        mag = math.hypot(bx, by)
        if mag > 1e-3:
            print("heading =", round(math.degrees(math.atan2(by, bx)), 1),
                  "deg (0=front 90=left -90=right)  speed", round(mag, 4), "m/s")
        w = {}
        for side in ORDER:
            k = side + "_wheel_joint"
            w[side] = float(vels[names.index(k)]) if k in names else 0.0
        fk_vx = R / 4.0 * (w["fl"] + w["fr"] + w["rl"] + w["rr"])
        fk_vy = R / 4.0 * (-w["fl"] + w["fr"] + w["rl"] - w["rr"])
        print("FK from actual wheel speed vx =", round(fk_vx, 4), " vy =", round(fk_vy, 4))

        if MODE in ("vx", "vy"):
            actual = by if MODE == "vy" else bx
            fk = fk_vy if MODE == "vy" else fk_vx
            print("vs command =", round(abs(actual) / SPEED * 100, 1), "%",
                  "  (includes drive tracking error)")
            if abs(fk) > 1e-3:
                print("vs wheel FK =", round(abs(actual) / abs(fk) * 100, 1), "%",
                      "  (pure slip; this is the metric to match the real robot)")
