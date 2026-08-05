# Script Editor 片段: 测量 OmniGraph / ros2_control 驱动时的速度达成率
#   只读, 不下任何指令, 不会与图抢夺 targetVelocity
#   Play 状态下, 车已进入稳态后运行
#
#   指令来源自动识别:
#     /Graph/Control/constant_double3.inputs:value      最小版
#     /Graph/ROS_Control/ros2_subscribe_twist           ROS2 版(读订阅到的值)
#     /Graph/ROS_Control/subscribe_joint_state           ros2_control 版(轮速指令反算车体指令)

from pxr import UsdPhysics
import omni.usd
import omni.timeline
import math

R = 0.04
LXY = 0.21722
CONTROL_LXY = 0.221
BASE = "/myAGV_plus/Geometry/base_footprint/base_link"
ORDER = ("fl", "fr", "rl", "rr")

stage = omni.usd.get_context().get_stage()

cmd = None
wheel_cmd = {}
p = stage.GetPrimAtPath("/Graph/Control/constant_double3")
if p:
    v = p.GetAttribute("inputs:value").Get()
    if v is not None:
        cmd = [float(v[0]), float(v[1]), float(v[2])]
        print("command source: constant_double3 =", [round(x, 4) for x in cmd])
if cmd is None and stage.GetPrimAtPath("/Graph/ROS_Control/ros2_subscribe_twist"):
    try:
        import omni.graph.core as og
        lin = og.Controller.attribute(
            "/Graph/ROS_Control/ros2_subscribe_twist.outputs:linearVelocity").get()
        ang = og.Controller.attribute(
            "/Graph/ROS_Control/ros2_subscribe_twist.outputs:angularVelocity").get()
        cmd = [float(lin[0]), float(lin[1]), float(ang[2])]
        print("command source: ros2_subscribe_twist =", [round(x, 4) for x in cmd])
    except Exception as e:
        print("failed to read ros2_subscribe_twist outputs:", e)
if cmd is None and stage.GetPrimAtPath("/Graph/ROS_Control/subscribe_joint_state"):
    try:
        import omni.graph.core as og
        joint_names_raw = og.Controller.attribute(
            "/Graph/ROS_Control/subscribe_joint_state.outputs:jointNames").get()
        velocity_command_raw = og.Controller.attribute(
            "/Graph/ROS_Control/subscribe_joint_state.outputs:velocityCommand").get()
        joint_names = list(joint_names_raw) if joint_names_raw is not None else []
        velocity_command = list(velocity_command_raw) if velocity_command_raw is not None else []
        by_name = {str(n): float(v) for n, v in zip(joint_names, velocity_command)}
        for side in ORDER:
            name = side + "_wheel_joint"
            if name in by_name:
                wheel_cmd[side] = by_name[name]
        if len(wheel_cmd) == 4:
            cmd = [
                R / 4.0 * (wheel_cmd["fl"] + wheel_cmd["fr"] +
                           wheel_cmd["rl"] + wheel_cmd["rr"]),
                R / 4.0 * (-wheel_cmd["fl"] + wheel_cmd["fr"] +
                           wheel_cmd["rl"] - wheel_cmd["rr"]),
                R / (4.0 * CONTROL_LXY) * (-wheel_cmd["fl"] + wheel_cmd["fr"] -
                                           wheel_cmd["rl"] + wheel_cmd["rr"]),
            ]
            print("command source: ros2_control wheel command FK =",
                  [round(x, 4) for x in cmd])
            print("wheel command:",
                  "  ".join("%s=%+.3f" % (side, wheel_cmd[side]) for side in ORDER))
    except Exception as e:
        print("failed to read ros2_control wheel command outputs:", e)
if cmd is None:
    print("no command read; reporting measured values only")

gain = None
for path in ("/Graph/Control/holonomic_controller",
             "/Graph/ROS_Control/holonomic_controller"):
    p = stage.GetPrimAtPath(path)
    if p:
        gain = p.GetAttribute("inputs:angularGain").Get()
if gain is not None:
    print("angularGain =", gain)

if not omni.timeline.get_timeline_interface().is_playing():
    print("!!! Stopped, readings invalid")
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

        w = {}
        print("")
        for side in ORDER:
            k = side + "_wheel_joint"
            if k not in names:
                continue
            w[side] = float(vels[names.index(k)])
            rs = sorted(abs(float(vels[i])) for i, n in enumerate(names)
                        if n.startswith(side + "_roller_"))
            if side in wheel_cmd:
                target = wheel_cmd[side]
                ratio = abs(w[side]) / abs(target) * 100 if abs(target) > 1e-6 else 0.0
                print("  %-3s cmd %7.3f | actual %7.3f | track %5.1f%% | roller max %6.1f  spinning %d/9"
                      % (side, target, w[side], ratio, rs[-1] if rs else 0,
                         sum(1 for v in rs if v > 1.0)))
            else:
                print("  %-3s wheel %7.3f | roller max %6.1f  spinning %d/9"
                      % (side, w[side], rs[-1] if rs else 0,
                         sum(1 for v in rs if v > 1.0)))

        lin = art.get_linear_velocity()
        angv = art.get_angular_velocity()
        pos, quat = art.get_world_pose()
        qw, qx, qy, qz = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qw * qz)
        r10 = 2 * (qx * qy + qw * qz)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r20 = 2 * (qx * qz - qw * qy)
        r21 = 2 * (qy * qz + qw * qx)
        wx, wy, wz_ = float(lin[0]), float(lin[1]), float(lin[2])
        bx = r00 * wx + r10 * wy + r20 * wz_
        by = r01 * wx + r11 * wy + r21 * wz_
        yaw_rate = float(angv[2])

        print("")
        print("body vx = %+.4f  vy = %+.4f  wz = %+.4f" % (bx, by, yaw_rate))

        fk = None
        if len(w) == 4:
            fk_vx = R / 4.0 * (w["fl"] + w["fr"] + w["rl"] + w["rr"])
            fk_vy = R / 4.0 * (-w["fl"] + w["fr"] + w["rl"] - w["rr"])
            fk_wz = R / (4.0 * LXY) * (-w["fl"] + w["fr"] - w["rl"] + w["rr"])
            fk = (fk_vx, fk_vy, fk_wz)
            print("wheel FK vx = %+.4f  vy = %+.4f  wz = %+.4f" % (fk_vx, fk_vy, fk_wz))

        if cmd:
            print("")
            labels = ("vx", "vy", "wz")
            actual = (bx, by, yaw_rate)
            for i in range(3):
                if abs(cmd[i]) < 1e-6:
                    continue
                print("  %s  cmd %+.4f  body %+.4f  body/cmd %.1f%%"
                      % (labels[i], cmd[i], actual[i],
                         abs(actual[i]) / abs(cmd[i]) * 100))
                if fk is not None:
                    print("      wheel FK %+.4f  FK/cmd %.1f%%  body/FK %.1f%%"
                          % (fk[i], abs(fk[i]) / abs(cmd[i]) * 100,
                             abs(actual[i]) / abs(fk[i]) * 100 if abs(fk[i]) > 1e-6 else 0.0))
            cross = [abs(actual[i]) for i in range(3) if abs(cmd[i]) < 1e-6]
            if cross:
                print("  cross-talk on uncommanded axes max %.4f" % max(cross))
