# 官方 Kaya(3 轮全向)对照工具(纯 Isaac, 无 ROS2):
#   默认       : 跑内置动作序列(前进/后退/横移/旋转循环), 看手感/飘不飘
#   加 --key   : 终端键盘遥控(键位同 teleop_twist_keyboard), 在【本终端】按键
#   总是加载调试扩展 —— 碰撞体线框 + Script Editor + Stage + Property 窗口
#   standalone 默认只加载跑仿真必需的扩展, 所以这些要显式打开
# 运行: ./isaacsim/python.sh kaya_motion_demo.py          (动作 demo)
#       ./isaacsim/python.sh kaya_motion_demo.py --key    (键盘遥控)

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--key", action="store_true", help="终端键盘遥控模式")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

if True:
    import carb.settings
    import omni.kit.app
    from isaacsim.core.utils.extensions import enable_extension

    for ext in ("omni.physx.ui", "omni.kit.window.script_editor",
                "omni.kit.window.stage", "omni.kit.property.bundle"):
        try:
            enable_extension(ext)
        except Exception as exc:
            print("[inspect] failed to load", ext, exc)
    simulation_app.update()

    mgr = omni.kit.app.get_app().get_extension_manager()
    print("[inspect] loaded physx / debug extensions:")
    for e in mgr.get_extensions():
        name = e.get("name", "")
        if any(w in name for w in ("physx", "script_editor", "window.stage", "debugdraw")):
            state = "on " if mgr.is_extension_enabled(name) else "off"
            print("   ", state, name)

    settings = carb.settings.get_settings()
    for key in ("/persistent/physics/visualizationDisplayColliders",
                "/persistent/physics/visualizationDisplayJoints"):
        settings.set(key, True)
        print("[inspect]", key, "->", settings.get(key))

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import DomeLight
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import HolonomicController
from isaacsim.robot.experimental.wheeled_robots.robots import (
    HolonomicRobotUsdSetup,
    WheeledRobot,
)
from isaacsim.storage.native import get_assets_root_path

DEVICE = "cpu"

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")
kaya_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Kaya/kaya.usd"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
    path="/World/ground",
)
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)

my_kaya = WheeledRobot(
    paths="/World/Kaya",
    wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
    usd_path=kaya_asset_path,
    positions=[0.0, 0.0, 0.02],
    orientations=[1.0, 0.0, 0.0, 0.0],
)
kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path=my_kaya.paths[0],
    com_prim_path="/World/Kaya/base_link/control_offset",
)
(
    wheel_radius,
    wheel_positions,
    wheel_orientations,
    mecanum_angles,
    wheel_axis,
    up_axis,
) = kaya_setup.get_holonomic_controller_params()
my_controller = HolonomicController(
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device=DEVICE)
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)
app_utils.play()
app_utils.update_app(steps=10)


def run_demo():
    segments = [
        ([0.4, 0.0, 0.0], "forward"),
        ([-0.4, 0.0, 0.0], "backward"),
        ([0.0, 0.4, 0.0], "strafe"),
        ([0.0, 0.0, 1.0], "rotate"),
        ([0.0, 0.0, 0.0], "stop"),
    ]
    seg_steps = 120
    step_count = 0
    while simulation_app.is_running():
        simulation_app.update()
        if app_utils.is_playing():
            command, label = segments[min(step_count // seg_steps, len(segments) - 1)]
            if step_count % seg_steps == 0:
                print(f"[{step_count}] {label}: {command}")
            my_kaya.apply_wheel_actions(my_controller.forward(command))
            step_count += 1
        if step_count >= seg_steps * len(segments):
            step_count = 0
    app_utils.stop()
    simulation_app.close()


def run_keyboard():
    import sys
    import select
    import termios
    import tty
    import atexit
    import os
    import signal

    msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""
    move = {
        "i": (1, 0, 0), "o": (1, 0, -1), "j": (0, 0, 1), "l": (0, 0, -1), "u": (1, 0, 1),
        ",": (-1, 0, 0), ".": (-1, 0, 1), "m": (-1, 0, -1),
        "I": (1, 0, 0), "O": (1, -1, 0), "J": (0, 1, 0), "L": (0, -1, 0), "U": (1, 1, 0),
        "<": (-1, 0, 0), ">": (-1, -1, 0), "M": (-1, 1, 0),
        "k": (0, 0, 0), "K": (0, 0, 0),
    }
    speed_keys = {
        "q": (1.1, 1.1), "z": (0.9, 0.9),
        "w": (1.1, 1.0), "x": (0.9, 1.0),
        "e": (1.0, 1.1), "c": (1.0, 0.9),
    }
    speed = 0.5
    turn = 1.0
    dirv = [0, 0, 0]
    status = 0
    print(msg)
    print(f"currently:\tspeed {speed}\tturn {turn}")

    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)

    def restore_term():
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)

    atexit.register(restore_term)
    signal.signal(signal.SIGINT, lambda *a: (restore_term(), os._exit(0)))  # Ctrl-C 先恢复终端再退出
    try:
        tty.setcbreak(fd)
        while simulation_app.is_running():
            simulation_app.update()
            while select.select([sys.stdin], [], [], 0.0)[0]:
                c = sys.stdin.read(1)
                if c in move:
                    dirv = list(move[c])
                elif c in speed_keys:
                    speed *= speed_keys[c][0]
                    turn *= speed_keys[c][1]
                    print(f"currently:\tspeed {speed:.3f}\tturn {turn:.3f}")
                    if status == 14:
                        print(msg)
                    status = (status + 1) % 15
                elif c == "\x03":
                    raise KeyboardInterrupt
                else:
                    dirv = [0, 0, 0]
            if app_utils.is_playing():
                command = [dirv[0] * speed, dirv[1] * speed, dirv[2] * turn]
                my_kaya.apply_wheel_actions(my_controller.forward(command))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        app_utils.stop()
        simulation_app.close()


run_keyboard() if args.key else run_demo()
