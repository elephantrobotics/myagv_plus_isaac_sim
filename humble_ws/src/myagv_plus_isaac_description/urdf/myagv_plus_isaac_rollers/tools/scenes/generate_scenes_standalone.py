# 一键批量生成离线示例场景: 遍历官方环境, 基于无环境 base 合成场景并 Collect 成自包含离线 usda。
# 用法: <isaac>/python.sh generate_scenes_standalone.py        (默认 headless 批量)
#       加 --gui 可视化观察; 加 --only warehouse 只做某一个
# 前提: BASE_USD 指向无环境母版 myagv_plus_isaac_rollers_noenv.usda(机器人 + 图 + 物理, 无地面/房间)。
# rollers/结构改动后只需重跑本脚本, 全部场景自动重生成。

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--only", default=None)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not args.gui})

import asyncio

import omni.kit.app
import omni.usd
from pxr import Gf, UsdGeom
from isaacsim.storage.native import get_assets_root_path

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.kit.usd.collect", True)
import omni.kit.usd.collect

BASE_USD = "/home/elephant/humble_ws/src/myagv_plus_isaac_description/urdf/myagv_plus_isaac_rollers/myagv_plus_isaac_rollers_noenv.usda"
OUT_DIR = "/home/elephant/isaacsim/scenes"
ENV_PRIM = "/Environment"

# 每行一个场景: 名字 / 环境相对路径 / 环境平移偏移(挪环境让空过道对准原点处的小车) / 环境绕 Z 旋转(度)
# offset 是一次性目视标定, 不受 rollers/结构改动影响; 首轮先用 0, 开生成的场景看小车位置再回填。
SCENES = [
    {"name": "warehouse",        "env": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",                 "offset": (0.0, 0.0, 0.0), "yaw": 0.0},
    {"name": "multiple_shelves", "env": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd", "offset": (0.0, 0.0, 0.0), "yaw": 0.0},
    {"name": "full_warehouse",   "env": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",            "offset": (0.0, 0.0, 0.0), "yaw": 0.0},
    {"name": "hospital",         "env": "/Isaac/Environments/Hospital/hospital.usd",                          "offset": (0.0, 0.0, 0.0), "yaw": 0.0},
    {"name": "office",           "env": "/Isaac/Environments/Office/office.usd",                              "offset": (0.0, 0.0, 0.0), "yaw": 0.0},
]

root = get_assets_root_path()
if root is None:
    raise RuntimeError("assets root = None (Isaac asset root not set / offline)")

if not os.path.isfile(BASE_USD):
    raise FileNotFoundError(f"BASE_USD not found, set it to your no-env base path: {BASE_USD}")

ctx = omni.usd.get_context()
base_dir = os.path.dirname(BASE_USD)
os.makedirs(OUT_DIR, exist_ok=True)


def run_async(coro):
    task = asyncio.ensure_future(coro)
    while not task.done():
        simulation_app.update()
    return task.result()


def make_progress(name):
    state = {"last": -1}

    def cb(cur, total):
        if not total:
            return
        pct = int(cur * 100 / total)
        if pct != state["last"] and pct % 10 == 0:
            state["last"] = pct
            print(f"[{name}] collecting {cur}/{total} ({pct}%)", flush=True)

    return cb


def attach_env(stage, sc):
    env = stage.DefinePrim(ENV_PRIM, "Xform")
    xf = UsdGeom.Xformable(env)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*sc["offset"]))
    xf.AddRotateZOp().Set(sc["yaw"])
    env.GetReferences().ClearReferences()
    env.GetReferences().AddReference(root + sc["env"])


for sc in SCENES:
    if args.only and sc["name"] != args.only:
        continue

    ctx.open_stage(BASE_USD)
    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError(f"failed to open BASE_USD (corrupt or unreadable): {BASE_USD}")
    attach_env(stage, sc)

    # 暂存到 base 同目录, 保证机器人 payload 相对路径仍可解析, 再交给 Collect 本地化
    staging = os.path.join(base_dir, f"myagv_plus_{sc['name']}.usda")
    stage.GetRootLayer().Export(staging)

    print(f"[{sc['name']}] collecting -> {OUT_DIR}", flush=True)
    collector = omni.kit.usd.collect.Collector(usd_path=staging, collect_dir=OUT_DIR, skip_existing=False)
    ok, collected = run_async(collector.collect(progress_callback=make_progress(sc["name"])))
    print(f"[{sc['name']}] collect ok={ok} -> {collected}", flush=True)

    if os.path.exists(staging):
        os.remove(staging)

simulation_app.close()
