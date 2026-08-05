# 从 ros2_lidar 场景生成 ros2_control 路线的两个 usda(建图代码只写一份):
#   sim.usda   = ros2_lidar + ros2_control 图              §15 自包含验证场景(地面+靶标+雷达)
#   noenv.usda = sim 去掉 /GroundPlane /LidarTargets /DomeLight   §16 批量派生的母版
#
# ros2_control 图由引擎(og.Controller)生成, schema 保证正确, 不手写 usda。
#
# 用法: <isaac>/python.sh make_sim_scenes.py          (headless)
#       加 --gui 可视化观察
#   改了 rollers / 图 / 雷达后重跑本脚本, 两个 usda 一起重生成。

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not args.gui})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
for _ in range(20):
    simulation_app.update()

import omni.usd
import omni.graph.core as og
from pxr import Sdf

HERE = os.path.dirname(os.path.abspath(__file__))
ROLLERS_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_USD = os.path.join(ROLLERS_DIR, "myagv_plus_isaac_rollers_ros2_lidar.usda")
SIM_USD = os.path.join(ROLLERS_DIR, "myagv_plus_isaac_rollers_sim.usda")
NOENV_USD = os.path.join(ROLLERS_DIR, "myagv_plus_isaac_rollers_noenv.usda")

BASE = "/myAGV_plus/Geometry/base_footprint/base_link"
CMD_TOPIC = "isaac_joint_commands"
STATE_TOPIC = "isaac_joint_states"
ENV_PRIMS = ("/GroundPlane", "/LidarTargets", "/DomeLight")

if not os.path.isfile(SRC_USD):
    raise FileNotFoundError("source scene not found: " + SRC_USD)

ctx = omni.usd.get_context()
ctx.open_stage(SRC_USD)
for _ in range(10):
    simulation_app.update()
stage = ctx.get_stage()

for p in ("/Graph/ROS_Control", "/Graph/ROS_RobotState"):
    if stage.GetPrimAtPath(p):
        stage.RemovePrim(p)
        print("removed old graph", p)

K = og.Controller.Keys

og.Controller.edit(
    {"graph_path": "/Graph/ROS_Control", "evaluator_name": "execution"},
    {
        K.CREATE_NODES: [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("subscribe_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        K.CONNECT: [
            ("on_playback_tick.outputs:tick", "subscribe_joint_state.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
            ("ros2_context.outputs:context", "subscribe_joint_state.inputs:context"),
            ("subscribe_joint_state.outputs:jointNames", "articulation_controller.inputs:jointNames"),
            ("subscribe_joint_state.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
        ],
        K.SET_VALUES: [
            ("ros2_context.inputs:useDomainIDEnvVar", True),
            ("subscribe_joint_state.inputs:topicName", CMD_TOPIC),
        ],
    },
)
print("built /Graph/ROS_Control")

og.Controller.edit(
    {"graph_path": "/Graph/ROS_RobotState", "evaluator_name": "execution"},
    {
        K.CREATE_NODES: [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("read_sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("publish_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        K.CONNECT: [
            ("on_playback_tick.outputs:tick", "publish_clock.inputs:execIn"),
            ("on_playback_tick.outputs:tick", "publish_joint_state.inputs:execIn"),
            ("ros2_context.outputs:context", "publish_clock.inputs:context"),
            ("ros2_context.outputs:context", "publish_joint_state.inputs:context"),
            ("read_sim_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
            ("read_sim_time.outputs:simulationTime", "publish_joint_state.inputs:timeStamp"),
        ],
        K.SET_VALUES: [
            ("ros2_context.inputs:useDomainIDEnvVar", True),
            ("read_sim_time.inputs:resetOnStop", False),
            ("publish_clock.inputs:topicName", "clock"),
            ("publish_joint_state.inputs:topicName", STATE_TOPIC),
        ],
    },
)
print("built /Graph/ROS_RobotState")

for path in ("/Graph/ROS_Control/articulation_controller",
             "/Graph/ROS_RobotState/publish_joint_state"):
    prim = stage.GetPrimAtPath(path)
    rel = prim.GetRelationship("inputs:targetPrim")
    if not rel:
        rel = prim.CreateRelationship("inputs:targetPrim")
    rel.SetTargets([Sdf.Path(BASE)])
    print("targetPrim", path, "->", BASE)

graphs = sorted(g.get_path_to_graph() for g in og.get_all_graphs()
                if g.get_path_to_graph().startswith("/Graph"))
print("final graphs:", graphs)

stage.GetRootLayer().Export(SIM_USD)
print("exported sim   ->", SIM_USD)

for p in ENV_PRIMS:
    if stage.GetPrimAtPath(p):
        stage.RemovePrim(p)
        print("noenv removed", p)
stage.GetRootLayer().Export(NOENV_USD)
print("exported noenv ->", NOENV_USD)

simulation_app.close()
