# Minimal Isaac Sim 6.0.1 RTX Lidar 2D -> ROS2 LaserScan test
# Does NOT use the GUI "ROS2 RTX Lidar Helper" OmniGraph node.
# Run with: ~/isaacsim/python.sh rtx_lidar_2d_laserscan.py

from isaacsim import SimulationApp

# Official RTX Lidar docs recommend single-GPU if RTX Lidar triggers CUDA issues.
simulation_app = SimulationApp({"headless": False, "multi_gpu": False})

import argparse
import math
import os

import omni.usd
import omni.timeline
import omni.replicator.core as rep
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom, UsdLux


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="scan")
    parser.add_argument("--frame-id", default="laser_link")
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--no-targets", action="store_true")
    parser.add_argument("--horizontal-fov", type=float, default=360.0)
    parser.add_argument("--horizontal-resolution", type=float, default=360.0 / POINTS_PER_REV)
    return parser.parse_args()


args = parse_args()

NEAR_RANGE_M = 0.1
FAR_RANGE_M = 12.0
SCAN_RATE_HZ = 10.0
POINTS_PER_REV = 300


def lidar_attributes():
    return {
        "omni:sensor:Core:nearRangeM": NEAR_RANGE_M,
        "omni:sensor:Core:farRangeM": FAR_RANGE_M,
        "omni:sensor:Core:scanRateBaseHz": SCAN_RATE_HZ,
        "omni:sensor:tickRate": SCAN_RATE_HZ,
        "omni:sensor:Core:reportRateBaseHz": float(POINTS_PER_REV * SCAN_RATE_HZ),
        "omni:sensor:Core:minAzimuthROI": 0.0,
        "omni:sensor:Core:maxAzimuthROI": 360.0,
        "omni:sensor:Core:validStartAzimuthDeg": 0.0,
        "omni:sensor:Core:validEndAzimuthDeg": 360.0,
    }


def apply_lidar_attributes(prim):
    for name, value in lidar_attributes().items():
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            attr.Set(value)

print("[INFO] ROS_DISTRO=", os.environ.get("ROS_DISTRO"))
print("[INFO] RMW_IMPLEMENTATION=", os.environ.get("RMW_IMPLEMENTATION"))
print("[INFO] Enabling extensions...")
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
enable_extension("omni.replicator.core")

# Let extensions finish loading.
for _ in range(20):
    simulation_app.update()

from isaacsim.sensors.experimental.rtx import Lidar

# Start from a clean empty stage.
omni.usd.get_context().new_stage()
for _ in range(5):
    simulation_app.update()
stage = omni.usd.get_context().get_stage()

# Simple light.
light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
light.CreateIntensityAttr(500.0)

# Optional simple boxes so LaserScan has returns.
def add_cube(path, translate, scale, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([color])
    xform = UsdGeom.XformCommonAPI(cube.GetPrim())
    xform.SetTranslate(Gf.Vec3d(*translate))
    xform.SetScale(Gf.Vec3f(*scale))

if not args.no_targets:
    add_cube("/World/front_wall", (2.0, 0.0, 0.5), (0.05, 1.5, 0.5), (0.2, 0.6, 1.0))
    add_cube("/World/left_box", (0.8, 1.0, 0.25), (0.25, 0.25, 0.25), (1.0, 0.4, 0.2))
    add_cube("/World/right_box", (0.8, -1.0, 0.25), (0.25, 0.25, 0.25), (0.2, 1.0, 0.4))

print("[INFO] Creating Example_Rotary_2D RTX Lidar...")
lidar_2d = Lidar.create(
    path="/World/sensor_2D",
    config="Example_Rotary_2D",
    tick_rate=SCAN_RATE_HZ,
    translations=[[0.0, 0.0, 0.25]],
    attributes=lidar_attributes(),
)

# Print loaded RTX Lidar attributes.  If horizontalFov or horizontalResolution
# is zero/missing, the ROS LaserScan writer cannot create valid scans.
stage = omni.usd.get_context().get_stage()
lidar_prim_path = lidar_2d.paths[0]
lidar_prim = stage.GetPrimAtPath(lidar_prim_path)
apply_lidar_attributes(lidar_prim)
print(f"[INFO] Lidar prim path: {lidar_prim_path}")
print(f"[INFO] Lidar prim type: {lidar_prim.GetTypeName()}")
print(f"[INFO] Real-car range sync: near={NEAR_RANGE_M}m far={FAR_RANGE_M}m scan_rate={SCAN_RATE_HZ}Hz")
for attr in lidar_prim.GetAttributes():
    name = attr.GetName()
    lower = name.lower()
    if any(k in lower for k in ["fov", "resolution", "scan", "tick", "accumulate", "azimuth", "range", "elevation"]):
        try:
            print(f"[LIDAR ATTR] {name} = {attr.Get()}")
        except Exception as exc:
            print(f"[LIDAR ATTR] {name} = <error: {exc}>")

# Official script route: render product + RtxLidarROS2PublishLaserScan writer.
print("[INFO] Creating render product [1, 1]...")
hydra_texture_2d = rep.create.render_product(lidar_2d.paths[0], [1, 1], name="rtx_lidar_2d")

print("[INFO] Attaching RtxLidarROS2PublishLaserScan writer...")
writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
writer.initialize(
    topicName=args.topic.strip("/") or "scan",
    frameId=args.frame_id,
    # ROS2 Publish Laser Scan requires these to be positive.
    # The RTX writer path is currently reading zeros from Example_Rotary_2D,
    # so pass explicit values for the minimal test.
    horizontalFov=args.horizontal_fov,
    horizontalResolution=args.horizontal_resolution,
)
writer.attach([hydra_texture_2d])

print(f"[INFO] Publishing LaserScan on /{args.topic.strip('/') or 'scan'} frame_id={args.frame_id}")
print("[INFO] In another terminal run:")
print(f"       source /opt/ros/humble/setup.bash && ros2 topic hz /{args.topic.strip('/') or 'scan'}")
print(f"       source /opt/ros/humble/setup.bash && ros2 topic echo /{args.topic.strip('/') or 'scan'} --once")

# Play timeline.
timeline = omni.timeline.get_timeline_interface()
timeline.play()

frames = 0
max_frames = int(max(args.seconds, 1.0) * 60.0)
while simulation_app.is_running() and frames < max_frames:
    simulation_app.update()
    frames += 1
    if frames % 300 == 0:
        print(f"[INFO] running... frame={frames}")

timeline.stop()
simulation_app.close()
