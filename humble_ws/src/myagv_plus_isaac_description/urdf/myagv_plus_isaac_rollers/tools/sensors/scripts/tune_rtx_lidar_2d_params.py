"""Shared RTX Lidar 2D parameters and open-scene tuning helper.

Usage in Isaac Sim:
1. Create an RTX Lidar 2D prim at the default myAGV Plus lidar path, or pass --lidar-prim.
2. Open and run this file from Window -> Script Editor.
"""

import argparse

import omni.usd

DEFAULT_LIDAR_PRIM = "/myAGV_plus/Geometry/base_footprint/base_link/laser_link/rtx_lidar"

FRAME_ID = "laser_link"
SCAN_TOPIC = "scan"

# Match the real X2 driver config in ydlidar_ros2_driver/params/X2.yaml.
NEAR_RANGE_M = 0.1
FAR_RANGE_M = 12.0
SCAN_RATE_HZ = 10.0

# Keep the sensor itself full 360 deg.  The real driver's ignore_array can be
# reproduced later with a ROS-side LaserScan filter if required.
VALID_START_AZIMUTH_DEG = 0.0
VALID_END_AZIMUTH_DEG = 360.0
MIN_AZIMUTH_ROI_DEG = 0.0
MAX_AZIMUTH_ROI_DEG = 360.0

HORIZONTAL_FOV_DEG = 360.0
# Points per 360 deg scan. Real YDLidar X2 = 300 (X2.yaml: 3 kHz / 10 Hz).
# Read the current sim scan length with: ros2 topic echo /scan --once --no-arr  (ranges length)
POINTS_PER_REV = 300


def lidar_attributes(scan_rate_hz=SCAN_RATE_HZ, points_per_rev=POINTS_PER_REV):
    """Return USD attributes needed for a valid 2D RTX Lidar LaserScan."""
    return {
        "omni:sensor:Core:nearRangeM": NEAR_RANGE_M,
        "omni:sensor:Core:farRangeM": FAR_RANGE_M,
        "omni:sensor:Core:scanRateBaseHz": float(scan_rate_hz),
        "omni:sensor:tickRate": float(scan_rate_hz),
        # points/rev = reportRateBaseHz / scanRateBaseHz; match real X2 (~300)
        "omni:sensor:Core:reportRateBaseHz": float(points_per_rev * scan_rate_hz),
        # Example_Rotary_2D can load with 400/400 ROI sentinel values in GUI.
        # LaserScan needs a positive azimuth span, so force full 360 deg ROI.
        "omni:sensor:Core:minAzimuthROI": MIN_AZIMUTH_ROI_DEG,
        "omni:sensor:Core:maxAzimuthROI": MAX_AZIMUTH_ROI_DEG,
        "omni:sensor:Core:validStartAzimuthDeg": VALID_START_AZIMUTH_DEG,
        "omni:sensor:Core:validEndAzimuthDeg": VALID_END_AZIMUTH_DEG,
    }


def apply_lidar_attributes(prim, scan_rate_hz=SCAN_RATE_HZ, points_per_rev=POINTS_PER_REV):
    """Set the shared RTX Lidar 2D attributes on an existing USD prim."""
    applied = {}
    missing = []
    for name, value in lidar_attributes(scan_rate_hz, points_per_rev).items():
        attr = prim.GetAttribute(name)
        if not attr or not attr.IsValid():
            missing.append(name)
            continue
        attr.Set(value)
        applied[name] = attr.Get()
    return applied, missing


def summarize_lidar_prim(prim):
    """Return relevant RTX Lidar attributes for logs and troubleshooting."""
    names = [
        "omni:sensor:Core:nearRangeM",
        "omni:sensor:Core:farRangeM",
        "omni:sensor:Core:scanRateBaseHz",
        "omni:sensor:Core:reportRateBaseHz",
        "omni:sensor:tickRate",
        "omni:sensor:Core:minAzimuthROI",
        "omni:sensor:Core:maxAzimuthROI",
        "omni:sensor:Core:validStartAzimuthDeg",
        "omni:sensor:Core:validEndAzimuthDeg",
    ]
    summary = []
    for name in names:
        attr = prim.GetAttribute(name)
        summary.append((name, attr.Get() if attr and attr.IsValid() else None))

    for attr in prim.GetAttributes():
        name = attr.GetName()
        if "elevationDeg" in name:
            values = attr.Get()
            if values:
                summary.append((f"{name}.count", len(values)))
                summary.append((f"{name}.min", min(values)))
                summary.append((f"{name}.max", max(values)))
            else:
                summary.append((name, values))
    return summary


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-prim", default=DEFAULT_LIDAR_PRIM)
    parser.add_argument("--scan-rate", type=float, default=SCAN_RATE_HZ)
    parser.add_argument("--points", type=int, default=POINTS_PER_REV, help="Points per 360 deg scan (real X2 = 300)")
    parser.add_argument("--inspect-only", action="store_true")
    args, _unknown = parser.parse_known_args()
    return args


def _get_lidar_prim(stage, lidar_prim_path):
    if lidar_prim_path:
        prim = stage.GetPrimAtPath(lidar_prim_path)
        if prim.IsValid():
            return prim
        if lidar_prim_path != DEFAULT_LIDAR_PRIM:
            raise RuntimeError(f"RTX Lidar prim not found: {lidar_prim_path}")

    selection = omni.usd.get_context().get_selection().get_selected_prim_paths()
    if not selection:
        raise RuntimeError(
            f"RTX Lidar prim not found at default path: {DEFAULT_LIDAR_PRIM}. "
            "Select the RTX Lidar prim in Stage or pass --lidar-prim."
        )

    prim = stage.GetPrimAtPath(selection[0])
    if not prim.IsValid():
        raise RuntimeError(f"Selected prim is invalid: {selection[0]}")
    return prim


def main():
    args = _parse_args()
    stage = omni.usd.get_context().get_stage()
    prim = _get_lidar_prim(stage, args.lidar_prim)

    print(f"[LIDAR] {prim.GetPath()}")
    print("[BEFORE]")
    for name, value in summarize_lidar_prim(prim):
        print(f"  {name} = {value}")

    if not args.inspect_only:
        applied, missing = apply_lidar_attributes(prim, scan_rate_hz=args.scan_rate, points_per_rev=args.points)
        print("[APPLIED]")
        for name, value in applied.items():
            print(f"  {name} = {value}")
        for name in missing:
            print(f"  [MISSING] {name}")

    print("[AFTER]")
    for name, value in summarize_lidar_prim(prim):
        print(f"  {name} = {value}")


if __name__ == "__main__":
    main()
