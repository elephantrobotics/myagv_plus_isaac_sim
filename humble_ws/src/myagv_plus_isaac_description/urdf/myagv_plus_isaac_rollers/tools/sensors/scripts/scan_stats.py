# /scan 统计。在 ROS2 终端跑, 不是 Isaac Script Editor:
#   source /opt/ros/humble/setup.bash && python3 scan_stats.py
#
# 输出每个方位的有效/无效分布, 用来判断是遮挡还是链路问题。

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

BINS = 24


class Stats(Node):
    def __init__(self):
        super().__init__("scan_stats")
        self.done = False
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        if self.done:
            return
        self.done = True
        n = len(m.ranges)
        print("points          %d" % n)
        print("range_min/max   %.3f / %.3f" % (m.range_min, m.range_max))
        print("angle_min/max   %.2f / %.2f deg" % (math.degrees(m.angle_min),
                                                   math.degrees(m.angle_max)))
        print("angle_increment %.4f deg" % math.degrees(m.angle_increment))
        good = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
        print("")
        print("valid           %d / %d  (%.1f%%)" % (len(good), n, 100.0 * len(good) / max(n, 1)))
        if good:
            g = sorted(good)
            print("range min/med/max  %.3f / %.3f / %.3f m" % (g[0], g[len(g) // 2], g[-1]))
        print("")
        print("by azimuth (lidar frame, 0=front):")
        per = max(1, n // BINS)
        for b in range(BINS):
            seg = m.ranges[b * per:(b + 1) * per]
            if not seg:
                continue
            ok = [r for r in seg if math.isfinite(r) and m.range_min <= r <= m.range_max]
            a0 = math.degrees(m.angle_min + b * per * m.angle_increment)
            bar = "#" * int(round(20.0 * len(ok) / len(seg)))
            d = ("%.2f" % (sum(ok) / len(ok))) if ok else "  -  "
            print("  %+7.1f deg  %3d/%3d  mean %s m  %s" % (a0, len(ok), len(seg), d, bar))


def main():
    rclpy.init()
    node = Stats()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
