# ros2_control 路线链路速率统计。在 ROS2 终端跑, 不是 Isaac Script Editor:
#   source /opt/ros/humble/setup.bash && python3 rate_stats.py [秒数]
#
# 量三件事:
#   /clock                仿真时间相对墙上时间的推进倍率(RTF)与步长分布
#   /isaac_joint_commands ros2_control -> Isaac 的指令下发速率
#   /isaac_joint_states   Isaac -> ros2_control 的反馈速率
#   /joint_states         joint_state_broadcaster 的反馈速率

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


class Rates(Node):
    def __init__(self):
        super().__init__("rate_stats")
        self.clock = []
        self.cmd = []
        self.state = []
        self.isaac = []
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(JointState, "/isaac_joint_commands", self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(JointState, "/isaac_joint_states", self.on_isaac, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)

    def on_clock(self, m):
        self.clock.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9))

    def on_cmd(self, m):
        self.cmd.append((time.time(), len(m.name), list(m.velocity)))

    def on_isaac(self, m):
        self.isaac.append((time.time(), len(m.name)))

    def on_state(self, m):
        self.state.append((time.time(), len(m.name)))


def wall_rate(stamps):
    if len(stamps) < 2:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def main():
    rclpy.init()
    n = Rates()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=0.1)

    print("sampled %.1f s" % DURATION)
    print("")

    if len(n.clock) < 3:
        print("/clock  got %d msgs -- Isaac not playing or domain mismatch" % len(n.clock))
    else:
        walls = [c[0] for c in n.clock]
        sims = [c[1] for c in n.clock]
        dw = walls[-1] - walls[0]
        ds = sims[-1] - sims[0]
        steps = [sims[i + 1] - sims[i] for i in range(len(sims) - 1)]
        zero = sum(1 for s in steps if s <= 1e-9)
        nz = sorted(s for s in steps if s > 1e-9)
        print("/clock")
        print("  wall rate      %.1f Hz  (%d msgs)" % (wall_rate(walls), len(n.clock)))
        print("  sim time advanced %.3f s / wall %.3f s  -> RTF %.3f" % (ds, dw, ds / dw if dw else 0))
        print("  zero-step msgs %d (%.0f%%)  <- only non-zero steps advance time" %
              (zero, 100.0 * zero / max(len(steps), 1)))
        if nz:
            print("  non-zero step  med %.4f s  min %.4f  max %.4f  -> effective %.1f Hz" %
                  (nz[len(nz) // 2], nz[0], nz[-1], 1.0 / nz[len(nz) // 2]))

    print("")
    for name, buf in (("/isaac_joint_commands", n.cmd),
                      ("/isaac_joint_states", n.isaac),
                      ("/joint_states", n.state)):
        if not buf:
            print("%-22s no data" % name)
            continue
        print("%-22s %.1f Hz  (%d msgs, %d joints)" %
              (name, wall_rate([b[0] for b in buf]), len(buf), buf[-1][1]))

    if n.cmd and n.cmd[-1][2]:
        print("")
        print("last wheel command (order matches joint name):")
        print("  " + "  ".join("%+.3f" % v for v in n.cmd[-1][2]))

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
