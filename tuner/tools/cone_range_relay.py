"""
Range-limiting relay for fused cone detections, used when replaying a rosbag into the
SLAM node (`rosbag_slam_replay.sh`).

Republishes `/cone_detection_fused_raw` (the bag's topic, remapped on playback) as
`/cone_detection_fused`, dropping every cone farther than `max_range` from the camera.

Why: bags recorded on `fsae_autonomous`'s `test/planning-teleop-ground-test` branch put
camera-only cones beyond 8 m on the fused topic (`camera_fallback_range`), and camera range
error grows with the square of distance. `cone_landmark_mapper` accepts anything up to
25 m, so without this filter those far, poorly ranged cones enter the map.

Needs ROS 2 and a built `fsae_interfaces` on the path (source the workspace that also
holds `fsae_slam`).

    python3 tuner/tools/cone_range_relay.py --ros-args -p max_range:=7.0 -p use_sim_time:=true
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fsae_interfaces.msg import ConeDetection

FIELDS = ('yellow', 'blue', 'small_orange', 'big_orange')


class ConeRangeRelay(Node):
    def __init__(self):
        super().__init__('cone_range_relay')
        self.max_range = self.declare_parameter('max_range', 7.0).value
        # RELIABLE: cone_landmark_mapper subscribes reliable, as it does to lidar_fusion.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(ConeDetection, '/cone_detection_fused', qos)
        self.create_subscription(ConeDetection, '/cone_detection_fused_raw', self._cb, qos)
        self.kept = self.dropped = 0

    def _cb(self, msg):
        for f in FIELDS:
            pts = getattr(msg, f)
            keep = [p for p in pts if math.hypot(p.x, p.y) <= self.max_range]
            self.kept += len(keep)
            self.dropped += len(pts) - len(keep)
            setattr(msg, f, keep)
        self.pub.publish(msg)
        self.get_logger().info(
            f'cones kept {self.kept}, dropped beyond {self.max_range} m {self.dropped}',
            throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = ConeRangeRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
