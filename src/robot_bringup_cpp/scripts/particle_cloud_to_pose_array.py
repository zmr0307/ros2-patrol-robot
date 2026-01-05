#!/usr/bin/env python3
import argparse
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray
from nav2_msgs.msg import ParticleCloud


class ParticleCloudToPoseArray(Node):
    def __init__(self, input_topic: str, output_topic: str, frame_id_override: str | None):
        super().__init__('particle_cloud_to_pose_array')
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.frame_id_override = frame_id_override

        # Match AMCL's publisher QoS (BEST_EFFORT + VOLATILE)
        particle_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.subscription = self.create_subscription(ParticleCloud, self.input_topic, self._on_particle_cloud, particle_qos)

        self.publisher = self.create_publisher(PoseArray, self.output_topic, 10)
        self.get_logger().info(
            f"Converting '{self.input_topic}' (ParticleCloud) -> '{self.output_topic}' (PoseArray)"
        )

    def _on_particle_cloud(self, msg: ParticleCloud) -> None:
        pose_array = PoseArray()
        pose_array.header = msg.header
        if self.frame_id_override:
            pose_array.header.frame_id = self.frame_id_override

        particles: Iterable = getattr(msg, 'particles', [])
        if not particles:
            return

        for p in particles:
            pose: Pose = getattr(p, 'pose', p)
            pose_array.poses.append(pose)

        self.publisher.publish(pose_array)


def main():
    parser = argparse.ArgumentParser(description='Convert AMCL ParticleCloud to PoseArray for RViz display')
    parser.add_argument('--in', dest='input_topic', default='/particle_cloud', help='Input ParticleCloud topic')
    parser.add_argument('--out', dest='output_topic', default='/amcl/particles', help='Output PoseArray topic')
    parser.add_argument('--frame', dest='frame_id', default=None, help='Override frame_id for PoseArray header')
    args, unknown = parser.parse_known_args()  # ✅ 忽略ROS2自动添加的--ros-args参数

    rclpy.init()
    node = ParticleCloudToPoseArray(args.input_topic, args.output_topic, args.frame_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


