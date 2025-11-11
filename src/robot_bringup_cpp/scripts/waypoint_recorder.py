#!/usr/bin/env python3
"""
Waypoint recorder:

Listens to RViz `/goal_pose` (PoseStamped) and `/clicked_point` (PointStamped)
to build patrol waypoint YAML. Provides simple CLI commands via stdin:
 - save [path]
 - load [path]
 - clear
 - undo
 - list
"""

from __future__ import annotations

import math
import sys
import threading
from pathlib import Path
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
import yaml


class WaypointRecorder(Node):

    def __init__(self) -> None:
        super().__init__("waypoint_recorder")
        try:
            pkg_share = Path(get_package_share_directory("robot_bringup_cpp"))
        except Exception:  # fallback when sourcing from workspace
            pkg_share = Path(__file__).resolve().parents[1]
        default_output = pkg_share / "config" / "patrol_waypoints.yaml"

        self.declare_parameter("output", str(default_output))
        self.declare_parameter("default_wait_s", 5.0)
        self.declare_parameter("enable_goal_pose", True)
        self.declare_parameter("enable_clicked_point", True)
        self.declare_parameter("default_yaw_deg", 0.0)

        self.output_path = Path(self.get_parameter("output").value)
        self.default_wait = float(self.get_parameter("default_wait_s").value)
        self.default_yaw = float(self.get_parameter("default_yaw_deg").value)

        self.waypoints: List[dict] = []
        if self.get_parameter("enable_goal_pose").value:
            self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        if self.get_parameter("enable_clicked_point").value:
            self.create_subscription(PointStamped, "/clicked_point", self.clicked_cb, 10)

        self._cli_thread = threading.Thread(target=self._cli_loop, daemon=True)
        self._cli_thread.start()
        self.get_logger().info(
            "Waypoint recorder running. Type commands (save/load/clear/undo/list) + Enter.")

    def goal_cb(self, msg: PoseStamped) -> None:
        yaw = self._quat_to_yaw_deg(msg.pose.orientation)
        self._add_waypoint(msg.pose.position.x, msg.pose.position.y, yaw)

    def clicked_cb(self, msg: PointStamped) -> None:
        self._add_waypoint(msg.point.x, msg.point.y, self.default_yaw)

    def _add_waypoint(self, x: float, y: float, yaw_deg: float) -> None:
        entry = {
            "x": float(x),
            "y": float(y),
            "yaw_deg": float(yaw_deg),
            "wait_s": self.default_wait,
        }
        self.waypoints.append(entry)
        self.get_logger().info(
            f"Added waypoint #{len(self.waypoints)}: "
            f"x={entry['x']:.3f}, y={entry['y']:.3f}, yaw={entry['yaw_deg']:.1f}°")

    @staticmethod
    def _quat_to_yaw_deg(orientation) -> float:
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def _cli_loop(self) -> None:
        while rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                break
            parts = line.strip().split()
            if not parts:
                continue
            cmd = parts[0].lower()
            args = parts[1:]
            if cmd == "save":
                path = Path(args[0]) if args else self.output_path
                self._save(path)
            elif cmd == "load":
                if not args:
                    self.get_logger().warn("Usage: load <path>")
                    continue
                self._load(Path(args[0]))
            elif cmd == "clear":
                self.waypoints.clear()
                self.get_logger().info("Cleared waypoints")
            elif cmd == "undo":
                if self.waypoints:
                    removed = self.waypoints.pop()
                    self.get_logger().info(
                        f"Removed waypoint: x={removed['x']:.3f}, y={removed['y']:.3f}")
                else:
                    self.get_logger().warn("No waypoints to undo")
            elif cmd == "list":
                if not self.waypoints:
                    self.get_logger().info("No waypoints recorded yet")
                for idx, wp in enumerate(self.waypoints, start=1):
                    self.get_logger().info(
                        f"{idx}: x={wp['x']:.3f}, y={wp['y']:.3f}, "
                        f"yaw={wp['yaw_deg']:.1f}°, wait={wp['wait_s']:.1f}s")
            else:
                self.get_logger().warn(f"Unknown command: {cmd}")

    def _save(self, path: Path) -> None:
        if not self.waypoints:
            self.get_logger().warn("No waypoints to save")
            return
        data = {"waypoints": self.waypoints}
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {path}")

    def _load(self, path: Path) -> None:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f"Failed to load {path}: {exc}")
            return
        self.waypoints = data.get("waypoints", [])
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {path}")


def main() -> None:
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
