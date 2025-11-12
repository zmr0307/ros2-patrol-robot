#!/usr/bin/env python3
"""
Patrol manager node.

Implements a waypoint-based patrol loop:
 - Loads waypoints (with wait time + optional tasks) from YAML.
 - Sends Nav2 NavigateToPose goals with retry logic.
 - Executes per-waypoint tasks (external commands with timeout & retry).
 - Supports Start/Pause/Resume/Stop services.
 - Publishes status, event logs, and visualization markers.
 - Persists structured logs per run.
"""

from __future__ import annotations

import copy
import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from robot_bringup_cpp.msg import PatrolStatus
from robot_bringup_cpp.srv import SetWaypoints


# Helper dataclasses --------------------------------------------------------

@dataclass
class TaskSpec:
    type: str = "generic"
    cmd: str = ""
    timeout: float = 10.0
    retry: int = 2


@dataclass
class Waypoint:
    x: float
    y: float
    yaw_deg: float
    wait_s: float
    tasks: List[TaskSpec] = field(default_factory=list)


# Patrol Manager ------------------------------------------------------------

class PatrolManager(Node):

    def __init__(self) -> None:
        super().__init__("patrol_manager")
        self._share_dir = Path(get_package_share_directory("robot_bringup_cpp"))

        # Parameters
        self.declare_parameters(
            "",
            [
                ("waypoints_file", str(self._share_dir / "config" / "patrol_waypoints.yaml")),
                ("loop", True),
                ("default_wait_s", 5.0),
                ("task_default_timeout_s", 10.0),
                ("task_default_retry", 2),
                ("status_hz", 1.0),
                ("log_dir", "/home/nvidia/robot_ws/log/patrol"),
                ("log_keep_runs", 10),
            ],
        )

        # Runtime state
        self.waypoints: List[Waypoint] = []
        self.current_index = -1
        self.loop_enabled = bool(self.get_parameter("loop").value)
        self.phase = "idle"
        self.run_id: Optional[str] = None
        self._run_start_time: float = 0.0
        self._nav_goal_handle: Optional[ClientGoalHandle] = None
        self._nav_retry_limit = 2
        self._nav_retries_total = 0
        self._nav_retries_current = 0
        self._task_retries_total = 0
        self._task_retries_current = 0
        self._task_proc: Optional[subprocess.Popen] = None
        self._stop_event = threading.Event()
        self._paused = False
        self._pause_cv = threading.Condition()
        self._runner_thread: Optional[threading.Thread] = None
        self._nav_lock = threading.Lock()
        self._task_lock = threading.Lock()

        self._visited_ok = 0
        self._visited_fail = 0
        self._task_ok = 0
        self._task_fail = 0
        self._current_task = ""
        self._current_goal_id = ""
        self._map_pose = Pose()
        self._odom_pose = Pose()

        # Logging
        self._log_dir = Path(self.get_parameter("log_dir").value).expanduser()
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._run_log_dir: Optional[Path] = None
        self._events_file = None
        self._status_file = None
        self._task_logs_dir: Optional[Path] = None

        # ROS interfaces
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status_pub = self.create_publisher(
            PatrolStatus,
            "/patrol/status",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._events_pub = self.create_publisher(
            String,
            "/patrol/events",
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            "/patrol/markers",
            QoSProfile(depth=10),
        )

        status_period = 1.0 / max(0.1, float(self.get_parameter("status_hz").value))
        self._status_timer = self.create_timer(status_period, self.publish_status)
        self._marker_timer = self.create_timer(1.0, self.publish_markers)
        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.5, self._update_map_pose)

        self._start_srv = self.create_service(Trigger, "/patrol/start", self.handle_start)
        self._pause_srv = self.create_service(Trigger, "/patrol/pause", self.handle_pause)
        self._resume_srv = self.create_service(Trigger, "/patrol/resume", self.handle_resume)
        self._stop_srv = self.create_service(Trigger, "/patrol/stop", self.handle_stop)
        self._set_waypoints_srv = self.create_service(
            SetWaypoints, "/patrol/set_waypoints", self.handle_set_waypoints)

        # Load waypoints
        self.load_waypoints(self.get_parameter("waypoints_file").value)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

    # Waypoint loading -----------------------------------------------------

    def load_waypoints(self, path: str) -> None:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load waypoints {path}: {exc}")
            self.waypoints = []
            return

        waypoints: List[Waypoint] = []
        default_wait = float(self.get_parameter("default_wait_s").value)
        task_def_timeout = float(self.get_parameter("task_default_timeout_s").value)
        task_def_retry = int(self.get_parameter("task_default_retry").value)

        for idx, entry in enumerate(data.get("waypoints", [])):
            try:
                tasks: List[TaskSpec] = []
                for t in entry.get("tasks", []):
                    tasks.append(
                        TaskSpec(
                            type=str(t.get("type", "generic")),
                            cmd=str(t.get("cmd", "")),
                            timeout=float(t.get("timeout", task_def_timeout)),
                            retry=int(t.get("retry", task_def_retry)),
                        )
                    )
                waypoints.append(
                    Waypoint(
                        x=float(entry.get("x", 0.0)),
                        y=float(entry.get("y", 0.0)),
                        yaw_deg=float(entry.get("yaw_deg", 0.0)),
                        wait_s=float(entry.get("wait_s", default_wait)),
                        tasks=tasks,
                    )
                )
            except (TypeError, ValueError) as exc:
                self.get_logger().warning(f"Skip waypoint #{idx}: {exc}")

        self.waypoints = waypoints
        self.publish_markers()

    # Logging utilities ----------------------------------------------------

    def _open_run_logs(self) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{ts}_{uuid.uuid4().hex[:6]}"
        self._run_log_dir = self._log_dir / self.run_id
        self._run_log_dir.mkdir(parents=True, exist_ok=True)
        self._events_file = open(self._run_log_dir / "events.jsonl", "a", encoding="utf-8")
        self._status_file = open(self._run_log_dir / "status.csv", "a", encoding="utf-8")
        if self._status_file.tell() == 0:
            self._status_file.write(
                "timestamp,phase,index,total,visited_ok,visited_fail,task_ok,task_fail\n")
        self._task_logs_dir = self._run_log_dir / "tasks"
        self._task_logs_dir.mkdir(exist_ok=True)

    def _close_run_logs(self) -> None:
        if self._events_file:
            self._events_file.close()
            self._events_file = None
        if self._status_file:
            self._status_file.close()
            self._status_file = None
        self._task_logs_dir = None
        self._rotate_logs()

    def _rotate_logs(self) -> None:
        keep = max(1, int(self.get_parameter("log_keep_runs").value))
        runs = sorted([p for p in self._log_dir.glob("*") if p.is_dir()])
        if len(runs) <= keep:
            return
        for old_run in runs[:-keep]:
            try:
                for sub in old_run.glob("**/*"):
                    if sub.is_file():
                        sub.unlink()
                for sub in sorted(old_run.glob("**/*"), reverse=True):
                    if sub.is_dir():
                        sub.rmdir()
                old_run.rmdir()
            except OSError:
                pass

    def _log_event(self, event: str, severity: str = "INFO", **data: Any) -> None:
        if not self.run_id:
            return
        payload = {
            "ts": datetime.now().isoformat(),
            "event": event,
            "index": self.current_index,
            "run_id": self.run_id,
            "phase": self.phase,
            "severity": severity,
            "retries": data.pop("retries", 0),
            "elapsed_s": time.time() - self._run_start_time if self._run_start_time else 0.0,
            "msg": data.pop("msg", ""),
            "data": data,
        }
        line = json.dumps(payload, ensure_ascii=False)
        self._events_pub.publish(String(data=line))
        if self._events_file:
            self._events_file.write(line + "\n")
            self._events_file.flush()

    # Marker publishing ----------------------------------------------------

    def publish_markers(self) -> None:
        markers = MarkerArray()

        # Waypoint markers
        stamp = self.get_clock().now().to_msg()
        for idx, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "patrol_waypoints"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp.x
            marker.pose.position.y = wp.y
            marker.pose.position.z = 0.05
            q = self._yaw_to_quaternion(math.radians(wp.yaw_deg))
            marker.pose.orientation = q
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            if idx == self.current_index:
                marker.color.r, marker.color.g, marker.color.b = (0.1, 0.8, 0.2)
            else:
                marker.color.r, marker.color.g, marker.color.b = (0.2, 0.4, 0.9)
            marker.color.a = 0.9
            markers.markers.append(marker)

        self._marker_pub.publish(markers)

    # Status publishing ----------------------------------------------------

    def publish_status(self) -> None:
        msg = PatrolStatus()
        msg.run_id = self.run_id or ""
        msg.loop = self.loop_enabled
        msg.phase = self.phase
        msg.current_index = self.current_index
        msg.total = len(self.waypoints)
        msg.retries_nav = self._nav_retries_total
        msg.retries_task = self._task_retries_total
        msg.visited_ok = self._visited_ok
        msg.visited_fail = self._visited_fail
        msg.task_ok = self._task_ok
        msg.task_fail = self._task_fail
        msg.elapsed_s = time.time() - self._run_start_time if self._run_start_time else 0.0
        msg.eta_s = 0.0  # TODO: compute remaining ETA
        msg.current_task_name = self._current_task or ""
        msg.remaining_total_eta_s = 0.0
        msg.current_goal_id = self._current_goal_id or ""
        msg.retries_nav_current = self._nav_retries_current
        msg.retries_task_current = self._task_retries_current
        msg.map_pose = self._map_pose
        msg.odom_pose = self._odom_pose
        self._status_pub.publish(msg)

        if self._status_file and self.run_id:
            self._status_file.write(
                f"{datetime.now().isoformat()},{self.phase},{self.current_index},"
                f"{len(self.waypoints)},{self._visited_ok},{self._visited_fail},"
                f"{self._task_ok},{self._task_fail}\n"
            )
            self._status_file.flush()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_pose = copy.deepcopy(msg.pose.pose)

    def _update_map_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        self._map_pose = pose

    # Service handlers -----------------------------------------------------

    def handle_start(self, _request, response):
        if self._runner_thread and self._runner_thread.is_alive():
            response.success = False
            response.message = "Patrol already running"
            return response
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints loaded"
            return response
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = "NavigateToPose action server not ready"
            return response

        self._stop_event.clear()
        self._paused = False
        self.current_index = -1
        self._visited_ok = self._visited_fail = 0
        self._task_ok = self._task_fail = 0
        self._nav_retries_total = 0
        self._task_retries_total = 0
        self._nav_retries_current = 0
        self._task_retries_current = 0
        self._current_task = ""
        self._current_goal_id = ""
        self.phase = "navigating"
        self._run_start_time = time.time()
        self._open_run_logs()
        self._log_event("RUN_START")

        self._runner_thread = threading.Thread(target=self._patrol_loop, daemon=True)
        self._runner_thread.start()

        response.success = True
        response.message = "Patrol started"
        return response

    def handle_pause(self, _request, response):
        if not self._runner_thread or not self._runner_thread.is_alive():
            response.success = False
            response.message = "Patrol not running"
            return response
        if self._paused:
            response.success = True
            response.message = "Already paused"
            return response
        self._paused = True
        self.phase = "paused"
        # Cancel nav and terminate task (with timeouts to prevent hanging)
        try:
            self._cancel_nav_goal("pause")
        except Exception as e:
            self.get_logger().error(f"Error canceling nav goal: {e}")
        try:
            self._terminate_task_proc()
        except Exception as e:
            self.get_logger().error(f"Error terminating task: {e}")
        response.success = True
        response.message = "Paused patrol"
        return response

    def handle_resume(self, _request, response):
        if not self._runner_thread or not self._runner_thread.is_alive():
            response.success = False
            response.message = "Patrol not running"
            return response
        if not self._paused:
            response.success = True
            response.message = "Already running"
            return response
        with self._pause_cv:
            self._paused = False
            self._pause_cv.notify_all()
        self.phase = "navigating"
        response.success = True
        response.message = "Resumed patrol"
        return response

    def handle_stop(self, _request, response):
        if not self._runner_thread or not self._runner_thread.is_alive():
            response.success = False
            response.message = "Patrol not running"
            return response
        self._stop_event.set()
        with self._pause_cv:
            self._paused = False
            self._pause_cv.notify_all()
        # Cancel nav and terminate task (with timeouts to prevent hanging)
        try:
            self._cancel_nav_goal("stop")
        except Exception as e:
            self.get_logger().error(f"Error canceling nav goal: {e}")
        try:
            self._terminate_task_proc()
        except Exception as e:
            self.get_logger().error(f"Error terminating task: {e}")
        # Wait for thread to finish, but don't block forever
        if self._runner_thread:
            self._runner_thread.join(timeout=5.0)
            if self._runner_thread.is_alive():
                self.get_logger().warning("Runner thread did not finish within timeout")
        self.phase = "stopped"
        response.success = True
        response.message = "Patrol stopped"
        return response

    def handle_set_waypoints(self, request: SetWaypoints.Request,
                             response: SetWaypoints.Response):
        if request.yaml_path:
            self.load_waypoints(request.yaml_path)
            response.success = True
            response.message = f"Loaded waypoints from {request.yaml_path}"
            return response
        if request.yaml_inline:
            try:
                data = yaml.safe_load(request.yaml_inline)
            except yaml.YAMLError as exc:
                response.success = False
                response.message = f"Inline YAML invalid: {exc}"
                return response
            tmp_file = self._log_dir / "inline_waypoints.yaml"
            with open(tmp_file, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f)
            self.load_waypoints(str(tmp_file))
            response.success = True
            response.message = "Loaded waypoints from inline YAML"
            return response
        response.success = False
        response.message = "Provide yaml_path or yaml_inline"
        return response

    # Patrol loop ----------------------------------------------------------

    def _patrol_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                for idx, waypoint in enumerate(self.waypoints):
                    self.current_index = idx
                    self._nav_retries_current = 0
                    self._task_retries_current = 0
                    self._current_task = ""
                    self.phase = "navigating"
                    self.publish_markers()
                    self._log_event("GOAL_SENT", waypoint=vars(waypoint))

                    nav_success = self._navigate_with_retry(waypoint)
                    if self._stop_event.is_set():
                        break
                    if not nav_success:
                        self._visited_fail += 1
                        self._log_event("GOAL_FAILED", severity="WARN")
                        continue

                    self._visited_ok += 1
                    self.phase = "waiting"
                    if not self._wait_with_pause(waypoint.wait_s):
                        break

                    # Execute tasks
                    for task_idx, task in enumerate(waypoint.tasks):
                        if self._stop_event.is_set():
                            break
                        self.phase = "task"
                        task_success = self._execute_task(task, task_idx)
                        if task_success:
                            self._task_ok += 1
                        else:
                            self._task_fail += 1
                    self._current_task = ""
                    if self._stop_event.is_set():
                        break
                else:
                    if self.loop_enabled and not self._stop_event.is_set():
                        continue
                break
        finally:
            self.phase = "stopped" if self._stop_event.is_set() else "idle"
            self._log_event("RUN_END")
            self._close_run_logs()
            self.run_id = None

    def _navigate_with_retry(self, waypoint: Waypoint) -> bool:
        for attempt in range(self._nav_retry_limit + 1):
            if self._stop_event.is_set():
                return False
            self._nav_retries_current = attempt
            self._wait_if_paused()
            success = self._send_nav_goal(waypoint)
            if success:
                self._log_event("GOAL_SUCCEEDED")
                return True
            self._nav_retries_total += 1
            self._log_event("GOAL_RETRY", retries=attempt + 1, severity="WARN")
            if attempt < self._nav_retry_limit:
                if not self._sleep_with_checks(2.0):
                    return False
        return False

    def _send_nav_goal(self, waypoint: Waypoint) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint.x
        goal_msg.pose.pose.position.y = waypoint.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(math.radians(waypoint.yaw_deg))

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb)
        self._current_goal_id = ""

        while not send_future.done():
            if self._stop_event.is_set():
                return False
            self._wait_if_paused()
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Navigate goal rejected")
            return False
        with self._nav_lock:
            self._nav_goal_handle = goal_handle
        self._current_goal_id = goal_handle.goal_id.uuid.tobytes().hex()

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self._stop_event.is_set():
                self._cancel_nav_goal("stop")
                return False
            self._wait_if_paused()
            time.sleep(0.2)

        status = result_future.result().status
        with self._nav_lock:
            self._nav_goal_handle = None
        self._current_goal_id = ""
        return status == GoalStatus.STATUS_SUCCEEDED

    def _nav_feedback_cb(self, _feedback):
        # Placeholder for future use (e.g., update ETA).
        pass

    # Wait / pause helpers -------------------------------------------------

    def _wait_if_paused(self) -> None:
        with self._pause_cv:
            while self._paused and not self._stop_event.is_set():
                self._pause_cv.wait(timeout=0.2)

    def _sleep_with_checks(self, duration: float) -> bool:
        end_time = time.time() + duration
        while time.time() < end_time:
            if self._stop_event.is_set():
                return False
            self._wait_if_paused()
            time.sleep(0.1)
        return True

    def _wait_with_pause(self, duration: float) -> bool:
        return self._sleep_with_checks(duration)

    # Task execution -------------------------------------------------------

    def _execute_task(self, task: TaskSpec, task_index: int) -> bool:
        if not task.cmd:
            return True
        for attempt in range(task.retry + 1):
            if self._stop_event.is_set():
                return False
            self._task_retries_current = attempt
            self._current_task = task.type or "task"
            self._log_event("TASK_START", task_type=task.type, cmd=task.cmd)
            try:
                log_handle = None
                log_path = None
                if self._task_logs_dir:
                    log_path = self._task_logs_dir / (
                        f"wp{self.current_index:03d}_task{task_index:02d}"
                        f"_try{attempt}_{int(time.time())}.log")
                    log_handle = open(log_path, "wb")
                    log_handle.write(b"STDOUT+STDERR (streamed)\n\n")
                else:
                    log_handle = open(os.devnull, "wb")
                with self._task_lock:
                    self._task_proc = subprocess.Popen(
                        task.cmd,
                        shell=True,
                        stdout=log_handle,
                        stderr=subprocess.STDOUT,
                        preexec_fn=os.setsid,
                    )
                    proc = self._task_proc
                start = time.time()
                retcode = None
                while True:
                    retcode = proc.poll()
                    if retcode is not None:
                        break
                    if self._stop_event.is_set() or self._paused:
                        self._terminate_task_proc()
                        break
                    elapsed = time.time() - start
                    if elapsed > task.timeout:
                        self._terminate_task_proc()
                        break
                    time.sleep(0.1)
                if retcode is None:
                    # Process was asked to terminate; wait a bit so log file captures trailing output.
                    try:
                        proc.wait(timeout=1.0)
                        retcode = proc.returncode
                    except subprocess.TimeoutExpired:
                        retcode = proc.returncode
                if retcode == 0:
                    self._log_event(
                        "TASK_OK",
                        task_type=task.type,
                        log_path=str(log_path) if log_path else "",
                    )
                    return True
                self._task_retries_total += 1
                self._log_event(
                    "TASK_FAIL",
                    severity="WARN",
                    retries=attempt + 1,
                    task_type=task.type,
                    log_path=str(log_path) if log_path else "",
                )
            finally:
                if log_handle and not log_handle.closed:
                    log_handle.close()
                with self._task_lock:
                    self._task_proc = None
            if attempt < task.retry:
                if not self._sleep_with_checks(1.0):
                    break
        return False

    def _terminate_task_proc(self) -> None:
        with self._task_lock:
            proc = self._task_proc
            if proc and proc.poll() is None:
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    pass
            self._task_proc = None

    # Nav goal cancellation ------------------------------------------------

    def _cancel_nav_goal(self, reason: str) -> None:
        with self._nav_lock:
            handle = self._nav_goal_handle
        if handle:
            self._log_event("GOAL_CANCEL", msg=reason)
            cancel_future = handle.cancel_goal_async()
            # Add timeout to prevent hanging
            timeout = 2.0
            start = time.time()
            while not cancel_future.done():
                if time.time() - start > timeout:
                    self.get_logger().warning(f"Cancel goal timeout after {timeout}s")
                    break
                time.sleep(0.05)
            with self._nav_lock:
                self._nav_goal_handle = None

    # Utility --------------------------------------------------------------

    @staticmethod
    def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q


# Main ----------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = PatrolManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
