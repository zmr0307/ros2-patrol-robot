from __future__ import annotations

import functools
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin
from robot_bringup_cpp.msg import PatrolStatus
from robot_bringup_cpp.srv import SetWaypoints, GoToReturnPoint, StartPatrol
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PatrolPanelNode(Node):

    def __init__(self) -> None:
        super().__init__("patrol_panel_gui")
        self.status: Optional[PatrolStatus] = None
        self.last_event: Optional[str] = None  # Track latest event
        # Keep references to QTimer objects to prevent premature GC that would suppress timeouts.
        self._active_timers: dict[object, QTimer] = {}
        self.create_subscription(
            PatrolStatus,
            "/patrol/status",
            self._status_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        # Subscribe to patrol events
        self.create_subscription(
            String,
            "/patrol/events",
            self._events_cb,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.start_client = self.create_client(Trigger, "/patrol/start")
        self.start_patrol_client = self.create_client(StartPatrol, "/patrol/start_patrol")
        self.pause_client = self.create_client(Trigger, "/patrol/pause")
        self.resume_client = self.create_client(Trigger, "/patrol/resume")
        self.stop_client = self.create_client(Trigger, "/patrol/stop")
        self.set_wps_client = self.create_client(SetWaypoints, "/patrol/set_waypoints")
        self.return_point_client = self.create_client(GoToReturnPoint, "/patrol/go_to_return_point")

    def _status_cb(self, msg: PatrolStatus) -> None:
        self.status = msg

    def _events_cb(self, msg: String) -> None:
        """Handle patrol events for return point status updates."""
        try:
            import json
            event_data = json.loads(msg.data)
            event_type = event_data.get("event", "")
            if event_type in ["RETURN_POINT_SUCCESS", "RETURN_POINT_FAILED"]:
                self.last_event = event_type
        except:  # noqa
            pass

    def call_trigger(self, client, on_result, timeout_sec: float = 5.0) -> None:
        if not client.wait_for_service(timeout_sec=2.0):
            on_result(False, "Service unavailable")
            return
        future = client.call_async(Trigger.Request())

        # Add timeout handling
        timeout_timer = QTimer()
        timeout_timer.setSingleShot(True)
        timeout_occurred = [False]
        self._active_timers[future] = timeout_timer

        def timeout_cb():
            if not future.done():
                timeout_occurred[0] = True
                on_result(False, f"Service call timeout after {timeout_sec}s")
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)

        timeout_timer.timeout.connect(timeout_cb)
        timeout_timer.start(int(timeout_sec * 1000))

        def done_cb(fut):
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)
            if timeout_occurred[0]:
                return  # Already handled by timeout
            try:
                resp = fut.result()
                on_result(resp.success, resp.message)
            except Exception as exc:  # noqa: BLE001
                on_result(False, str(exc))

        future.add_done_callback(done_cb)

    def call_set_waypoints(self, yaml_path: str, on_result) -> None:
        if not self.set_wps_client.wait_for_service(timeout_sec=2.0):
            on_result(False, "Service unavailable")
            return
        req = SetWaypoints.Request()
        req.yaml_path = yaml_path
        future = self.set_wps_client.call_async(req)

        # Add timeout handling
        timeout_timer = QTimer()
        timeout_timer.setSingleShot(True)
        timeout_occurred = [False]
        self._active_timers[future] = timeout_timer

        def timeout_cb():
            if not future.done():
                timeout_occurred[0] = True
                on_result(False, "Service call timeout after 5s")
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)

        timeout_timer.timeout.connect(timeout_cb)
        timeout_timer.start(5000)  # 5 seconds

        def done_cb(fut):
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)
            if timeout_occurred[0]:
                return  # Already handled by timeout
            try:
                resp = fut.result()
                on_result(resp.success, resp.message)
            except Exception as exc:  # noqa: BLE001
                on_result(False, str(exc))

        future.add_done_callback(done_cb)

    def call_go_to_return_point(self, x: float, y: float, yaw_deg: float, on_result) -> None:
        if not self.return_point_client.wait_for_service(timeout_sec=2.0):
            on_result(False, "Service unavailable")
            return
        req = GoToReturnPoint.Request()
        req.x = x
        req.y = y
        req.yaw_deg = yaw_deg
        future = self.return_point_client.call_async(req)

        # Add timeout handling
        timeout_timer = QTimer()
        timeout_timer.setSingleShot(True)
        timeout_occurred = [False]
        self._active_timers[future] = timeout_timer

        def timeout_cb():
            if not future.done():
                timeout_occurred[0] = True
                on_result(False, "Service call timeout after 5s")
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)

        timeout_timer.timeout.connect(timeout_cb)
        timeout_timer.start(5000)  # 5 seconds

        def done_cb(fut):
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)
            if timeout_occurred[0]:
                return  # Already handled by timeout
            try:
                resp = fut.result()
                on_result(resp.success, resp.message)
            except Exception as exc:  # noqa: BLE001
                on_result(False, str(exc))

        future.add_done_callback(done_cb)

    def call_start_patrol(self, cycles: int, on_result) -> None:
        if not self.start_patrol_client.wait_for_service(timeout_sec=2.0):
            on_result(False, "Service unavailable")
            return
        req = StartPatrol.Request()
        req.cycles = cycles
        future = self.start_patrol_client.call_async(req)

        # Add timeout handling
        timeout_timer = QTimer()
        timeout_timer.setSingleShot(True)
        timeout_occurred = [False]
        self._active_timers[future] = timeout_timer

        def timeout_cb():
            if not future.done():
                timeout_occurred[0] = True
                on_result(False, "Service call timeout after 5s")
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)

        timeout_timer.timeout.connect(timeout_cb)
        timeout_timer.start(5000)  # 5 seconds

        def done_cb(fut):
            timeout_timer.stop()
            timeout_timer.deleteLater()
            self._active_timers.pop(future, None)
            if timeout_occurred[0]:
                return  # Already handled by timeout
            try:
                resp = fut.result()
                on_result(resp.success, resp.message)
            except Exception as exc:  # noqa: BLE001
                on_result(False, str(exc))

        future.add_done_callback(done_cb)


class PatrolPanel(Plugin):

    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName("PatrolPanel")
        # Initialize rclpy if not already initialized (safe to call multiple times)
        try:
            rclpy.init(args=None)
        except RuntimeError:
            # Already initialized, ignore
            pass
        self._node = PatrolPanelNode()

        self._widget = QWidget()
        self._widget.setObjectName("PatrolPanelWidget")
        layout = QVBoxLayout()
        self._widget.setLayout(layout)

        default_yaml = ""
        try:
            default_yaml = str(
                Path(get_package_share_directory("robot_bringup_cpp")) /
                "config" / "patrol_waypoints.yaml")
        except Exception:
            pass

        self._status_text = QTextEdit()
        self._status_text.setReadOnly(True)
        layout.addWidget(self._status_text)

        control_box = QGroupBox("Control")
        control_layout = QHBoxLayout()
        control_box.setLayout(control_layout)
        layout.addWidget(control_box)

        self._start_btn = QPushButton("Start")
        self._pause_btn = QPushButton("Pause")
        self._resume_btn = QPushButton("Resume")
        self._stop_btn = QPushButton("Stop")
        control_layout.addWidget(self._start_btn)
        control_layout.addWidget(self._pause_btn)
        control_layout.addWidget(self._resume_btn)
        control_layout.addWidget(self._stop_btn)

        self._start_btn.clicked.connect(functools.partial(self._call_trigger, "start"))
        self._pause_btn.clicked.connect(functools.partial(self._call_trigger, "pause"))
        self._resume_btn.clicked.connect(functools.partial(self._call_trigger, "resume"))
        self._stop_btn.clicked.connect(functools.partial(self._call_trigger, "stop"))

        params_box = QGroupBox("Waypoints")
        params_layout = QFormLayout()
        params_box.setLayout(params_layout)
        layout.addWidget(params_box)

        self._waypoints_path = QLineEdit(default_yaml)
        self._loop_check = QCheckBox()
        self._loop_check.setChecked(True)
        self._wait_spin = QDoubleSpinBox()
        self._wait_spin.setRange(0.0, 120.0)
        self._wait_spin.setValue(5.0)
        self._timeout_spin = QDoubleSpinBox()
        self._timeout_spin.setRange(0.0, 120.0)
        self._timeout_spin.setValue(10.0)
        self._retry_spin = QSpinBox()
        self._retry_spin.setRange(0, 10)
        self._retry_spin.setValue(2)
        params_layout.addRow("Waypoints YAML", self._waypoints_path)
        params_layout.addRow("Loop", self._loop_check)
        params_layout.addRow("Default wait (s)", self._wait_spin)
        params_layout.addRow("Task timeout (s)", self._timeout_spin)
        params_layout.addRow("Task retry", self._retry_spin)

        self._reload_btn = QPushButton("Reload Waypoints")
        params_layout.addRow(self._reload_btn)
        self._reload_btn.clicked.connect(self._reload_waypoints)

        # Loop Settings section
        loop_box = QGroupBox("循环设置")
        loop_layout = QFormLayout()
        loop_box.setLayout(loop_layout)
        layout.addWidget(loop_box)

        self._patrol_cycles = QSpinBox()
        self._patrol_cycles.setRange(1, 100)
        self._patrol_cycles.setValue(1)
        self._patrol_cycles.setToolTip("设置完整巡检的次数（1-100次）")
        loop_layout.addRow("巡检次数", self._patrol_cycles)

        # Return Point section
        return_box = QGroupBox("Return Point")
        return_layout = QFormLayout()
        return_box.setLayout(return_layout)
        layout.addWidget(return_box)

        self._return_x = QDoubleSpinBox()
        self._return_x.setRange(-100.0, 100.0)
        self._return_x.setDecimals(3)
        self._return_x.setValue(0.0)
        
        self._return_y = QDoubleSpinBox()
        self._return_y.setRange(-100.0, 100.0)
        self._return_y.setDecimals(3)
        self._return_y.setValue(0.0)
        
        self._return_yaw = QDoubleSpinBox()
        self._return_yaw.setRange(-180.0, 180.0)
        self._return_yaw.setDecimals(1)
        self._return_yaw.setValue(0.0)
        
        return_layout.addRow("X (m)", self._return_x)
        return_layout.addRow("Y (m)", self._return_y)
        return_layout.addRow("Yaw (deg)", self._return_yaw)
        
        self._return_btn = QPushButton("🏠 Go To Return Point")
        self._return_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        return_layout.addRow(self._return_btn)
        self._return_btn.clicked.connect(self._go_to_return_point)

        self._log_label = QLabel("Ready")
        layout.addWidget(self._log_label)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} ({context.serial_number()})")
        context.add_widget(self._widget)

        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._spin_once)
        self._timer.start(50)

    # RQT hooks -----------------------------------------------------------

    def shutdown_plugin(self) -> None:
        self._timer.stop()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings) -> None:
        instance_settings.set_value("waypoints_path", self._waypoints_path.text())
        instance_settings.set_value("loop", int(self._loop_check.isChecked()))
        instance_settings.set_value("wait", self._wait_spin.value())
        instance_settings.set_value("timeout", self._timeout_spin.value())
        instance_settings.set_value("retry", self._retry_spin.value())
        instance_settings.set_value("patrol_cycles", self._patrol_cycles.value())
        instance_settings.set_value("return_x", self._return_x.value())
        instance_settings.set_value("return_y", self._return_y.value())
        instance_settings.set_value("return_yaw", self._return_yaw.value())

    def restore_settings(self, plugin_settings, instance_settings) -> None:
        if instance_settings.contains("waypoints_path"):
            self._waypoints_path.setText(instance_settings.value("waypoints_path"))
        if instance_settings.contains("loop"):
            self._loop_check.setChecked(bool(int(instance_settings.value("loop"))))
        if instance_settings.contains("wait"):
            self._wait_spin.setValue(float(instance_settings.value("wait")))
        if instance_settings.contains("timeout"):
            self._timeout_spin.setValue(float(instance_settings.value("timeout")))
        if instance_settings.contains("retry"):
            self._retry_spin.setValue(int(instance_settings.value("retry")))
        if instance_settings.contains("patrol_cycles"):
            self._patrol_cycles.setValue(int(instance_settings.value("patrol_cycles")))
        if instance_settings.contains("return_x"):
            self._return_x.setValue(float(instance_settings.value("return_x")))
        if instance_settings.contains("return_y"):
            self._return_y.setValue(float(instance_settings.value("return_y")))
        if instance_settings.contains("return_yaw"):
            self._return_yaw.setValue(float(instance_settings.value("return_yaw")))

    # Internal slots ------------------------------------------------------

    def _spin_once(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.0)
        status = self._node.status
        if not status:
            return
        lines = [
            f"Run: {status.run_id}",
            f"Phase: {status.phase}",
            f"Index: {status.current_index}/{status.total}",
            f"Visited OK/Fail: {status.visited_ok}/{status.visited_fail}",
            f"Tasks OK/Fail: {status.task_ok}/{status.task_fail}",
            f"Loop: {status.loop}",
            f"Current task: {status.current_task_name}",
            f"Retries nav/task current: {status.retries_nav_current}/{status.retries_task_current}",
        ]
        self._status_text.setPlainText("\n".join(lines))

        # Update return point status
        if self._node.last_event == "RETURN_POINT_SUCCESS":
            self._log_label.setText("Return: ✓ SUCCESS - Arrived at return point!")
            self._node.last_event = None  # Clear after displaying
        elif self._node.last_event == "RETURN_POINT_FAILED":
            self._log_label.setText("Return: ✗ FAILED - Could not reach return point")
            self._node.last_event = None  # Clear after displaying

    def _call_trigger(self, action: str) -> None:
        if action == "start":
            # Use start_patrol with cycles parameter
            cycles = self._patrol_cycles.value()
            def on_result(success: bool, message: str) -> None:
                self._log_label.setText(f"{action}: {'OK' if success else 'FAIL'} - {message}")
            self._node.call_start_patrol(cycles, on_result)
            return

        mapping = {
            "pause": self._node.pause_client,
            "resume": self._node.resume_client,
            "stop": self._node.stop_client,
        }
        client = mapping[action]

        def on_result(success: bool, message: str) -> None:
            self._log_label.setText(f"{action}: {'OK' if success else 'FAIL'} - {message}")

        self._node.call_trigger(client, on_result)

    def _reload_waypoints(self) -> None:
        path = self._waypoints_path.text().strip()
        if not path:
            self._log_label.setText("Waypoints path empty")
            return

        def on_result(success: bool, message: str) -> None:
            self._log_label.setText(f"Reload: {'OK' if success else 'FAIL'} - {message}")

        self._node.call_set_waypoints(path, on_result)

    def _go_to_return_point(self) -> None:
        x = self._return_x.value()
        y = self._return_y.value()
        yaw = self._return_yaw.value()

        def on_result(success: bool, message: str) -> None:
            self._log_label.setText(f"Return: {'OK' if success else 'FAIL'} - {message}")

        self._node.call_go_to_return_point(x, y, yaw, on_result)
