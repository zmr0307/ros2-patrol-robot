import sys
import os
import time
import serial
import shutil
import math
from pathlib import Path
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QPushButton, QLabel, QTextEdit, QLineEdit, QMessageBox,
    QGroupBox, QDoubleSpinBox, QSpinBox
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QProcess, QTimer
from PyQt5.QtGui import QTextCursor


# ------------------ 机械臂控制相关函数 ------------------
# MIT 协议参数区间
POS_RANGE = (-12.56, 12.56)       # 位置，单位 rad
VEL_RANGE = (-45.0, 45.0)         # 速度，单位 rad/s
KP_RANGE = (0.0, 500.0)           # Kp 范围
KD_RANGE = (0.0, 5.0)             # Kd 范围
TORQUE_RANGE = (-18.0, 18.0)      # 前馈扭矩范围


def clamp(value, bounds):
    """限制 value 在 bounds 区间内"""
    low, high = bounds
    return max(low, min(high, value))


def float_to_uint(value, bounds, bits):
    """浮点数线性映射到无符号整型"""
    low, high = bounds
    scale = (1 << bits) - 1
    value = clamp(value, bounds)
    encoded = int(round((value - low) * scale / (high - low)))
    return max(0, min(scale, encoded))


def build_mit_frame(position, velocity, kp, kd, torque, motor_id=1):
    """根据 MIT 协议，组装 CAN ID 和 8字节数据帧"""
    can_id = 0x200 + motor_id
    p_enc = float_to_uint(position, POS_RANGE, 16)
    v_enc = float_to_uint(velocity, VEL_RANGE, 12)
    kp_enc = float_to_uint(kp, KP_RANGE, 12)
    kd_enc = float_to_uint(kd, KD_RANGE, 12)
    t_enc = float_to_uint(torque, TORQUE_RANGE, 12)

    data_bytes = [
        (p_enc >> 8) & 0xFF,
        p_enc & 0xFF,
        (v_enc >> 4) & 0xFF,
        ((v_enc & 0xF) << 4) | ((kp_enc >> 8) & 0xF),
        kp_enc & 0xFF,
        (kd_enc >> 4) & 0xFF,
        ((kd_enc & 0xF) << 4) | ((t_enc >> 8) & 0xF),
        t_enc & 0xFF,
    ]

    frame_bytes = bytes([
        (can_id >> 8) & 0xFF,
        can_id & 0xFF,
        *data_bytes
    ])
    return frame_bytes


# ------------------ 机械臂控制线程 ------------------
class RobotArmThread(QThread):
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()
    
    def __init__(self, serial_port, action, **kwargs):
        super().__init__()
        self.serial_port = serial_port
        self.action = action
        self.kwargs = kwargs
        self.should_stop = False
        self._ser = None
    
    def run(self):
        try:
            self._ser = serial.Serial(self.serial_port, 115200, timeout=0.5)
            self.log_signal.emit(f"串口 {self.serial_port} 已打开")
            time.sleep(1)
            
            if self.action == 'enable':
                self.enable_motors()
            elif self.action == 'set_zero':
                self.set_zero_position()
            elif self.action == 'move_motor':
                self.move_motor()
            elif self.action == 'move_arm_extended':
                self.move_arm_extended()
            elif self.action == 'return_to_zero':
                self.return_to_zero()
                
        except serial.SerialException as e:
            self.log_signal.emit(f"串口错误: {str(e)}")
        except Exception as e:
            self.log_signal.emit(f"错误: {str(e)}")
        finally:
            if self._ser and self._ser.is_open:
                self._ser.close()
                self.log_signal.emit("串口已关闭")
            self.finished_signal.emit()
    
    def enable_motors(self):
        """使能电机"""
        for motor_id in [1, 2]:
            if self.should_stop:
                break
            can_id_high = 0x00
            can_id_low = motor_id & 0xFF
            ENABLE_DATA_BYTES = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
            frame = bytes([can_id_high, can_id_low] + ENABLE_DATA_BYTES)
            self._ser.write(frame)
            self.log_signal.emit(f"电机{motor_id}使能指令已发送: " + ' '.join(f"{b:02X}" for b in frame))
            time.sleep(2)
    
    def set_zero_position(self):
        """设置零点"""
        for motor_id in [1, 2]:
            if self.should_stop:
                break
            can_id_high = 0x00
            can_id_low = motor_id & 0xFF
            ZERO_DATA_BYTES = [0xFF]*7 + [0xFE]
            frame = bytes([can_id_high, can_id_low] + ZERO_DATA_BYTES)
            self._ser.write(frame)
            self.log_signal.emit(f"电机{motor_id}零点设置指令已发送: " + ' '.join(f"{b:02X}" for b in frame))
            time.sleep(0.1)
    
    def gradually_move(self, start_deg, end_deg, steps, kp, kd, torque, motor_id):
        """逐步移动电机"""
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        step_size = (end_rad - start_rad) / steps
        
        self.log_signal.emit(f"电机{motor_id}: {start_deg}° → {end_deg}°，共{steps}步")
        
        for i in range(steps + 1):
            if self.should_stop:
                break
            pos = start_rad + step_size * i
            frame = build_mit_frame(pos, velocity=0, kp=kp, kd=kd, torque=torque, motor_id=motor_id)
            self._ser.write(frame)
            
            if i % 10 == 0 or i == steps:  # 每10步或最后一步输出日志
                self.log_signal.emit(f"电机{motor_id} 第{i}步: {math.degrees(pos):.1f}°")
            
            time.sleep(0.3)
    
    def move_motor(self):
        """单个电机移动"""
        motor_id = self.kwargs.get('motor_id', 1)
        start_deg = self.kwargs.get('start_deg', 0)
        end_deg = self.kwargs.get('end_deg', 90)
        steps = self.kwargs.get('steps', 30)
        kp = self.kwargs.get('kp', 10)
        kd = self.kwargs.get('kd', 3)
        torque = self.kwargs.get('torque', 0.4)
        
        self.gradually_move(start_deg, end_deg, steps, kp, kd, torque, motor_id)
    
    def move_arm_extended(self):
        """机械臂伸直运动（只伸直，不回零）"""
        target_angle = self.kwargs.get('target_angle', 30)
        steps = self.kwargs.get('steps', 50)
        
        self.log_signal.emit(f"=== 机械臂移动到 {target_angle}° (伸直状态) ===")
        
        # 步骤1：大臂运动到目标角度
        self.log_signal.emit(f"步骤1: 大臂运动到 {target_angle}°")
        self.gradually_move(0, target_angle, steps, 10, 3, 0.5, 2)
        if not self.should_stop:
            time.sleep(0.5)
        
        # 步骤2：小臂伸直到180°
        if not self.should_stop:
            self.log_signal.emit(f"步骤2: 小臂伸直到 180°")
            self.gradually_move(0, 180, steps*2, 10, 3, 0.4, 1)
        
        if not self.should_stop:
            self.log_signal.emit(f"机械臂已伸直到位！大臂: {target_angle}°, 小臂: 180°")
            self.log_signal.emit("机械臂现在可以进行工作，完成后请点击'机械臂回零'按钮")
    
    def return_to_zero(self):
        """机械臂回零运动（从当前位置回到零点）"""
        current_big_arm_angle = self.kwargs.get('current_big_arm_angle', 30)
        steps = self.kwargs.get('steps', 50)
        
        self.log_signal.emit(f"=== 机械臂回零运动 ===")
        
        # 步骤1：大臂从当前角度回到0°
        self.log_signal.emit(f"步骤1: 大臂从 {current_big_arm_angle}° 回到 0°")
        self.gradually_move(current_big_arm_angle, 0, steps, 10, 3, 0.5, 2)
        if not self.should_stop:
            time.sleep(2)
        
        # 步骤2：小臂从180°折叠回0°
        if not self.should_stop:
            self.log_signal.emit(f"步骤2: 小臂从 180° 折叠回 0°")
            self.gradually_move(180, 0, steps*2, 10, 3, 0.4, 1)
        
        if not self.should_stop:
            self.log_signal.emit("机械臂已回零完成！")
    
    def stop(self):
        self.should_stop = True


# ------------------ 升降机控制线程 ------------------
class MotorControlThread(QThread):
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()
    position_signal = pyqtSignal(int)

    def __init__(self, serial_port, action, duration=0):
        super().__init__()
        self.serial_port = serial_port
        self.action = action
        self.duration = duration
        self.should_stop = False
        self._ser = None

    def run(self):
        try:
            self._ser = serial.Serial(
                self.serial_port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )

            if self.action == 'up':
                self.log_signal.emit("发送上升指令")
                self._ser.write(bytes.fromhex("01 06 00 01 00 02 59CB"))
                elapsed = 0
                while elapsed < self.duration and not self.should_stop:
                    sleep_time = min(0.1, self.duration - elapsed)
                    time.sleep(sleep_time)
                    elapsed += sleep_time
                if not self.should_stop:
                    self._ser.write(bytes.fromhex("01 06 00 01 00 01 19CA"))
                    self.log_signal.emit("自动停止")

            elif self.action == 'down':
                self.log_signal.emit("发送下降指令")
                self._ser.write(bytes.fromhex("01 06 00 01 00 04 D9C9"))
                elapsed = 0
                while elapsed < self.duration and not self.should_stop:
                    sleep_time = min(0.1, self.duration - elapsed)
                    time.sleep(sleep_time)
                    elapsed += sleep_time
                if not self.should_stop:
                    self._ser.write(bytes.fromhex("01 06 00 01 00 01 19CA"))
                    self.log_signal.emit("自动停止")

            elif self.action == 'stop':
                self.log_signal.emit("发送停止指令")
                self._ser.write(bytes.fromhex("01 06 00 01 00 01 19CA"))

            elif self.action == 'read_position':
                self.log_signal.emit("读取电机位置")
                self._ser.write(bytes.fromhex("01 03 00 02 00 01 25CA"))
                time.sleep(0.1)
                response = self._ser.read(64)
                self.log_signal.emit(f"收到回复(hex): {response.hex()}")
                res_hex = response.hex()
                idx = 0
                pos = None
                while idx < len(res_hex) - 12:
                    if res_hex[idx:idx+6] == "010302":
                        pos_high = int(res_hex[idx+6:idx+8], 16)
                        pos_low = int(res_hex[idx+8:idx+10], 16)
                        pos = (pos_high << 8) | pos_low
                        break
                    idx += 2
                if pos is not None:
                    self.log_signal.emit(f"电机当前位置: {pos}")
                    self.position_signal.emit(pos)
                else:
                    self.log_signal.emit("未收到有效位置数据")

        except serial.SerialException as e:
            self.log_signal.emit(f"串口错误: {str(e)}")
        except Exception as e:
            self.log_signal.emit(f"错误: {str(e)}")
        finally:
            if self._ser and self._ser.is_open:
                self._ser.close()
                self.log_signal.emit("串口已关闭")
            self.finished_signal.emit()

    def stop(self):
        self.should_stop = True


# ------------------ 机械臂控制 Tab ------------------
class RobotArmTab(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port = '/dev/ttyCH9344USB1'
        self.robot_thread = None
        self.current_big_arm_angle = 0  # 记录当前大臂角度
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        title = QLabel("机械臂控制面板")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # 串口设置
        serial_group = QGroupBox("串口设置")
        serial_layout = QHBoxLayout()
        serial_group.setLayout(serial_layout)
        serial_layout.addWidget(QLabel("串口:"))
        self.input_serial = QLineEdit(self.serial_port)
        self.input_serial.setFixedWidth(200)
        serial_layout.addWidget(self.input_serial)
        serial_layout.addStretch()
        layout.addWidget(serial_group)

        # 基本控制
        basic_group = QGroupBox("基本控制")
        basic_layout = QHBoxLayout()
        basic_group.setLayout(basic_layout)

        self.btn_enable = QPushButton("使能电机")
        self.btn_enable.clicked.connect(self.enable_motors)
        self.btn_enable.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        basic_layout.addWidget(self.btn_enable)

        self.btn_set_zero = QPushButton("设置零点")
        self.btn_set_zero.clicked.connect(self.set_zero_position)
        self.btn_set_zero.setStyleSheet("background-color: #FF9800; color: white; font-size: 14px; padding: 10px;")
        basic_layout.addWidget(self.btn_set_zero)

        self.btn_stop = QPushButton("紧急停止")
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_stop.setStyleSheet("background-color: #f44336; color: white; font-size: 14px; padding: 10px;")
        self.btn_stop.setEnabled(False)
        basic_layout.addWidget(self.btn_stop)

        layout.addWidget(basic_group)

        # 单电机控制
        motor_group = QGroupBox("单电机控制")
        motor_layout = QVBoxLayout()
        motor_group.setLayout(motor_layout)

        # 电机选择和参数
        motor_param_layout = QHBoxLayout()
        motor_param_layout.addWidget(QLabel("电机ID:"))
        self.input_motor_id = QSpinBox()
        self.input_motor_id.setRange(1, 2)
        self.input_motor_id.setValue(1)
        self.input_motor_id.setFixedWidth(60)
        motor_param_layout.addWidget(self.input_motor_id)

        motor_param_layout.addWidget(QLabel("起始角度(°):"))
        self.input_start_angle = QDoubleSpinBox()
        self.input_start_angle.setRange(-360, 360)
        self.input_start_angle.setValue(0)
        self.input_start_angle.setFixedWidth(80)
        motor_param_layout.addWidget(self.input_start_angle)

        motor_param_layout.addWidget(QLabel("目标角度(°):"))
        self.input_end_angle = QDoubleSpinBox()
        self.input_end_angle.setRange(-360, 360)
        self.input_end_angle.setValue(90)
        self.input_end_angle.setFixedWidth(80)
        motor_param_layout.addWidget(self.input_end_angle)

        motor_param_layout.addWidget(QLabel("步数:"))
        self.input_steps = QSpinBox()
        self.input_steps.setRange(1, 200)
        self.input_steps.setValue(30)
        self.input_steps.setFixedWidth(60)
        motor_param_layout.addWidget(self.input_steps)

        motor_param_layout.addStretch()
        motor_layout.addLayout(motor_param_layout)

        # PID参数
        pid_layout = QHBoxLayout()
        pid_layout.addWidget(QLabel("Kp:"))
        self.input_kp = QDoubleSpinBox()
        self.input_kp.setRange(0, 500)
        self.input_kp.setValue(10)
        self.input_kp.setFixedWidth(80)
        pid_layout.addWidget(self.input_kp)

        pid_layout.addWidget(QLabel("Kd:"))
        self.input_kd = QDoubleSpinBox()
        self.input_kd.setRange(0, 5)
        self.input_kd.setValue(3)
        self.input_kd.setFixedWidth(80)
        pid_layout.addWidget(self.input_kd)

        pid_layout.addWidget(QLabel("扭矩:"))
        self.input_torque = QDoubleSpinBox()
        self.input_torque.setRange(-18, 18)
        self.input_torque.setValue(0.4)
        self.input_torque.setSingleStep(0.1)
        self.input_torque.setFixedWidth(80)
        pid_layout.addWidget(self.input_torque)

        self.btn_move_motor = QPushButton("执行单电机运动")
        self.btn_move_motor.clicked.connect(self.move_single_motor)
        self.btn_move_motor.setStyleSheet("background-color: #2196F3; color: white; font-size: 14px; padding: 10px;")
        pid_layout.addWidget(self.btn_move_motor)

        pid_layout.addStretch()
        motor_layout.addLayout(pid_layout)

        layout.addWidget(motor_group)

        # 机械臂整体控制
        arm_group = QGroupBox("机械臂整体控制")
        arm_layout = QVBoxLayout()
        arm_group.setLayout(arm_layout)

        # 伸直控制
        extend_layout = QHBoxLayout()
        extend_layout.addWidget(QLabel("目标角度(°):"))
        self.input_arm_angle = QDoubleSpinBox()
        self.input_arm_angle.setRange(-180, 180)
        self.input_arm_angle.setValue(30)
        self.input_arm_angle.setFixedWidth(80)
        extend_layout.addWidget(self.input_arm_angle)

        extend_layout.addWidget(QLabel("运动步数:"))
        self.input_arm_steps = QSpinBox()
        self.input_arm_steps.setRange(10, 200)
        self.input_arm_steps.setValue(50)
        self.input_arm_steps.setFixedWidth(80)
        extend_layout.addWidget(self.input_arm_steps)

        self.btn_move_arm = QPushButton("机械臂伸直")
        self.btn_move_arm.clicked.connect(self.move_arm_extended)
        self.btn_move_arm.setStyleSheet("background-color: #9C27B0; color: white; font-size: 14px; padding: 10px;")
        extend_layout.addWidget(self.btn_move_arm)

        extend_layout.addStretch()
        arm_layout.addLayout(extend_layout)

        # 回零控制
        return_layout = QHBoxLayout()
        return_layout.addWidget(QLabel("当前大臂角度(°):"))
        self.label_current_angle = QLabel("0")
        self.label_current_angle.setStyleSheet("font-size: 14px; font-weight: bold; color: #2196F3;")
        return_layout.addWidget(self.label_current_angle)

        self.btn_return_zero = QPushButton("机械臂回零")
        self.btn_return_zero.clicked.connect(self.return_to_zero)
        self.btn_return_zero.setStyleSheet("background-color: #607D8B; color: white; font-size: 14px; padding: 10px;")
        return_layout.addWidget(self.btn_return_zero)

        return_layout.addStretch()
        arm_layout.addLayout(return_layout)

        layout.addWidget(arm_group)

        # 日志
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout()
        log_group.setLayout(log_layout)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        log_layout.addWidget(self.log_area)
        layout.addWidget(log_group)

    def log(self, text):
        self.log_area.moveCursor(QTextCursor.End)
        self.log_area.insertPlainText(f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.log_area.moveCursor(QTextCursor.End)

    def set_buttons_enabled(self, enabled):
        self.btn_enable.setEnabled(enabled)
        self.btn_set_zero.setEnabled(enabled)
        self.btn_move_motor.setEnabled(enabled)
        self.btn_move_arm.setEnabled(enabled)
        self.btn_return_zero.setEnabled(enabled)
        self.btn_stop.setEnabled(not enabled)

    def on_thread_finished(self):
        self.set_buttons_enabled(True)
        if self.robot_thread:
            self.robot_thread.wait()
            self.robot_thread = None

    def enable_motors(self):
        if self.robot_thread and self.robot_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        
        self.log("开始使能电机...")
        self.serial_port = self.input_serial.text()
        self.robot_thread = RobotArmThread(self.serial_port, 'enable')
        self.robot_thread.log_signal.connect(self.log)
        self.robot_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.robot_thread.start()

    def set_zero_position(self):
        if self.robot_thread and self.robot_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        
        self.log("设置零点位置...")
        self.serial_port = self.input_serial.text()
        self.robot_thread = RobotArmThread(self.serial_port, 'set_zero')
        self.robot_thread.log_signal.connect(self.log)
        self.robot_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.robot_thread.start()

    def move_single_motor(self):
        if self.robot_thread and self.robot_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        
        motor_id = self.input_motor_id.value()
        start_deg = self.input_start_angle.value()
        end_deg = self.input_end_angle.value()
        steps = self.input_steps.value()
        kp = self.input_kp.value()
        kd = self.input_kd.value()
        torque = self.input_torque.value()
        
        self.log(f"单电机运动: 电机{motor_id} {start_deg}° → {end_deg}°")
        self.serial_port = self.input_serial.text()
        
        self.robot_thread = RobotArmThread(
            self.serial_port, 'move_motor',
            motor_id=motor_id, start_deg=start_deg, end_deg=end_deg,
            steps=steps, kp=kp, kd=kd, torque=torque
        )
        self.robot_thread.log_signal.connect(self.log)
        self.robot_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.robot_thread.start()

    def move_arm_extended(self):
        """机械臂伸直（只伸直，不回零）"""
        if self.robot_thread and self.robot_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        
        target_angle = self.input_arm_angle.value()
        steps = self.input_arm_steps.value()
        
        # 更新当前角度记录
        self.current_big_arm_angle = target_angle
        self.label_current_angle.setText(f"{target_angle}")
        
        self.log(f"机械臂伸直运动: 目标角度 {target_angle}°")
        self.serial_port = self.input_serial.text()
        
        self.robot_thread = RobotArmThread(
            self.serial_port, 'move_arm_extended',
            target_angle=target_angle, steps=steps
        )
        self.robot_thread.log_signal.connect(self.log)
        self.robot_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.robot_thread.start()

    def return_to_zero(self):
        """机械臂回零"""
        if self.robot_thread and self.robot_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        
        steps = self.input_arm_steps.value()
        
        self.log(f"机械臂回零运动: 从当前位置回到零点")
        self.serial_port = self.input_serial.text()
        
        self.robot_thread = RobotArmThread(
            self.serial_port, 'return_to_zero',
            current_big_arm_angle=self.current_big_arm_angle, steps=steps
        )
        self.robot_thread.log_signal.connect(self.log)
        self.robot_thread.finished_signal.connect(self.on_thread_finished)
        self.robot_thread.finished_signal.connect(lambda: self.update_current_angle(0))
        self.set_buttons_enabled(False)
        self.robot_thread.start()

    def update_current_angle(self, angle):
        """更新当前角度显示"""
        self.current_big_arm_angle = angle
        self.label_current_angle.setText(f"{angle}")

    def emergency_stop(self):
        self.log("执行紧急停止")
        if self.robot_thread and self.robot_thread.isRunning():
            self.robot_thread.stop()
            self.robot_thread.wait()
        self.set_buttons_enabled(True)


# ------------------ 升降机控制 Tab ------------------
class MotorControlTab(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port = '/dev/ttyCH9344USB0'
        self.motor_speed = 10.2
        self.motor_thread = None
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        title = QLabel("升降机电机控制面板")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # 串口设置
        serial_group = QGroupBox("串口设置")
        serial_layout = QHBoxLayout()
        serial_group.setLayout(serial_layout)
        serial_layout.addWidget(QLabel("串口:"))
        self.input_serial = QLineEdit(self.serial_port)
        self.input_serial.setFixedWidth(200)
        serial_layout.addWidget(self.input_serial)
        serial_layout.addWidget(QLabel("速度(mm/s):"))
        self.input_speed = QDoubleSpinBox()
        self.input_speed.setRange(1.0, 50.0)
        self.input_speed.setValue(self.motor_speed)
        self.input_speed.setSingleStep(0.1)
        self.input_speed.setFixedWidth(100)
        serial_layout.addWidget(self.input_speed)
        serial_layout.addStretch()
        layout.addWidget(serial_group)

        # 控制按钮
        control_group = QGroupBox("电机控制")
        control_layout = QVBoxLayout()
        control_group.setLayout(control_layout)

        up_layout = QHBoxLayout()
        self.btn_up = QPushButton("电机上升")
        self.btn_up.clicked.connect(self.motor_up)
        self.btn_up.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        up_layout.addWidget(self.btn_up)
        up_layout.addWidget(QLabel("上升高度(mm):"))
        self.input_up_height = QDoubleSpinBox()
        self.input_up_height.setRange(0.0, 1000.0)
        self.input_up_height.setValue(50.0)
        self.input_up_height.setSingleStep(1.0)
        self.input_up_height.setFixedWidth(120)
        up_layout.addWidget(self.input_up_height)
        control_layout.addLayout(up_layout)

        down_layout = QHBoxLayout()
        self.btn_down = QPushButton("电机下降")
        self.btn_down.clicked.connect(self.motor_down)
        self.btn_down.setStyleSheet("background-color: #2196F3; color: white; font-size: 14px; padding: 10px;")
        down_layout.addWidget(self.btn_down)
        down_layout.addWidget(QLabel("下降高度(mm):"))
        self.input_down_height = QDoubleSpinBox()
        self.input_down_height.setRange(0.0, 1000.0)
        self.input_down_height.setValue(50.0)
        self.input_down_height.setSingleStep(1.0)
        self.input_down_height.setFixedWidth(120)
        down_layout.addWidget(self.input_down_height)
        control_layout.addLayout(down_layout)

        action_layout = QHBoxLayout()
        self.btn_stop = QPushButton("紧急停止")
        self.btn_stop.clicked.connect(self.motor_stop)
        self.btn_stop.setStyleSheet("background-color: #f44336; color: white; font-size: 14px; padding: 10px;")
        self.btn_stop.setEnabled(False)
        action_layout.addWidget(self.btn_stop)

        self.btn_read_pos = QPushButton("读取当前位置")
        self.btn_read_pos.clicked.connect(self.read_position)
        self.btn_read_pos.setStyleSheet("background-color: #FF9800; color: white; font-size: 14px; padding: 10px;")
        action_layout.addWidget(self.btn_read_pos)
        control_layout.addLayout(action_layout)

        layout.addWidget(control_group)

        # 位置显示
        position_group = QGroupBox("位置信息")
        position_layout = QHBoxLayout()
        position_group.setLayout(position_layout)
        position_layout.addWidget(QLabel("当前位置:"))
        self.label_position = QLabel("未知")
        self.label_position.setStyleSheet("font-size: 16px; font-weight: bold; color: #2196F3;")
        position_layout.addWidget(self.label_position)
        position_layout.addStretch()
        layout.addWidget(position_group)

        # 日志
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout()
        log_group.setLayout(log_layout)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        log_layout.addWidget(self.log_area)
        layout.addWidget(log_group)

    def log(self, text):
        self.log_area.moveCursor(QTextCursor.End)
        self.log_area.insertPlainText(f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.log_area.moveCursor(QTextCursor.End)

    def set_buttons_enabled(self, enabled):
        self.btn_up.setEnabled(enabled)
        self.btn_down.setEnabled(enabled)
        self.btn_read_pos.setEnabled(enabled)
        self.btn_stop.setEnabled(not enabled)

    def on_thread_finished(self):
        self.set_buttons_enabled(True)
        if self.motor_thread:
            self.motor_thread.wait()
            self.motor_thread = None

    def motor_up(self):
        if self.motor_thread and self.motor_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        height = self.input_up_height.value()
        speed = self.input_speed.value()
        duration = height / speed
        self.log(f"电机上升 {height} mm (预计 {duration:.2f} 秒)")
        self.serial_port = self.input_serial.text()
        self.motor_thread = MotorControlThread(self.serial_port, 'up', duration)
        self.motor_thread.log_signal.connect(self.log)
        self.motor_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.motor_thread.start()

    def motor_down(self):
        if self.motor_thread and self.motor_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        height = self.input_down_height.value()
        speed = self.input_speed.value()
        duration = height / speed
        self.log(f"电机下降 {height} mm (预计 {duration:.2f} 秒)")
        self.serial_port = self.input_serial.text()
        self.motor_thread = MotorControlThread(self.serial_port, 'down', duration)
        self.motor_thread.log_signal.connect(self.log)
        self.motor_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.motor_thread.start()

    def motor_stop(self):
        self.log("执行紧急停止")
        if self.motor_thread and self.motor_thread.isRunning():
            self.motor_thread.stop()
            self.motor_thread.wait()
        self.serial_port = self.input_serial.text()
        stop_thread = MotorControlThread(self.serial_port, 'stop')
        stop_thread.log_signal.connect(self.log)
        stop_thread.finished_signal.connect(lambda: stop_thread.wait())
        stop_thread.finished_signal.connect(lambda: self.set_buttons_enabled(True))
        stop_thread.start()

    def read_position(self):
        if self.motor_thread and self.motor_thread.isRunning():
            self.log("等待前一个任务完成...")
            return
        self.log("读取电机位置")
        self.serial_port = self.input_serial.text()
        self.motor_thread = MotorControlThread(self.serial_port, 'read_position')
        self.motor_thread.log_signal.connect(self.log)
        self.motor_thread.position_signal.connect(self.update_position)
        self.motor_thread.finished_signal.connect(self.on_thread_finished)
        self.set_buttons_enabled(False)
        self.motor_thread.start()

    def update_position(self, position):
        self.label_position.setText(f"{position}")


# ------------------ Azure Kinect 处理 Tab ------------------
def get_desktop_path():
    return str(Path.home() / "Desktop")


class KinectTab(QWidget):
    def __init__(self):
        super().__init__()
        self.desktop_path = Path(get_desktop_path())
        self.rgb_output_path = self.desktop_path / "rgb_output.mkv"
        self.rgb_frames_dir = self.desktop_path / "rgb_frames"
        self.k4arecorder_path = shutil.which("k4arecorder") or "/usr/bin/k4arecorder"
        self.ffmpeg_path = shutil.which("ffmpeg") or "/usr/bin/ffmpeg"
        self.process = None
        
        # 倒计时相关
        self.countdown_timer = QTimer()
        self.countdown_timer.timeout.connect(self.update_countdown)
        self.countdown_remaining = 0
        
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignLeft)
        layout.addWidget(self.status_label)

        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        layout.addWidget(self.log_area)

        btn_layout = QHBoxLayout()
        self.btn_record = QPushButton("开始录制 RGB 视频")
        self.btn_record.clicked.connect(self.run_record)
        btn_layout.addWidget(self.btn_record)
        
        self.btn_rgb = QPushButton("抽取 RGB 帧")
        self.btn_rgb.clicked.connect(self.run_rgb_extract)
        btn_layout.addWidget(self.btn_rgb)
        
        layout.addLayout(btn_layout)

        self.note_label = QLabel("说明: RGB视频保存到桌面，抽取的帧保存到 rgb_frames 文件夹。(4K@30fps)")
        self.note_label.setStyleSheet("color: gray;")
        layout.addWidget(self.note_label)

        self._build_inputs(layout)

    def _build_inputs(self, parent_layout):
        group = QGroupBox("参数设置")
        group_layout = QHBoxLayout()
        group.setLayout(group_layout)
        
        # 等待时间
        self.lbl_wait_time = QLabel("拍摄前等待(秒):")
        self.input_wait_time = QSpinBox()
        self.input_wait_time.setRange(0, 300)
        self.input_wait_time.setValue(3)
        self.input_wait_time.setFixedWidth(80)
        group_layout.addWidget(self.lbl_wait_time)
        group_layout.addWidget(self.input_wait_time)
        
        # 录制时长
        self.lbl_duration = QLabel("录制时长(秒):")
        self.input_duration = QLineEdit("100")
        self.input_duration.setFixedWidth(120)
        group_layout.addWidget(self.lbl_duration)
        group_layout.addWidget(self.input_duration)

        # RGB抽帧间隔
        self.lbl_rgb_step = QLabel("RGB抽帧间隔(帧):")
        self.input_rgb_step = QLineEdit("5")
        self.input_rgb_step.setFixedWidth(120)
        group_layout.addWidget(self.lbl_rgb_step)
        group_layout.addWidget(self.input_rgb_step)

        parent_layout.addWidget(group)

    def log(self, text):
        self.log_area.moveCursor(QTextCursor.End)
        self.log_area.insertPlainText(text)
        self.log_area.moveCursor(QTextCursor.End)

    def set_running(self, running: bool):
        if running:
            self.status_label.setText("正在执行,请稍候...")
        else:
            self.status_label.setText("就绪")
        self.btn_record.setEnabled(not running)
        self.btn_rgb.setEnabled(not running)

    def update_countdown(self):
        if self.countdown_remaining > 0:
            self.status_label.setText(f"拍摄倒计时: {self.countdown_remaining} 秒")
            self.log(f"倒计时: {self.countdown_remaining} 秒\n")
            self.countdown_remaining -= 1
        else:
            self.countdown_timer.stop()
            self.status_label.setText("开始录制...")
            self.log("倒计时结束，开始录制\n")
            self.start_actual_recording()

    def start_actual_recording(self):
        program = self.k4arecorder_path
        duration = self.input_duration.text().strip()
        
        cmd_args = [
            "-d", "OFF",
            "-c", "2160p",
            "-r", "30",
            "-l", duration,
            "--imu", "OFF",
            str(self.rgb_output_path)
        ]
        
        self.log(f"开始录制 4K@30fps RGB视频，时长 {duration} 秒\n")
        self.start_process(program, cmd_args)

    def start_process(self, program, arguments):
        if self.process is not None and self.process.state() == QProcess.Running:
            self.log("已有进程在运行,请稍后再试。\n")
            return
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyRead.connect(self.handle_output)
        self.process.started.connect(lambda: self.set_running(True))
        self.process.finished.connect(self.handle_finished)
        self.log(f"执行: {program} {' '.join(arguments)}\n")
        self.process.start(program, arguments)
        if not self.process.waitForStarted(5000):
            self.log("无法启动进程,请检查路径和参数。\n")
            self.set_running(False)

    def handle_output(self):
        if self.process is None:
            return
        data = self.process.readAllStandardOutput()
        if data is None:
            return
        try:
            text = bytes(data).decode('utf-8', errors='replace')
        except Exception:
            text = str(data)
        if text:
            self.log(text)

    def handle_finished(self, exitCode, exitStatus):
        self.set_running(False)
        if exitCode == 0:
            self.log(f"\n进程完成,退出码 {exitCode}\n")
        else:
            self.log(f"\n进程结束,退出码 {exitCode},状态 {exitStatus}\n")

    def run_record(self):
        self.rgb_output_path.parent.mkdir(parents=True, exist_ok=True)
        program = self.k4arecorder_path
        
        if not program or not Path(program).exists():
            QMessageBox.critical(self, "错误", f"k4arecorder 未找到,请确保已安装并在 PATH 中:{self.k4arecorder_path}")
            return
            
        duration = self.input_duration.text().strip()
        if not duration.isdigit() or int(duration) <= 0:
            QMessageBox.critical(self, "错误", "请在参数区域输入有效的录制时长(正整数,单位:秒)")
            return
            
        if not os.access(self.rgb_output_path.parent, os.W_OK):
            QMessageBox.critical(self, "权限错误", f"无法写入输出文件夹:{self.rgb_output_path.parent}")
            return

        wait_time = self.input_wait_time.value()
        if wait_time > 0:
            self.log(f"准备录制，等待 {wait_time} 秒后开始...\n")
            self.countdown_remaining = wait_time
            self.set_running(True)
            self.countdown_timer.start(1000)
        else:
            self.log("立即开始录制\n")
            self.start_actual_recording()

    def run_rgb_extract(self):
        self.rgb_frames_dir.mkdir(parents=True, exist_ok=True)
        input_path = str(self.rgb_output_path)
        
        if not Path(input_path).exists():
            QMessageBox.critical(self, "错误", f"输入文件不存在:{input_path}")
            return
            
        program = self.ffmpeg_path
        if not program or not Path(program).exists():
            QMessageBox.critical(self, "错误", f"ffmpeg 未找到,请确保已安装并在 PATH 中:{self.ffmpeg_path}")
            return
            
        rgb_step = self.input_rgb_step.text().strip()
        if not rgb_step.isdigit() or int(rgb_step) <= 0:
            QMessageBox.critical(self, "错误", "请在参数区域输入有效的RGB抽帧间隔(正整数,单位:帧)")
            return
            
        frame_path = str(self.rgb_frames_dir / "color-%04d.jpg")
        filter_expr = f"select=not(mod(n\\,{rgb_step}))"
        cmd_args = [
            "-i", input_path,
            "-vf", filter_expr,
            "-vsync", "vfr",
            frame_path
        ]
        
        self.log(f"抽取RGB帧:每{rgb_step}帧抽取一帧\n")
        self.start_process(program, cmd_args)


# ------------------ 主窗口 ------------------
class MainAppWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("升降机 & 机械臂 & Azure Kinect 控制台")
        self.resize(1200, 800)
        tabs = QTabWidget()
        tabs.addTab(MotorControlTab(), "升降机控制")
        tabs.addTab(RobotArmTab(), "机械臂控制")
        tabs.addTab(KinectTab(), "Azure Kinect RGB 录制")
        layout = QVBoxLayout(self)
        layout.addWidget(tabs)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainAppWindow()
    window.show()
    sys.exit(app.exec_())