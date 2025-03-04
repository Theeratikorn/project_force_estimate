import sys
import threading
import time
import math
import pandas as pd
import rtde_control
import rtde_receive
import os
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, 
    QLineEdit, QFormLayout, QHBoxLayout, QMessageBox
)
from PyQt6.QtCore import QTimer

class UR5eControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        # Robot Connection
        self.rtde_c = None
        self.rtde_r = None
        self.stop_event = threading.Event()
        self.data = []

        # Timer for real-time updates
        self.timer = QTimer()
        # self.timer.timeout.connect(self.update_status)

    def initUI(self):
        self.setWindowTitle("UR5e Control GUI")
        self.setGeometry(100, 100, 400, 600)

        layout = QVBoxLayout()

        # IP Address Input
        form_layout = QFormLayout()
        self.input_ip = QLineEdit("192.168.56.101")  # Default IP
        self.btn_connect = QPushButton("Connect to Robot")
        self.btn_connect.clicked.connect(self.connect_robot)
        form_layout.addRow("Robot IP:", self.input_ip)
        form_layout.addRow(self.btn_connect)

        # Input Fields for Motion Parameters
        self.input_amp1 = QLineEdit("0.0025")
        self.input_amp2 = QLineEdit("0.005")
        self.input_amp3 = QLineEdit("0.0075")
        self.input_amp4 = QLineEdit("0.01")
        self.input_T_motion = QLineEdit("0.05")
        self.input_steps_per_cycle = QLineEdit("1000")
        self.input_cycles = QLineEdit("5")
        self.input_freq_logging = QLineEdit("500")
        self.input_speed = QLineEdit("0.5")
        self.input_acceleration = QLineEdit("2.0")

        form_layout.addRow("Amplitude 1 (m):", self.input_amp1)
        form_layout.addRow("Amplitude 2 (m):", self.input_amp2)
        form_layout.addRow("Amplitude 3 (m):", self.input_amp3)
        form_layout.addRow("Amplitude 4 (m):", self.input_amp4)
        form_layout.addRow("T_motion (s):", self.input_T_motion)
        form_layout.addRow("Steps per cycle:", self.input_steps_per_cycle)
        form_layout.addRow("Cycles:", self.input_cycles)
        form_layout.addRow("Logging Frequency (Hz):", self.input_freq_logging)
        form_layout.addRow("Speed (m/s):", self.input_speed)
        form_layout.addRow("Acceleration (m/s²):", self.input_acceleration)

        layout.addLayout(form_layout)

        # Start/Stop Buttons
        btn_layout = QHBoxLayout()
        self.btn_start = QPushButton("Start Robot")
        self.btn_stop = QPushButton("Stop Robot")
        
        self.btn_start.clicked.connect(self.start_robot)
        self.btn_stop.clicked.connect(self.stop_robot)

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(False)

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        layout.addLayout(btn_layout)

        # Status Label
        self.status_label = QLabel("Status: Disconnected")
        layout.addWidget(self.status_label)

        # Data Display Label
        self.data_label = QLabel("Data: N/A")
        layout.addWidget(self.data_label)

        self.setLayout(layout)

    def connect_robot(self):
        """Attempts to connect to the UR5e robot"""
        try:
            ip = self.input_ip.text().strip()
            self.rtde_c = rtde_control.RTDEControlInterface(ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
            self.status_label.setText("Status: Connected")
            self.btn_start.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect to {ip}\nError: {e}")
            self.rtde_c = None
            self.rtde_r = None
            self.status_label.setText("Status: Disconnected")

    def start_robot(self):
        """Starts the robot movement and logging"""
        if not self.rtde_c or not self.rtde_r:
            QMessageBox.warning(self, "Warning", "Not connected to robot!")
            return

        try:
            self.stop_event.clear()
            self.data = []  # Clear previous data

            # Get user inputs
            amps = [
                float(self.input_amp1.text()),
                float(self.input_amp2.text()),
                float(self.input_amp3.text()),
                float(self.input_amp4.text())
            ]
            T_motion = float(self.input_T_motion.text())
            steps_per_cycle = int(self.input_steps_per_cycle.text())
            cycles = int(self.input_cycles.text())
            freq_logging = int(self.input_freq_logging.text())
            speed = float(self.input_speed.text())
            acceleration = float(self.input_acceleration.text())

            freq_motion = 1 / T_motion
            dt_motion = T_motion / steps_per_cycle
            dt_logging = 1 / freq_logging
            total_steps = cycles * steps_per_cycle

            self.status_label.setText("Status: Running...")
            self.btn_start.setEnabled(False)
            self.btn_stop.setEnabled(True)

            # Start Threads
            for amp in amps:
                if self.stop_event.is_set():
                    break

                self.data = []  # Clear data for each amplitude
                robot_thread = threading.Thread(target=self.move_robot, args=(amp, freq_motion, dt_motion, total_steps, speed, acceleration))
                log_thread = threading.Thread(target=self.log_data, args=(dt_logging,))

                robot_thread.start()
                log_thread.start()

                robot_thread.join()
                log_thread.join()

                self.export_csv(amp)

            self.status_label.setText("Status: Completed")
            self.btn_start.setEnabled(True)
            self.btn_stop.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start: {str(e)}")

    def stop_robot(self):
        """Stops the robot and exports data"""
        self.stop_event.set()
        self.status_label.setText("Status: Stopped")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.timer.stop()
        
        self.export_csv()

    def move_robot(self, amp, freq_motion, dt_motion, total_steps, speed, acceleration):
        """Moves the robot in a sinusoidal Z motion"""
        try:
            start_pose = self.rtde_r.getActualTCPPose()
            
            for i in range(total_steps):
                if self.stop_event.is_set():
                    break

                start_time = time.time()
                z_offset = amp * math.cos(2 * math.pi * freq_motion * (i / total_steps)) - amp
                new_pose = start_pose.copy()
                new_pose[2] += z_offset

                self.rtde_c.moveL(new_pose, speed, acceleration)

                elapsed_time = time.time() - start_time
                remaining_time = dt_motion - elapsed_time
                if (remaining_time > 0):
                    time.sleep(remaining_time)

            self.stop_event.set()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to move robot: {str(e)}")

    def log_data(self, dt_logging):
        """Logs robot data"""
        try:
            start_global_time = time.time()
            
            while not self.stop_event.is_set():
                start_time = time.time()

                actual_joint_positions = self.rtde_r.getActualQ()
                actual_joint_speeds = self.rtde_r.getActualQd()
                actual_joint_efforts = self.rtde_c.getJointTorques()
                actual_control_current = self.rtde_r.getActualCurrent()
                actual_control_voltage = self.rtde_r.getActualJointVoltage()
                actual_tcp_pose = self.rtde_r.getActualTCPPose()
                actual_tcp_speed = self.rtde_r.getActualTCPSpeed()
                actual_tcp_force = self.rtde_r.getActualTCPForce()

                target_joint_positions = self.rtde_r.getTargetQ()
                target_joint_speeds = self.rtde_r.getTargetQd()
                target_joint_accelerations = self.rtde_r.getTargetQdd()
                target_joint_moments = self.rtde_r.getTargetMoment()

                target_joint_currents = self.rtde_r.getTargetCurrent()
                # target_joint_voltage = self.rtde_r.getTargetVoltage()

                actual_momentums = self.rtde_r.getActualMomentum()
                target_tcp_pose = self.rtde_r.getTargetTCPPose()
                target_tcp_speed = self.rtde_r.getTargetTCPSpeed()

                timestamp = time.time() - start_global_time
                self.data.append([
                    timestamp, *actual_joint_positions, *actual_joint_speeds, *actual_joint_efforts,
                    *actual_control_current, *actual_control_voltage, *actual_tcp_force, *actual_tcp_pose, *actual_tcp_speed,
                    *target_joint_positions, *target_joint_speeds, *target_joint_accelerations,
                    *target_joint_moments, *target_joint_currents, #*target_joint_voltage,
                     actual_momentums, *target_tcp_pose, *target_tcp_speed
                ])

                elapsed_time = time.time() - start_time
                remaining_time = dt_logging - elapsed_time
                if remaining_time > 0:
                    time.sleep(remaining_time)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to log data: {str(e)}")

    def update_status(self):
        """Updates the GUI with the latest recorded data"""
        if self.data:
            latest_data = self.data[-1]
            tcp_x, tcp_y, tcp_z = latest_data[7:10]
            force_x, force_y, force_z = latest_data[13:16]
            self.data_label.setText(f"TCP: ({tcp_x:.3f}, {tcp_y:.3f}, {tcp_z:.3f}), Force: ({force_x:.3f}, {force_y:.3f}, {force_z:.3f})")
        else:
            self.data_label.setText("Data: N/A")

    def export_csv(self, amp):
        """Exports collected data to CSV"""
        if not self.data:
            QMessageBox.warning(self, "Warning", "No data to export!")
            return

        amp_str = str(amp).replace(".", "_")
        filename = f"UR5e_Amp_{amp_str}.csv"
        count = 1

        while os.path.exists(filename):
            filename = f"UR5e_Amp_{amp_str}_{count}.csv"
            count += 1

        columns = ["timestamp"] + \
            [f"actual_position_{i}" for i in range(6)] + \
            [f"actual_velocity_{i}" for i in range(6)] + \
            [f"actual_effort_{i}" for i in range(6)] + \
            [f"actual_control_current_{i}" for i in range(6)] + \
            [f"actual_control_voltage_{i}" for i in range(6)] + \
            ["actual_force_x", "actual_force_y", "actual_force_z", "actual_torque_x", "actual_torque_y", "actual_torque_z"] + \
            ["actual_tcp_pos_x", "actual_tcp_pos_y", "actual_tcp_pos_z", "actual_tcp_ori_x", "actual_tcp_ori_y", "actual_tcp_ori_z"] + \
            ["actual_tcp_speed_x", "actual_tcp_speed_y", "actual_tcp_speed_z", "actual_tcp_speed_rx", "actual_tcp_speed_ry", "actual_tcp_speed_rz"] + \
            [f"target_position_{i}" for i in range(6)] + \
            [f"target_velocity_{i}" for i in range(6)] + \
            [f"target_acceleration_{i}" for i in range(6)] + \
            [f"target_moment_{i}" for i in range(6)] + \
            [f"target_current_{i}" for i in range(6)] + \
            ["actual_momentum"]+ \
            ["target_tcp_pos_x", "target_tcp_pos_y", "target_tcp_pos_z", "target_tcp_ori_x", "target_tcp_ori_y", "target_tcp_ori_z"] + \
            ["target_tcp_speed_x", "target_tcp_speed_y", "target_tcp_speed_z", "target_tcp_speed_rx", "target_tcp_speed_ry", "target_tcp_speed_rz"]

        df = pd.DataFrame(self.data, columns=columns)
        df.to_csv(filename, index=False)

        QMessageBox.information(self, "Export", f"Data saved as {filename}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = UR5eControlGUI()
    window.show()
    sys.exit(app.exec())
