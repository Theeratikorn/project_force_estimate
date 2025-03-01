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

        # Zero values
        self.zero_tcp = None
        self.zero_force = None

        # Timer for real-time updates
        self.timer = QTimer()
        # self.timer.timeout.connect(self.update_status)

    def initUI(self):
        self.setWindowTitle("UR5e Control GUI")
        self.setGeometry(100, 100, 400, 550)

        layout = QVBoxLayout()

        # IP Address Input
        form_layout = QFormLayout()
        self.input_ip = QLineEdit("192.168.1.102")  # Default IP
        self.btn_connect = QPushButton("Connect to Robot")
        self.btn_connect.clicked.connect(self.connect_robot)
        form_layout.addRow("Robot IP:", self.input_ip)
        form_layout.addRow(self.btn_connect)

        # Input Fields for Motion Parameters
        self.input_amps = QLineEdit("0.0025, 0.005, 0.0075, 0.01")  # Default amplitudes
        self.input_T_motion = QLineEdit("0.05")
        self.input_steps_per_cycle = QLineEdit("1000")
        self.input_cycles = QLineEdit("5")
        self.input_freq_logging = QLineEdit("500")
        self.input_speed = QLineEdit("0.5")
        self.input_acceleration = QLineEdit("2.0")

        form_layout.addRow("Amplitudes (m, comma-separated):", self.input_amps)
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
        self.btn_move_rec = QPushButton("Move and Record")  # New button
        self.btn_stop_move_rec = QPushButton("Stop Move and Record")  # New button
        self.btn_show_data = QPushButton("Show Current Data")  # New button
        self.btn_set_zero_tcp = QPushButton("Set Zero TCP")  # New button
        self.btn_set_zero_force = QPushButton("Set Zero Force")  # New button

        self.btn_start.clicked.connect(self.start_robot)
        self.btn_stop.clicked.connect(self.stop_robot)
        self.btn_move_rec.clicked.connect(self.move_and_record)  # Connect new button
        self.btn_stop_move_rec.clicked.connect(self.stop_move_and_record)  # Connect new button
        self.btn_show_data.clicked.connect(self.show_current_data)  # Connect new button
        self.btn_set_zero_tcp.clicked.connect(self.set_zero_tcp)  # Connect new button
        self.btn_set_zero_force.clicked.connect(self.set_zero_force)  # Connect new button

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_move_rec.setEnabled(False)  # Disable new button
        self.btn_stop_move_rec.setEnabled(False)  # Disable new button
        self.btn_show_data.setEnabled(False)  # Disable new button
        self.btn_set_zero_tcp.setEnabled(False)  # Disable new button
        self.btn_set_zero_force.setEnabled(False)  # Disable new button

        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_stop)
        btn_layout.addWidget(self.btn_move_rec)  # Add new button to layout
        btn_layout.addWidget(self.btn_stop_move_rec)  # Add new button to layout
        btn_layout.addWidget(self.btn_show_data)  # Add new button to layout
        btn_layout.addWidget(self.btn_set_zero_tcp)  # Add new button to layout
        btn_layout.addWidget(self.btn_set_zero_force)  # Add new button to layout
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
            self.btn_move_rec.setEnabled(True)  # Enable new button
            self.btn_show_data.setEnabled(True)  # Enable new button
            self.btn_set_zero_tcp.setEnabled(True)  # Enable new button
            self.btn_set_zero_force.setEnabled(True)  # Enable new button
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", f"Failed to connect to {ip}\nError: {e}")
            self.rtde_c = None
            self.rtde_r = None
            self.status_label.setText("Status: Disconnected")
            self.btn_start.setEnabled(False)
            self.btn_move_rec.setEnabled(False)  # Disable new button
            self.btn_show_data.setEnabled(False)  # Disable new button
            self.btn_set_zero_tcp.setEnabled(False)  # Disable new button
            self.btn_set_zero_force.setEnabled(False)  # Disable new button

    def start_robot(self):
        """Starts the robot movement without recording"""
        if not self.rtde_c or not self.rtde_r:
            QMessageBox.warning(self, "Warning", "Not connected to robot!")
            return

        try:
            self.stop_event.clear()

            # Get user inputs
            amplitudes = [float(amp.strip()) for amp in self.input_amps.text().split(",")]
            T_motion = float(self.input_T_motion.text())
            steps_per_cycle = int(self.input_steps_per_cycle.text())
            cycles = int(self.input_cycles.text())
            speed = float(self.input_speed.text())
            acceleration = float(self.input_acceleration.text())

            freq_motion = 1 / T_motion
            dt_motion = T_motion / steps_per_cycle
            total_steps = cycles * steps_per_cycle

            self.status_label.setText("Status: Running...")
            self.btn_start.setEnabled(False)
            self.btn_stop.setEnabled(True)

            for amp in amplitudes:
                if self.stop_event.is_set():
                    break

                self.status_label.setText(f"Status: Running... Amplitude: {amp} m")

                # Start Thread
                robot_thread = threading.Thread(target=self.move_robot, args=(amp, freq_motion, dt_motion, total_steps, speed, acceleration))
                robot_thread.start()
                robot_thread.join()

            self.status_label.setText("Status: Completed")
            self.btn_start.setEnabled(True)
            self.btn_stop.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start: {str(e)}")

    def stop_robot(self):
        """Stops the robot movement"""
        self.stop_event.set()
        self.status_label.setText("Status: Stopped")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def move_and_record(self):
        """Starts the robot movement and logging"""
        if not self.rtde_c or not self.rtde_r:
            QMessageBox.warning(self, "Warning", "Not connected to robot!")
            return

        try:
            self.stop_event.clear()
            self.data = []  # Clear previous data

            # Get user inputs
            amplitudes = [float(amp.strip()) for amp in self.input_amps.text().split(",")]
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

            self.status_label.setText("Status: Running and Recording...")
            self.btn_move_rec.setEnabled(False)
            self.btn_stop_move_rec.setEnabled(True)

            for amp in amplitudes:
                if self.stop_event.is_set():
                    break

                self.data = []  # Clear data for each amplitude
                self.status_label.setText(f"Status: Running... Amplitude: {amp} m")

                # Start Threads
                robot_thread = threading.Thread(target=self.move_robot, args=(amp, freq_motion, dt_motion, total_steps, speed, acceleration))
                log_thread = threading.Thread(target=self.log_data, args=(dt_logging,))

                robot_thread.start()
                log_thread.start()

                robot_thread.join()
                log_thread.join()

                self.export_csv(amp)

            self.status_label.setText("Status: Completed")
            self.btn_move_rec.setEnabled(True)
            self.btn_stop_move_rec.setEnabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start: {str(e)}")

    def stop_move_and_record(self):
        """Stops the robot movement and logging"""
        self.stop_event.set()
        self.status_label.setText("Status: Stopped")
        self.btn_move_rec.setEnabled(True)
        self.btn_stop_move_rec.setEnabled(False)
        self.export_csv()

    def move_robot(self, amp, freq_motion, dt_motion, total_steps, speed, acceleration):
        """Moves the robot in a sinusoidal Z motion"""
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

            self.status_label.setText(f"Status: Running... Amplitude: {amp} m, Step: {i+1}/{total_steps}")

        self.stop_event.set()

    def log_data(self, dt_logging):
        """Logs robot data"""
        start_global_time = time.time()
        
        while not self.stop_event.is_set():
            start_time = time.time()

            actual_tcp_pose = self.rtde_r.getActualTCPPose()
            actual_joint_positions = self.rtde_r.getActualQ()
            actual_tcp_force = self.rtde_r.getActualTCPForce()

            if self.zero_tcp:
                actual_tcp_pose = [actual_tcp_pose[i] - self.zero_tcp[i] for i in range(6)]
            if self.zero_force:
                actual_tcp_force = [actual_tcp_force[i] - self.zero_force[i] for i in range(6)]

            timestamp = time.time() - start_global_time
            self.data.append([timestamp, *actual_joint_positions, *actual_tcp_pose, *actual_tcp_force])

            elapsed_time = time.time() - start_time
            remaining_time = dt_logging - elapsed_time
            if remaining_time > 0:
                time.sleep(remaining_time)

    def update_status(self):
        """Updates the GUI with the latest recorded data"""
        if self.data:
            latest_data = self.data[-1]
            tcp_x, tcp_y, tcp_z = latest_data[7:10]
            force_x, force_y, force_z = latest_data[13:16]
            self.data_label.setText(f"TCP: ({tcp_x:.3f}, {tcp_y:.3f}, {tcp_z:.3f}), Force: ({force_x:.3f}, {force_y:.3f}, {force_z:.3f})")
        else:
            self.data_label.setText("Data: N/A")

    def show_current_data(self):
        """Shows the current TCP position and force"""
        if not self.rtde_r:
            QMessageBox.warning(self, "Warning", "Not connected to robot!")
            return

        try:
            actual_tcp_pose = self.rtde_r.getActualTCPPose()
            actual_tcp_force = self.rtde_r.getActualTCPForce()

            tcp_x, tcp_y, tcp_z = actual_tcp_pose[:3]
            force_x, force_y, force_z = actual_tcp_force[:3]

            QMessageBox.information(self, "Current Data", 
                                    f"TCP Position: (X: {tcp_x:.3f}, Y: {tcp_y:.3f}, Z: {tcp_z:.3f})\n"
                                    f"TCP Force: (X: {force_x:.3f}, Y: {force_y:.3f}, Z: {force_z:.3f})")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to retrieve data: {str(e)}")

    def set_zero_tcp(self):
        """Sets the current TCP position as zero"""
        if not self.rtde_r:
            QMessageBox.warning(self, "Warning", "Not connected to robot!")
            return

        try:
            self.zero_tcp = self.rtde_r.getActualTCPPose()
            QMessageBox.information(self, "Set Zero TCP", "Current TCP position set as zero.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to set zero TCP: {str(e)}")

    def set_zero_force(self):
        """Sets the current TCP force as zero"""
        if not self.rtde_r:
            QMessageBox.warning(self, "Warning", "Not connected to robot!")
            return

        try:
            self.zero_force = self.rtde_r.getActualTCPForce()
            QMessageBox.information(self, "Set Zero Force", "Current TCP force set as zero.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to set zero force: {str(e)}")

    def export_csv(self, amp=None):
        """Exports collected data to CSV"""
        if not self.data:
            QMessageBox.warning(self, "Warning", "No data to export!")
            return

        if amp is not None:
            amp_str = str(amp).replace(".", "_")
            filename = f"UR5e_Amp_{amp_str}.csv"
        else:
            amp = self.input_amp.text().replace(".", "_")
            filename = f"UR5e_Amp_{amp}.csv"
        
        count = 1

        while os.path.exists(filename):
            filename = f"{filename.split('.')[0]}_{count}.csv"
            count += 1

        columns = ["timestamp"] + [f"position_{i}" for i in range(6)] + \
                  ["tcp_pos_x", "tcp_pos_y", "tcp_pos_z", "tcp_ori_x", "tcp_ori_y", "tcp_ori_z"] + \
                  ["force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"]

        df = pd.DataFrame(self.data, columns=columns)
        df.to_csv(filename, index=False)

        QMessageBox.information(self, "Export", f"Data saved as {filename}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = UR5eControlGUI()
    window.show()
    sys.exit(app.exec())
