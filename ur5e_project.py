import rtde_control
import rtde_receive
import time
import math
import threading
import pandas as pd
import matplotlib.pyplot as plt

# ตั้งค่า IP ของหุ่นยนต์ UR5e
ROBOT_IP = "192.168.1.102"  # เปลี่ยนเป็น IP ของคุณ

# เชื่อมต่อกับ UR5e
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

# ตั้งค่าพารามิเตอร์การเคลื่อนที่
dept = 1 # cm dept press
amp = dept/2/100  # ความสูงของการเคลื่อนที่ (m)
T_motion = 1.0  # ค่าคาบของการเคลื่อนที่ (1 รอบใช้เวลา 1 วินาที)
freq_motion = 1 / T_motion  # ความถี่ของการเคลื่อนที่ (Hz)
steps_per_cycle = 100  # จำนวนสเต็ปต่อ 1 รอบ
dt_motion = T_motion / steps_per_cycle  # เวลาต่อสเต็ปของการเคลื่อนที่

cycles = 5  # จำนวนรอบที่ต้องการให้เคลื่อนที่
total_steps = cycles * steps_per_cycle  # จำนวนสเต็ปทั้งหมด

# ตั้งค่าความถี่ในการบันทึกข้อมูล
freq_logging = 50  # Hz (เช่น บันทึกข้อมูลที่ 50Hz)
dt_logging = 1 / freq_logging  # เวลาต่อ 1 การบันทึกข้อมูล

# อ่านตำแหน่งปัจจุบันของ TCP
start_pose = rtde_r.getActualTCPPose()

# เตรียมเก็บข้อมูล
data = []
stop_threads = False  # ใช้สำหรับหยุด Thread

# -------------------------- ฟังก์ชันควบคุมการเคลื่อนที่ --------------------------
def move_robot():
    global stop_threads
    start_global_time = time.time()  # จับเวลาเริ่มต้น
    for i in range(total_steps):
        if stop_threads:
            break  # หยุด Loop ถ้าได้รับสัญญาณให้หยุด

        start_time = time.time()

        # คำนวณตำแหน่ง Z ใหม่ตามคลื่นไซน์
        z_offset = amp * math.sin(2 * math.pi * freq_motion * (i / steps_per_cycle))
        new_pose = start_pose.copy()
        new_pose[2] += z_offset

        # ส่งคำสั่งเคลื่อนที่
        rtde_c.moveL(new_pose, speed=0.5, acceleration=2.0)

        # คำนวณเวลาที่ใช้จริง และปรับ sleep ให้คงที่
        elapsed_time = time.time() - start_time
        remaining_time = dt_motion - elapsed_time
        if remaining_time > 0:
            time.sleep(remaining_time)

    stop_threads = True  # แจ้งให้ Thread อื่นหยุดทำงาน

# -------------------------- ฟังก์ชันบันทึกข้อมูล --------------------------
def log_data():
    global stop_threads
    start_global_time = time.time()
    
    while not stop_threads:
        start_time = time.time()
        
        # อ่านค่าปัจจุบันจาก UR5e
        actual_tcp_pose = rtde_r.getActualTCPPose()
        actual_joint_positions = rtde_r.getActualQ()  # Joint Positions
        actual_joint_velocities = rtde_r.getActualQd()  # Joint Velocities
        actual_joint_torques = rtde_r.getActualCurrentQ()  # Joint Effort (Torques)
        actual_tcp_force = rtde_r.getActualTCPForce()  # TCP Forces
        tool_current = rtde_r.getActualToolCurrent()  # Tool Current
        speed_scaling = rtde_r.getSpeedScaling()  # Speed Scaling

        # บันทึกข้อมูล
        timestamp = time.time() - start_global_time
        data.append([
            timestamp, 
            *actual_joint_positions, 
            *actual_joint_velocities, 
            *actual_joint_torques, 
            *actual_tcp_force,
            *actual_tcp_pose, 
            tool_current, 
            speed_scaling
        ])

        # คำนวณเวลาที่ใช้จริง และหน่วงเวลาที่เหลือ
        elapsed_time = time.time() - start_time
        remaining_time = dt_logging - elapsed_time
        if remaining_time > 0:
            time.sleep(remaining_time)

# -------------------------- รันทั้ง 2 Thread พร้อมกัน --------------------------
robot_thread = threading.Thread(target=move_robot)
log_thread = threading.Thread(target=log_data)

robot_thread.start()
log_thread.start()

robot_thread.join()
log_thread.join()

# -------------------------- บันทึกข้อมูลลง CSV --------------------------
columns = ["timestamp"] + \
          [f"position_{i}" for i in range(6)] + \
          [f"velocity_{i}" for i in range(6)] + \
          [f"effort_{i}" for i in range(6)] + \
          ["force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"] + \
          ["tcp_pos_x", "tcp_pos_y", "tcp_pos_z", "tcp_ori_x", "tcp_ori_y", "tcp_ori_z"] + \
          ["tool_current", "speed_scaling"]

df = pd.DataFrame(data, columns=columns)
df.to_csv("ur5e_full_data.csv", index=False)
print("✅ บันทึกข้อมูลสำเร็จ: ur5e_full_data.csv")

# -------------------------- แสดงกราฟ --------------------------
plt.figure(figsize=(10, 5))
plt.plot(df["timestamp"], df["tcp_pos_z"], label="TCP Z Position")
plt.xlabel("Time (s)")
plt.ylabel("TCP Z Position (m)")
plt.title("UR5e TCP Z Position Over Time")
plt.legend()
plt.grid()
plt.show()
