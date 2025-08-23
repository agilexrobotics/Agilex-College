import asyncio
import websockets
import socket
import json
import math
import time
import numpy as np
from piper_sdk import *


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def quat_from_euler(pitch, roll, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    q0 = cy * cp * cr + sy * sp * sr
    q1 = cy * sp * cr + sy * cp * sr
    q2 = sy * cp * cr - cy * sp * sr
    q3 = cy * cp * sr - sy * sp * cr
    return np.array([q0, q1, q2, q3])

def euler_from_quat(q):
    q0, q1, q2, q3 = q
    roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    pitch = np.arcsin(2*(q0*q2 - q3*q1))
    yaw = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    return pitch, roll, yaw

def normalize_quat(q):
    return q / np.linalg.norm(q)


class EKFAHRS:
    def __init__(self):
        # State vector: [q0, q1, q2, q3, bgx, bgy, bgz]
        self.x = np.zeros(7)
        self.x[0] = 1.0  # Initial quaternion
        self.P = np.eye(7)
        # Noise parameters
        self.Q = np.diag([1e-6]*4 + [1e-8]*3)
        self.R = np.diag([1e-1]*3)
        self.g = 9.81

    def set_init(self, acc, mag):
        # acc: 3, mag: 3, units g and μT
        acc = acc / np.linalg.norm(acc)
        pitch = np.arcsin(-acc[0])
        roll = np.arctan2(acc[1], acc[2])
        # Initialize yaw using magnetometer
        mx, my, mz = mag
        mag_x = mx * np.cos(pitch) + my * np.sin(roll) * np.sin(pitch) + mz * np.cos(roll) * np.sin(pitch)
        mag_y = my * np.cos(roll) - mz * np.sin(roll)
        yaw = np.arctan2(-mag_y, mag_x)
        q = quat_from_euler(pitch, roll, yaw)
        self.x[:4] = normalize_quat(q)

    def predict(self, gyro, dt):
        # gyro: rad/s, dt: s
        q = self.x[:4]
        bg = self.x[4:]
        omega = gyro - bg
        wx, wy, wz = omega
        Omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        dq = 0.5 * Omega @ q
        q_new = q + dq * dt
        q_new = normalize_quat(q_new)
        self.x[:4] = q_new
        # Approximate state transition matrix
        F = np.eye(7)
        F[:4, :4] += 0.5 * Omega * dt
        self.P = F @ self.P @ F.T + self.Q

    def update(self, acc):
        # acc: 3, unit g
        q = self.x[:4]
        # Observation model: acc = R(q)^T * [0,0,1]
        hx = np.array([
            2*(q[1]*q[3] - q[0]*q[2]),
            2*(q[0]*q[1] + q[2]*q[3]),
            q[0]**2 + q[3]**2 - q[1]**2 - q[2]**2
        ])
        acc = acc / np.linalg.norm(acc)
        y = acc - hx
        # Jacobian matrix
        H = np.zeros((3,7))
        q0, q1, q2, q3 = q
        H[0,0] = -2*q2
        H[0,1] = 2*q3
        H[0,2] = -2*q0
        H[0,3] = 2*q1
        H[1,0] = 2*q1
        H[1,1] = 2*q0
        H[1,2] = 2*q3
        H[1,3] = 2*q2
        H[2,0] = 2*q0
        H[2,1] = -2*q1
        H[2,2] = -2*q2
        H[2,3] = 2*q3
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        self.x += dx
        self.x[:4] = normalize_quat(self.x[:4])
        self.P = (np.eye(7) - K @ H) @ self.P

    def get_euler(self):
        return euler_from_quat(self.x[:4])


async def echo(websocket):
    global last_time, ekf_initialized
    async for message in websocket:
        try:
            data = json.loads(message)
            sensor_name = data.get("SensorName", "").lower()
            timestamp = data.get("Timestamp", 0)

            if sensor_name in sensor_data:
                sensor_data[sensor_name]['x'] = float(data.get('x', 0))
                sensor_data[sensor_name]['y'] = float(data.get('y', 0))
                sensor_data[sensor_name]['z'] = float(data.get('z', 0))
                sensor_data[sensor_name]['timestamp'] = timestamp

                # Check if all three types of data are fresh
                accel_ts = sensor_data['accelerometer']['timestamp']
                gyro_ts = sensor_data['gyroscope']['timestamp']
                mag_ts = sensor_data['magnetometer']['timestamp']
                if abs(accel_ts - gyro_ts) < 100 and abs(accel_ts - mag_ts) < 100:
                    current_time = time.time()
                    dt = current_time - last_time
                    last_time = current_time

                    # Unit Explanation:
                    # Accelerometer ax, ay, az units are g (gravity)
                    # Gyroscope gx, gy, gz units are rad/s
                    # Magnetometer mx, my, mz units are μT
                    gx = sensor_data['gyroscope']['x']
                    gy = sensor_data['gyroscope']['y']
                    gz = sensor_data['gyroscope']['z']
                    ax = sensor_data['accelerometer']['x']
                    ay = sensor_data['accelerometer']['y']
                    az = sensor_data['accelerometer']['z']
                    mx = sensor_data['magnetometer']['x']
                    my = sensor_data['magnetometer']['y']
                    mz = sensor_data['magnetometer']['z']

                    # EKF Initialization
                    if not ekf_initialized:
                        ekf.set_init(np.array([ax, ay, az]), np.array([mx, my, mz]))
                        ekf_initialized = True

                    # EKF Calculation
                    ekf.predict(np.array([gx, gy, gz]), dt)
                    ekf.update(np.array([ax, ay, az]))
                    epitch, eroll, eyaw = ekf.get_euler()
                    print(f"EKF - Roll: {math.degrees(eroll):.2f}, Pitch: {math.degrees(epitch):.2f}, Yaw: {math.degrees(eyaw):.2f}")

                    # Send results to the robotic arm
                    ctl = [round(initial_position[0] * 1000),
                           round(initial_position[1] * 1000),
                           round(initial_position[2] * 1000),
                           round(math.degrees(eroll) * 1000),
                           round(math.degrees(epitch) * 1000),
                           round(math.degrees(eyaw) * 1000)]
                    robot.EndPoseCtrl(*ctl)

        except Exception as e:
            print(f"Error: {e}")

async def main():
    port = 5000
    async with websockets.serve(echo, '0.0.0.0', port, max_size=1_000_000_000):
        await asyncio.Future()


if __name__ == "__main__":
    robot = C_PiperInterface_V2()
    robot.ConnectPort()

    # Enable Piper
    while not robot.EnablePiper():
        time.sleep(0.01)
        
    # Set control mode
    mode = 0xAD
    spd = 100
    robot.MotionCtrl_2(0x01, 0x00, spd, mode)
    time.sleep(0.1)
    robot.MotionCtrl_2(0x01, 0x00, spd, mode)

    # Set initial position
    initial_position = [0, 0, 500]  # X, Y, Z (0.001mm)
    # Send initial position to the robotic arm
    robot.EndPoseCtrl(
                    initial_position[0] * 1000,     # X
                    initial_position[1] * 1000,     # Y
                    initial_position[2] * 1000,     # Z
                    0 * 1000,                       # RX
                    0 * 1000,                       # RY
                    0 * 1000                        # RZ
                )
    
    # Sensor data cache
    sensor_data = {
        'accelerometer': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'timestamp': 0},
        'gyroscope': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'timestamp': 0},
        'magnetometer': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'timestamp': 0},
    }

    last_time = time.time()
    ekf = EKFAHRS()
    ekf_initialized = False
    
    hostname = socket.gethostname()
    IPAddr = get_ip()
    port = 5000
    print("Your Computer Name is: " + hostname)
    print("Your Computer IP Address is: " + IPAddr)
    print("* Enter {0}:{1} in the app.\n* Press the 'Set IP Address' button.\n* Select the sensors to stream.\n* Update the 'update interval' by entering a value in ms.".format(IPAddr, port))

    # Start WebSocket server
    asyncio.run(main())