#!/usr/bin/env python3
# -*-coding:utf8-*-
# Record continuous trajectory
import os, time
from piper_sdk import *

if __name__ == "__main__":
    # Whether there is a gripper
    have_gripper = True
    # Maximum recording time in seconds (0 = unlimited, stop by terminating program)
    record_time = 10.0
    # Teach mode detection timeout in seconds
    timeout = 10.0
    # CSV file path for saving trajectory
    CSV_path = os.path.join(os.path.dirname(__file__), "trajectory.csv")
    # Initialize and connect to robotic arm
    piper = Piper("can0")
    interface = piper.init()
    piper.connect()
    time.sleep(0.1)

    def get_pos():
        '''Get current joint angles and gripper opening distance'''
        joint_state = piper.get_joint_states()[0]
        if have_gripper:
            return joint_state + (piper.get_gripper_states()[0][0], )
        return joint_state
    
    print("step 1: Press teach button to enter teach mode")
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 2:
        if over_time < time.time():
            print("ERROR: Teach mode detection timeout. Please check if teach mode is enabled")
            exit()
        time.sleep(0.01)

    input("step 2: Press Enter to start recording trajectory")
    csv = open(CSV_path, "w")
    last_pos = get_pos()
    last_time = time.time()
    over_time = last_time + record_time
    while record_time == 0 or time.time() < over_time:
        current_pos = get_pos()
        if current_pos != last_pos:  # Record only when position changes
            wait_time = round(time.time() - last_time, 4)
            print(f"INFO: Wait time: {wait_time:0.4f}s, current position: {current_pos}")
            last_pos = current_pos
            last_time = time.time()
            csv.write(f"{wait_time}," + ",".join(map(str, current_pos)) + "\n")
        time.sleep(0.01)
    csv.close()
    print("INFO: Recording complete. Press teach button again to exit teach mode")