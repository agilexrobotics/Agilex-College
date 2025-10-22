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
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.1)

    def get_pos():
        '''Get current joint angles and gripper opening distance'''
        joint_state = piper.GetArmJointMsgs().joint_state
        joint_state = tuple(getattr(joint_state, f"joint_{i+1}") / 1e3 * 0.0174533 for i in range(6))
        if have_gripper:
            return joint_state + (piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1e6, )
        return joint_state
    
    print("step 1: Press teach button to enter teach mode")
    over_time = time.time() + timeout
    while piper.GetArmStatus().arm_status.ctrl_mode != 2:
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
            wait_time = time.time() - last_time
            print(f"INFO: Wait time: {wait_time:0.4f}s, current position: {current_pos}")
            last_pos = current_pos
            last_time = time.time()
            csv.write(f"{wait_time}," + ",".join(map(str, current_pos)) + "\n")
        time.sleep(0.01)
    csv.close()
    print("INFO: Recording complete. Press teach button again to exit teach mode")