#!/usr/bin/env python3
# -*-coding:utf8-*-
# Record points
import os, time
from piper_sdk import *

if __name__ == "__main__":
    # Whether there is a gripper
    have_gripper = True
    # Teach mode detection timeout (seconds)
    timeout = 10.0
    # CSV file path for saving points
    CSV_path = os.path.join(os.path.dirname(__file__), "pos.csv")
    # Initialize and connect to the robotic arm
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
    
    print("INFO: Please press the teach button to enter teach mode")
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 2:
        if over_time < time.time():
            print("ERROR: Teach mode detection timeout. Please check if teach mode is enabled")
            exit()
        time.sleep(0.01)

    count = 1
    csv = open(CSV_path, "w")
    while input("INPUT: Enter 'q' to quit, press Enter to record: ") != "q":
        current_pos = get_pos()
        print(f"INFO: Point {count}, recorded position: {current_pos}")
        csv.write(",".join(map(str, current_pos)) + "\n")
        count += 1
    csv.close()
    print("INFO: Recording completed. Press the teach button again to exit teach mode")