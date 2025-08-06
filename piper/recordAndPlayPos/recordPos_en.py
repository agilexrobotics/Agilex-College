#!/usr/bin/env python3
# -*-coding:utf8-*-
# Record positions
import os, time
from piper_sdk import *

if __name__ == "__main__":
    # Whether there is a gripper
    have_gripper = True
    # Timeout for teaching mode detection, unit: second
    timeout = 10.0
    # CSV file path for saving positions
    CSV_path = os.path.join(os.path.dirname(__file__), "pos.csv")
    # Initialize and connect the robotic arm
    piper = Piper("can0")
    interface = piper.init()
    piper.connect()
    time.sleep(0.1)

    def get_pos():
        '''Get the current joint radians of the robotic arm and the gripper opening distance'''
        joint_state = piper.get_joint_states()[0]
        if have_gripper:
            return joint_state + (piper.get_gripper_states()[0][0], )
        return joint_state
    
    print("INFO: Please click the teach button to enter the teaching mode")
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 2:
        if over_time < time.time():
            print("ERROR:Teaching mode detection timeout, please check whether the teaching mode is enabled")
            exit()
        time.sleep(0.01)

    count = 1
    csv = open(CSV_path, "w")
    while input("INPUT: Enter q to exit, press Enter directly to record:  ") != "q":
        current_pos = get_pos()
        print(f"INFO:  {count}th position, recorded position: {current_pos}")
        csv.write(",".join(map(str, current_pos)) + "\n")
        count += 1
    csv.close()
    print("INFO: Recording ends, click the teach button again to exit the teaching mode")