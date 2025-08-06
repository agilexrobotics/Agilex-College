#!/usr/bin/env python3
# -*-coding:utf8-*-
# Play continuous trajectory
import os, time, csv
from piper_sdk import *

if __name__ == "__main__":
    # Whether there is a gripper
    have_gripper = True
    # Playback times (0 means infinite loop)
    play_times = 1
    # Playback interval in seconds
    play_interval = 1.0
    # Motion speed percentage (recommended range: 10-100)
    move_spd_rate_ctrl = 100
    # Playback speed multiplier (recommended range: 0.1-2)
    play_speed = 1.0
    # CAN mode switch timeout in seconds
    timeout = 5.0
    # CSV file path for saved trajectory
    CSV_path = os.path.join(os.path.dirname(__file__), "trajectory.csv")
    # Read trajectory file
    try:
        with open(CSV_path, 'r', encoding='utf-8') as f:
            track = list(csv.reader(f))
            if not track:
                print("ERROR: Trajectory file is empty")
                exit()
            track = [[float(j) for j in i] for i in track]    # Convert to float lists
    except FileNotFoundError:
        print("ERROR: Trajectory file not found")
        exit()

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
    
    def stop():
        '''Stop robotic arm; must call this function when first exiting teach mode before using CAN mode'''
        interface.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # Arm only restored when joints 2,3,5 are within safe range
        pos = get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and pos[4] < limit_angle[1] and pos[4] > limit_angle[2]):
            time.sleep(0.01)
            pos = get_pos()
        # Restore arm
        piper.disable_arm()
        time.sleep(1.0)
    
    def enable():
        '''Enable robotic arm and gripper'''
        while not piper.enable_arm():
            time.sleep(0.01)
        if have_gripper:
            time.sleep(0.01)
            piper.enable_gripper()
        interface.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        print("INFO: Enable successful")

    print("step 1: Ensure robotic arm has exited teach mode before playback")
    if interface.GetArmStatus().arm_status.ctrl_mode != 1:
        stop()  # Required when first exiting teach mode
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 1:
        if over_time < time.time():
            print("ERROR: CAN mode switch failed. Please confirm teach mode is exited")
            exit()
        interface.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        time.sleep(0.01)
    
    enable()
    count = 0
    input("step 2: Press Enter to start trajectory playback")
    while play_times == 0 or abs(play_times) != count:
        for n, pos in enumerate(track):
            piper.move_j(pos[1:-1], move_spd_rate_ctrl)
            if have_gripper and len(pos) == 8:
                piper.move_gripper(pos[-1], 1)
            print(f"INFO: Playback #{count + 1}, wait time: {pos[0] / play_speed:0.4f}s, target position: {pos[1:]}")
            if n == len(track) - 1:
                time.sleep(play_interval)  # Final point delay
            else:
                time.sleep(pos[0] / play_speed)  # Point-to-point delay
        count += 1