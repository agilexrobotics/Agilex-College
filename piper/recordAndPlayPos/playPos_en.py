#!/usr/bin/env python3
# -*-coding:utf8-*-
# Play positions
import os, time, csv
from piper_sdk import Piper

if __name__ == "__main__":
    # Whether there is a gripper
    have_gripper = True
    # Number of playbacks, 0 means infinite loop
    play_times = 1
    # Playback interval, unit: second; negative value means manual key control
    play_interval = 0
    # Movement speed percentage, recommended range: 10-100
    move_spd_rate_ctrl = 100
    # Timeout for switching to CAN mode, unit: second
    timeout = 5.0
    # CSV file path for saving positions
    CSV_path = os.path.join(os.path.dirname(__file__), "pos.csv")
    # Read the position file
    try:
        with open(CSV_path, 'r', encoding='utf-8') as f:
            track = list(csv.reader(f))
            if not track:
                print("ERROR: Position file is empty")
                exit()
            track = [[float(j) for j in i] for i in track]    # Convert to a list of floating-point numbers
    except FileNotFoundError:
        print("ERROR: Position file does not exist")
        exit()

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

    def stop():
        '''Stop the robotic arm; this function must be called first when exiting the teaching mode for the first time to control the robotic arm in CAN mode'''
        interface.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # The robotic arm can be restored only when the radians of joints 2, 3, and 5 are within the limit range to prevent damage caused by falling from a large radian
        pos = get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and pos[4] < limit_angle[1] and pos[4] > limit_angle[2]):
            time.sleep(0.01)
            pos = get_pos()
        # Restore the robotic arm
        piper.disable_arm()
        time.sleep(1.0)
    
    def enable():
        '''Enable the robotic arm and gripper'''
        while not piper.enable_arm():
            time.sleep(0.01)
        if have_gripper:
            time.sleep(0.01)
            piper.enable_gripper()
        interface.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        print("INFO: Enable successful")

    print("step 1:  Please ensure the robotic arm has exited the teaching mode before playback")
    if interface.GetArmStatus().arm_status.ctrl_mode != 1:
        stop()  # This function must be called first when exiting the teaching mode for the first time to switch to CAN mode
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 1:
        if over_time < time.time():
            print("ERROR: Failed to switch to CAN mode, please check if the teaching mode is exited")
            exit()
        interface.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        time.sleep(0.01)
    
    enable()
    count = 0
    input("step 2: Press Enter to start playing positions")
    while play_times == 0 or abs(play_times) != count:
        for n, pos in enumerate(track):
            while True:
                piper.move_j(pos[:-1], move_spd_rate_ctrl)
                time.sleep(0.01)
                current_pos = get_pos()
                print(f"INFO: {count + 1}th playback, {n + 1}th position, current position: {current_pos}, target position: {pos}")
                if all(abs(current_pos[i] - pos[i]) < 0.0698 for i in range(6)):
                    break
            if have_gripper and len(pos) == 7:
                piper.move_gripper(pos[-1], 1)
                time.sleep(0.5)
            if play_interval < 0:
                if n != len(track) - 1 and input("INPUT: Enter 'q' to exit, press Enter directly to play:  ") == 'q':
                    exit()
            else:
                time.sleep(play_interval)
        count += 1