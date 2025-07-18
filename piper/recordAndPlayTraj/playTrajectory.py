#!/usr/bin/env python3
# -*-coding:utf8-*-
# 播放连续轨迹
import os, time, csv
from piper_sdk import *

if __name__ == "__main__":
    # 是否有夹爪
    have_gripper = True
    # 播放次数 0表示无限循环
    play_times = 1
    # 播放间隔，单位：秒
    play_interval = 1.0
    # 运动速度百分比，建议范围：10-100
    move_spd_rate_ctrl = 100
    # 播放倍速，建议范围：0.1-2
    play_speed = 1.0
    # 切换CAN模式超时时间，单位：秒
    timeout = 5.0
    # 保存轨迹的CSV文件路径
    CSV_path = os.path.join(os.path.dirname(__file__), "trajectory.csv")
    # 读取轨迹文件
    try:
        with open(CSV_path, 'r', encoding='utf-8') as f:
            track = list(csv.reader(f))
            if not track:
                print("ERROR: 轨迹文件为空")
                exit()
            track = [[float(j) for j in i] for i in track]    # 转换为浮点数列表
    except FileNotFoundError:
        print("ERROR: 轨迹文件不存在")
        exit()

    # 初始化并连接机械臂
    piper = Piper("can0")
    interface = piper.init()
    piper.connect()
    time.sleep(0.1)

    def get_pos():
        '''获取机械臂当前关节弧度和夹爪弧度'''
        joint_state = piper.get_joint_states()[0]
        if have_gripper:
            return joint_state + (piper.get_gripper_states()[0][0], )
        return joint_state
    
    def stop():
        '''停止机械臂；初次退出示教模式需先调用此函数才能使用CAN模式控制机械臂'''
        interface.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # 2、3、5关节弧度在限制范围内时才恢复机械臂，防止大弧度直接掉落造成损坏
        pos = get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and pos[4] < limit_angle[1] and pos[4] > limit_angle[2]):
            time.sleep(0.01)
            pos = get_pos()
        # 恢复机械臂
        piper.disable_arm()
        time.sleep(1.0)
    
    def enable():
        '''使能机械臂和夹爪'''
        while not piper.enable_arm():
            time.sleep(0.01)
        if have_gripper:
            time.sleep(0.01)
            piper.enable_gripper()
        interface.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        print("INFO: 使能成功")

    print("step 1: 播放前请确保机械臂已退出示教模式")
    if interface.GetArmStatus().arm_status.ctrl_mode != 1:
        stop()  # 初次退出示教模式需先调用此函数才能切换至CAN模式
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 1:
        if over_time < time.time():
            print("ERROR: CAN模式切换失败，请检查是否退出示教模式")
            exit()
        interface.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        time.sleep(0.01)
    
    enable()
    count = 0
    input("step 2: 回车开始播放轨迹")
    while play_times == 0 or abs(play_times) != count:
        for n, pos in enumerate(track):
            piper.move_j(pos[1:-1], move_spd_rate_ctrl)
            if have_gripper and len(pos) == 8:
                piper.move_gripper(pos[-1], 1)
            print(f"INFO: 第{count + 1}次播放，等待时间:{pos[0] / play_speed : 0.4f}s，目标位置: {pos[1:]}")
            if n == len(track) - 1:
                time.sleep(play_interval)
            else:
                time.sleep(pos[0] / play_speed)
        count += 1