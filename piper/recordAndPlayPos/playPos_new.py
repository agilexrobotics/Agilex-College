#!/usr/bin/env python3
# -*-coding:utf8-*-
# 播放点位
import os, time, csv
from piper_sdk import *

if __name__ == "__main__":
    # 是否有夹爪
    have_gripper = True
    # 播放次数，0表示无限循环
    play_times = 1
    # 播放间隔，单位：秒，负数表示人工按键控制
    play_interval = 0
    # 运动速度百分比，建议范围：10-100
    move_spd_rate_ctrl = 100
    # 切换CAN模式超时时间，单位：秒
    timeout = 5.0
    # 保存点位的CSV文件路径
    CSV_path = os.path.join(os.path.dirname(__file__), "pos.csv")
    # 读取点位文件
    try:
        with open(CSV_path, 'r', encoding='utf-8') as f:
            track = list(csv.reader(f))
            if not track:
                print("ERROR: 点位文件为空")
                exit()
            track = [[float(j) for j in i] for i in track]    # 转换为浮点数列表
    except FileNotFoundError:
        print("ERROR: 点位文件不存在")
        exit()

    # 初始化并连接机械臂
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.1)

    def get_pos():
        '''获取机械臂当前关节弧度和夹爪张开距离'''
        joint_state = piper.GetArmJointMsgs().joint_state
        joint_state = tuple(getattr(joint_state, f"joint_{i+1}") / 1e3 * 0.0174533 for i in range(6))
        if have_gripper:
            return joint_state + (piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1e6, )
        return joint_state
    
    def stop():
        '''停止机械臂；初次退出示教模式需先调用此函数才能使用CAN模式控制机械臂'''
        piper.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # 2、3、5关节弧度在限制范围内时才恢复机械臂，防止大弧度直接掉落造成损坏
        pos = get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and pos[4] < limit_angle[1] and pos[4] > limit_angle[2]):
            time.sleep(0.01)
            pos = get_pos()
        # 恢复机械臂
        piper.EmergencyStop(0x02)
        time.sleep(1.0)
    
    def enable():
        '''使能机械臂和夹爪'''
        while not piper.EnablePiper():
            time.sleep(0.01)
        if have_gripper:
            time.sleep(0.01)
            piper.GripperCtrl(0, 1000, 0x01, 0x00)
        piper.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        print("INFO: 使能成功")

    print("step 1: 播放前请确保机械臂已退出示教模式")
    if piper.GetArmStatus().arm_status.ctrl_mode != 1:
        stop()  # 初次退出示教模式需先调用此函数才能切换至CAN模式
    over_time = time.time() + timeout
    while piper.GetArmStatus().arm_status.ctrl_mode != 1:
        if over_time < time.time():
            print("ERROR: CAN模式切换失败，请检查是否退出示教模式")
            exit()
        piper.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        time.sleep(0.01)
    
    enable()
    count = 0
    input("step 2: 回车开始播放点位")
    while play_times == 0 or abs(play_times) != count:
        for n, pos in enumerate(track):
            while True:
                joints = [round(i / 0.0174533 * 1e3) for i in pos[:-1]]
                piper.MotionCtrl_2(0x01, 0x01, move_spd_rate_ctrl, 0x00)
                piper.JointCtrl(*joints)
                time.sleep(0.01)
                current_pos = get_pos()
                print(f"INFO: 第{count + 1}次播放，第{n + 1}个点位，当前位置: {current_pos}，目标位置: {pos}")
                if all(abs(current_pos[i] - pos[i]) < 0.0698 for i in range(6)):
                    break
            if have_gripper and len(pos) == 7:
                piper.GripperCtrl(round(pos[-1] * 1e6), 1000, 0x01, 0x00)
                time.sleep(0.5)
            if play_interval < 0:
                if n != len(track) - 1 and input("INPUT: 输入 q 退出，直接回车播放: ") == 'q':
                    exit()
            else:
                time.sleep(play_interval)
        count += 1