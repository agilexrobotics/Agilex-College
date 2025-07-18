#!/usr/bin/env python3
# -*-coding:utf8-*-
# 录制点位
import os, time
from piper_sdk import *

if __name__ == "__main__":
    # 是否有夹爪
    have_gripper = True
    # 示教模式检测超时时间，单位：秒
    timeout = 10.0
    # 保存点位的CSV文件路径
    CSV_path = os.path.join(os.path.dirname(__file__), "pos.csv")
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
    
    print("INFO: 请点击示教按钮进入示教模式")
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 2:
        if over_time < time.time():
            print("ERROR: 示教模式检测超时，请检查示教模式是否开启")
            exit()
        time.sleep(0.01)

    count = 1
    csv = open(CSV_path, "w")
    while input("INPUT: 输入 q 退出，直接回车录制: ") != "q":
        current_pos = get_pos()
        print(f"INFO: 第{count}个点位，记录位置: {current_pos}")
        csv.write(",".join(map(str, current_pos)) + "\n")
        count += 1
    csv.close()
    print("INFO: 录制结束，再次点击示教按钮退出示教模式")