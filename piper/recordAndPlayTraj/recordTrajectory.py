#!/usr/bin/env python3
# -*-coding:utf8-*-
# 录制连续轨迹
import os, time
from piper_sdk import *

if __name__ == "__main__":
    # 是否有夹爪
    have_gripper = True
    # 最大录制时间，单位：秒，0表示无限长（强制关闭程序可结束录制）
    record_time = 10.0
    # 示教模式检测超时时间，单位：秒
    timeout = 10.0
    # 保存轨迹的CSV文件路径
    CSV_path = os.path.join(os.path.dirname(__file__), "trajectory.csv")
    # 初始化并连接机械臂
    piper = Piper("can0")
    interface = piper.init()
    piper.connect()
    time.sleep(0.1)

    def get_pos():
        '''获取机械臂当前关节角度和夹爪角度'''
        joint_state = piper.get_joint_states()[0]
        if have_gripper:
            return joint_state + (piper.get_gripper_states()[0][0], )
        return joint_state
    
    print("step 1: 请点击示教按钮进入示教模式")
    over_time = time.time() + timeout
    while interface.GetArmStatus().arm_status.ctrl_mode != 2:
        if over_time < time.time():
            print("ERROR: 示教模式检测超时，请检查示教模式是否开启")
            exit()
        time.sleep(0.01)

    input("step 2: 回车开始录制轨迹")
    csv = open(CSV_path, "w")
    last_pos = get_pos()
    last_time = time.time()
    over_time = last_time + record_time
    while record_time == 0 or time.time() < over_time:
        current_pos = get_pos()
        if current_pos != last_pos: # 发生变化时记录
            wait_time = round(time.time() - last_time, 4)
            print(f"INFO: 等待时间:{wait_time : 0.4f}s，当前位置: {current_pos}")
            last_pos = current_pos
            last_time = time.time()
            csv.write(f"{wait_time}," + ",".join(map(str, current_pos)) + "\n")
        time.sleep(0.01)
    csv.close()
    print("INFO: 录制结束，再次点击示教按钮退出示教模式")