#!/usr/bin/env python3
# _*_ coding:utf-8 _*_

import rospy
import sys
import select
import tty
import termios
from piper_msgs.msg import PosCmd

class PiperArmController:
    def __init__(self):
        rospy.init_node('test_arm')
        self.pub = rospy.Publisher('/pos_cmd', PosCmd, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        
        # Initial position
        self.msg = PosCmd()
        self.msg.x = -0.344
        self.msg.y = 0
        self.msg.z = 0.11
        self.msg.pitch = 0
        self.msg.yaw = 0
        self.msg.roll = 0
        self.msg.gripper = 0
        self.msg.mode1 = 1
        self.msg.mode2 = 0
        
        # Save original terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
    def __del__(self):
        # Restore terminal settings when done
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        # Non-blocking key check
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def print_instructions(self):
        print("\nControl commands:")
        print("q: x+0.1, a: x-0.1")
        print("w: y+0.1, s: y-0.1")
        print("e: z+0.1, d: z-0.1")
        print("Current position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            self.msg.x, self.msg.y, self.msg.z))
    
    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        self.print_instructions()
        
        while not rospy.is_shutdown():
            key = self.get_key()
            if key:
                if key == 'q':
                    self.msg.x += 0.05
                    # self.print_instructions()
                    rospy.loginfo("x:%f",self.msg.x)
                elif key == 'a':
                    self.msg.x -= 0.05
                    # self.print_instructions()
                    rospy.loginfo("x:%f",self.msg.x)
                elif key == 'w':
                    self.msg.y += 0.05
                    # self.print_instructions()
                    rospy.loginfo("x:%f",self.msg.y)
                elif key == 's':
                    self.msg.y -= 0.05
                    # self.print_instructions()
                    rospy.loginfo("x:%f",self.msg.y)
                elif key == 'e':
                    self.msg.z += 0.05
                    # self.print_instructions()
                    rospy.loginfo("x:%f",self.msg.z)
                elif key == 'd':
                    self.msg.z -= 0.05
                    # self.print_instructions()
                    rospy.loginfo("x:%f",self.msg.z)
                elif key == '\x03':  # CTRL+C
                    break
            
            # Publish current position at fixed rate
            self.pub.publish(self.msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PiperArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
