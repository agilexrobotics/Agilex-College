#!/bin/python3
# coding:utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sys
import select
import tty
import termios
from piper_msgs.msg import PosCmd
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs  # 必须导入，即使看起来没用
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class PointCloudSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pointcloud_subscriber', anonymous=True)

        # 初始化tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅PointCloud2话题
        self.sub = rospy.Subscriber("/line_path", PointCloud2, self.callback)
        
        # 发布PosCmd话题
        self.pos_cmd_pub = rospy.Publisher('/pos_cmd', PosCmd, queue_size=10)
        # 发布转换后的点云
        self.cloud_pub = rospy.Publisher('/transformed_cloud', PointCloud2, queue_size=10)
        
        self.rate = rospy.Rate(30)  # 10Hz
        
        # 存储接收到的点云
        self.original_points = []  # 存储原始点云
        self.transformed_points = []  # 存储转换后的点云
        self.current_point_idx = 0  # 当前发布的点索引
        self.continuous_publishing = False  # 连续发布标志
        
        # 初始化PosCmd消息
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
        
        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # rospy.loginfo("PointCloud2 subscriber node started. Waiting for messages...")

    def transform_point_cloud(self, cloud_msg):
        try:
            # 确保点云有正确的frame_id
            if cloud_msg.header.frame_id == '':
                rospy.logwarn("接收到的点云没有frame_id!")
                return None
            # 获取从frame_1到frame_2的变换
            transform = self.tf_buffer.lookup_transform(
                'arm_base',                     # 目标坐标系
                cloud_msg.header.frame_id,     # 源坐标系(通常是frame_1)
                rospy.Time(0),                # 获取最新可用变换
                rospy.Duration(1.0)           # 等待1秒
            )
            # 转换整个点云
            transformed_cloud = do_transform_cloud(cloud_msg, transform)
            
            # 发布转换后的点云
            self.cloud_pub.publish(transformed_cloud)
            
            # 返回转换后的点云数据
            return np.array(list(pc2.read_points(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True)))
            
        except (tf2_ros.LookupException, 
            tf2_ros.ConnectivityException, 
            tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("坐标变换失败: %s", str(e))
            return None
        except Exception as e:
            rospy.logerr("处理点云时发生错误: %s", str(e))
            return None

    def __del__(self):
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        # 非阻塞键盘输入检查
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def print_instructions(self):
        print("\nControl commands:")
        print("r: 记录当前点云并转换到frame_2坐标系")
        print("s: 发送转换后的点云位置到pos_cmd")
        print("p: 开始/停止连续发送所有转换后的点(10Hz)")
        print("Current position: x={:.3f}, y={:.3f}, z={:.3f}".format(
            self.msg.x, self.msg.y, self.msg.z))
        print("Stored points: {}".format(len(self.transformed_points)))
    
    def callback(self, msg):
        try:
            # 将点云转换为numpy数组并存储
            self.original_points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            # rospy.loginfo("Received new PointCloud2 with {} points".format(len(self.original_points)))
            
        except Exception as e:
            rospy.logerr("Error processing PointCloud2 message: %s", str(e))
    
    def publish_next_point(self):
        if len(self.transformed_points) == 0:
            print("No transformed points available! Press 'r' to record and transform first.")
            return False
        
        # 获取当前点
        point = self.transformed_points[self.current_point_idx]
        
        # 更新PosCmd消息
        self.msg.x = -float(point[0])+0.06
        self.msg.y = -float(point[1])
        self.msg.z = (-float(point[2]))+0.08
        # self.msg.z = 0.11

        # print("x:",self.msg.x)
        # print("y:",self.msg.y)
        # print("z:",self.msg.z)

        # # 发布当前点
        self.pos_cmd_pub.publish(self.msg)
        
        # 更新索引（循环）
        self.current_point_idx = (self.current_point_idx + 1) % len(self.transformed_points)
        
        print("Published point {}/{}: x={:.3f}, y={:.3f}, z={:.3f}".format(
            self.current_point_idx, len(self.transformed_points), 
            self.msg.x, self.msg.y, self.msg.z))
        
        return True

    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        self.print_instructions()
        
        while not rospy.is_shutdown():
            key = self.get_key()
            if key:
                if key == 'r':
                    # 记录并转换点云
                    if len(self.original_points) > 0:
                        # 创建一个临时的PointCloud2消息用于转换
                        cloud_msg = PointCloud2()
                        cloud_msg.header.stamp = rospy.Time.now()
                        cloud_msg.header.frame_id = "camera_color_optical_frame"  # 假设原始点云在frame_1坐标系
                        cloud_msg.height = 1
                        cloud_msg.width = len(self.original_points)
                        cloud_msg.fields = [
                            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1)
                        ]
                        cloud_msg.is_bigendian = False
                        cloud_msg.point_step = 12
                        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
                        cloud_msg.is_dense = True
                        cloud_msg.data = self.original_points.astype(np.float32).tobytes()
                        
                        # 转换点云
                        self.transformed_points = self.transform_point_cloud(cloud_msg)
                        if self.transformed_points is not None:
                            print(f"Recorded and transformed {len(self.transformed_points)} points to frame_2")
                            self.current_point_idx = 0  # 重置索引
                    else:
                        print("No points available to transform!")
                
                elif key == 's':
                    # 发送转换后的点
                    if len(self.transformed_points) > 0:
                        self.publish_next_point()
                    else:
                        print("No transformed points available! Press 'r' first.")
                
                elif key == 'p':
                    # 切换连续发布模式
                    self.continuous_publishing = not self.continuous_publishing
                    if self.continuous_publishing:
                        print("Started continuous publishing at 10Hz")
                    else:
                        print("Stopped continuous publishing")
            
            # 如果处于连续发布模式，则发布下一个点
            if self.continuous_publishing and len(self.transformed_points) > 0:
                self.publish_next_point()
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PointCloudSubscriber()
        controller.run()
    except rospy.ROSInterruptException:
        pass