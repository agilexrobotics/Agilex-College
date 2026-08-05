#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperMirrorController(Node):
    def __init__(self):
        super().__init__('gripper_mirror_controller')

        # 订阅 joint7 的状态
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/controller_state',
            self.joint_state_callback,
            10
        )

        # 发布 joint8 控制命令
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/gripper8_controller/joint_trajectory',
            10
        )

        # 定时器，控制每秒发布的频率
        self.timer = self.create_timer(0.02, self.publish_joint8_command)

        self.joint7_position = None  # 用于存储 joint7 的位置

    def joint_state_callback(self, msg):
        try:
            # 找到 joint7 的索引
            joint_index = msg.joint_names.index("joint7")
            self.joint7_position = msg.reference.positions[joint_index]

        except ValueError:
            self.get_logger().warn("joint7 not found in /gripper_controller/state")

    def publish_joint8_command(self):
        if self.joint7_position is not None:
            # 计算反向值
            joint8_position = -self.joint7_position

            # 创建 JointTrajectory 消息
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ["joint8"]

            # 设定轨迹点
            point = JointTrajectoryPoint()
            point.positions = [joint8_position]

            traj_msg.points.append(point)

            # 发布到 gripper8_controller
            self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperMirrorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
