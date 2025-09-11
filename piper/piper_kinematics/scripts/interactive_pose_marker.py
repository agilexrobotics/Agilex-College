#!/usr/bin/env python3
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf
from geometry_msgs.msg import TransformStamped

class InteractivePoseMarker:
    def __init__(self):
        rospy.init_node('interactive_pose_marker')
        
        # 参数
        self.marker_name = rospy.get_param('~marker_name', 'target_pose')
        self.reference_frame = rospy.get_param('~reference_frame', 'base_link')
        self.initial_position = rospy.get_param('~initial_position', [0.5, 0.0, 0.5])
        self.marker_scale = rospy.get_param('~marker_scale', 0.2)
        self.pose_topic = rospy.get_param('~pose_topic', '/target_pose')
        
        # 创建交互标记服务器
        self.server = InteractiveMarkerServer("interactive_marker")
        
        # 创建位姿发布者
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        
        # 创建TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 初始化标记
        self.create_marker()
        
        rospy.loginfo("Interactive pose marker '%s' is ready in frame '%s'", 
                      self.marker_name, self.reference_frame)
    
    def create_marker(self):
        """创建一个6自由度交互式标记"""
        # 创建交互标记
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.reference_frame
        int_marker.header.stamp = rospy.Time.now()
        int_marker.name = self.marker_name
        int_marker.description = f"{self.marker_name} (6DOF Control)"
        int_marker.pose.position.x = self.initial_position[0]
        int_marker.pose.position.y = self.initial_position[1]
        int_marker.pose.position.z = self.initial_position[2]
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = self.marker_scale

        # 添加6自由度控制
        self.add_6dof_control(int_marker)
        
        # 添加到服务器
        self.server.insert(int_marker, self.marker_feedback_cb)
        self.server.applyChanges()
    
    def add_6dof_control(self, int_marker):
        """为标记添加6自由度控制"""
        # 旋转控制
        for axis in ['x', 'y', 'z']:
            control = self.make_arrow_control(axis, mode=InteractiveMarkerControl.ROTATE_AXIS)
            int_marker.controls.append(control)
        
        # 平移控制
        for axis in ['x', 'y', 'z']:
            control = self.make_arrow_control(axis, mode=InteractiveMarkerControl.MOVE_AXIS)
            int_marker.controls.append(control)
    
    def make_arrow_control(self, axis, mode):
        """创建箭头控制"""
        control = InteractiveMarkerControl()
        
        if axis == 'x':
            control.orientation.w = 0.7071
            control.orientation.x = 0.7071
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.name = f"rotate_{axis}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{axis}"
        elif axis == 'y':
            control.orientation.w = 0.7071
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 0.7071
            control.name = f"rotate_{axis}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{axis}"
        elif axis == 'z':
            control.orientation.w = 0.7071
            control.orientation.x = 0.0
            control.orientation.y = 0.7071
            control.orientation.z = 0.0
            control.name = f"rotate_{axis}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{axis}"
        
        control.interaction_mode = mode
        control.always_visible = True
        return control
    
    def marker_feedback_cb(self, feedback):
        """标记移动时的回调函数"""
        # 发布位姿
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose
        self.pose_pub.publish(pose_msg)
        
        # 发布TF - ROS1方式
        self.tf_broadcaster.sendTransform(
            (feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z),
            (feedback.pose.orientation.x, feedback.pose.orientation.y, 
             feedback.pose.orientation.z, feedback.pose.orientation.w),
            rospy.Time.now(),
            feedback.marker_name,
            feedback.header.frame_id
        )

if __name__ == '__main__':
    try:
        node = InteractivePoseMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass