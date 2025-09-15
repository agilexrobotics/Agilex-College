#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import math
import rospy
from geometry_msgs.msg import Twist

class VelocityPIDController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # PID parameters for velocity control (tuned values)
        self.kp = 40.0  # Proportional gain
        self.ki = 0.01   # Integral gain
        self.kd = 1    # Derivative gain
        
        # Initialize error terms for each wheel
        self.prev_error = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        self.integral = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        
        # Wheel joint and actuator names
        self.wheel_joints = {
            'fl': 'fl_wheel',
            'fr': 'fr_wheel',
            'rl': 'rl_wheel',
            'rr': 'rr_wheel'
        }
        
        self.wheel_actuators = {
            'fl': 'fl_wheel_motor',
            'fr': 'fr_wheel_motor',
            'rl': 'rl_wheel_motor',
            'rr': 'rr_wheel_motor'
        }
        
        # Control limits
        self.max_control = 10.0
        self.min_control = -10.0

    def compute_vel_pid(self, wheel, setpoint):
        """Compute velocity PID control for a single wheel."""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.wheel_joints[wheel])
        current_vel = self.data.joint(joint_id).qvel[0]  # Using velocity instead of position
        
        error = setpoint - current_vel
        self.integral[wheel] += error
        derivative = error - self.prev_error[wheel]
        
        # Compute PID output
        output = (self.kp * error + 
                 self.ki * self.integral[wheel] + 
                 self.kd * derivative)
        
        # Apply output limits
        output = np.clip(output, self.min_control, self.max_control)
        self.prev_error[wheel] = error
        
        return output
    
    def set_wheel_vel(self, velocities):
        """Set target velocities for all wheels."""
        for wheel in ['fl', 'fr', 'rl', 'rr']:
            if wheel in velocities:
                control = self.compute_vel_pid(wheel, velocities[wheel])
                actuator_id = mujoco.mj_name2id(
                    self.model, 
                    mujoco.mjtObj.mjOBJ_ACTUATOR, 
                    self.wheel_actuators[wheel]
                )
                self.data.ctrl[actuator_id] = control

class AnglePIDController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # PID parameters for steering (tuned values)
        self.kp = 10.0  # Proportional gain
        self.ki = 0.01   # Integral gain
        self.kd = 1.0   # Derivative gain
        
        # Initialize error terms
        self.prev_error = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        self.integral = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        
        # Steering joint and actuator names
        self.steering_joints = {
            'fl': 'fl_steering_joint',
            'fr': 'fr_steering_joint',
            'rl': 'rl_steering_joint',
            'rr': 'rr_steering_joint'
        }
        
        self.steering_actuators = {
            'fl': 'fl_steering',
            'fr': 'fr_steering',
            'rl': 'rl_steering',
            'rr': 'rr_steering'
        }
        
        # Control limits (in radians)
        self.max_control = math.pi/2   # 90 degrees
        self.min_control = -math.pi/2

    def compute_pos_pid(self, wheel, setpoint):
        """Compute position PID control for steering."""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.steering_joints[wheel])
        current_pos = self.data.joint(joint_id).qpos[0]
        
        error = setpoint - current_pos
        self.integral[wheel] += error
        derivative = error - self.prev_error[wheel]
        
        # Compute PID output
        output = (self.kp * error + 
                 self.ki * self.integral[wheel] + 
                 self.kd * derivative)
        
        # Apply output limits
        output = np.clip(output, self.min_control, self.max_control)
        self.prev_error[wheel] = error
        
        return output
    
    def set_steering_angle(self, angles):
        """Set steering angles for all wheels."""
        for wheel in ['fl', 'fr', 'rl', 'rr']:
            if wheel in angles:
                control = self.compute_pos_pid(wheel, angles[wheel])
                actuator_id = mujoco.mj_name2id(
                    self.model, 
                    mujoco.mjtObj.mjOBJ_ACTUATOR, 
                    self.steering_actuators[wheel]
                )
                self.data.ctrl[actuator_id] = control

class FourWheelSteeringController:
    def __init__(self):
        # Robot dimensions (adjust according to your robot)
        self.track = 0.4  # Wheel track (distance between left and right wheels)
        self.wheel_steering_y_offset = 0.05  # Steering joint offset from wheel center
        self.wheel_radius = 0.1  # Wheel radius (m)
        self.wheel_base = 0.5  # Wheelbase (distance between front and rear wheels)
        
        # Current command
        self.lin_x = 0.0
        self.lin_y = 0.0
        self.ang = 0.0
        
        # ROS subscriber
        rospy.init_node('four_wheel_steering_controller')
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
    
    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel topic."""
        self.lin_x = msg.linear.x
        self.lin_y = msg.linear.y
        self.ang = msg.angular.z
    
    def calculate_wheel_commands(self):
        """Calculate wheel velocities and steering angles based on cmd_vel."""
        
        # Initialize outputs
        wheel_velocities = {}
        steering_angles = {}

        # Pure rotation
        if (self.lin_x == 0 and self.lin_y == 0 and self.ang != 0):
            # Compute the steering angle (same for all wheels, but left/right have opposite signs)
            steering_angle = math.atan2(self.wheel_base, self.track)
            rotate_direction = self.ang/abs(self.ang)
            
            steering_angles = {
                'fl': steering_angle,         # Front left: turn left
                'fr': -steering_angle,        # Front right: turn right
                'rl': -steering_angle,         # Rear left: turn left
                'rr': steering_angle         # Rear right: turn right
            }
            
            # Compute the wheel speed (same magnitude, direction depends on ang)
            wheel_speed = math.sqrt((self.track * 0.5)**2 + (self.wheel_base * 0.5)**2) * abs(self.ang)
            
            wheel_velocities = {
                'fl': wheel_speed*rotate_direction,            # Front left: forward
                'fr': -wheel_speed*rotate_direction,            # Front right: forward
                'rl': wheel_speed*rotate_direction,            # Rear left: forward
                'rr': -wheel_speed*rotate_direction             # Rear right: forward
            }

        # Translation (modified to handle reverse motion)
        if ((self.lin_x != 0 or self.lin_y != 0) and self.ang == 0):
            # Compute steering angle (same for forward/reverse)
            angle = math.atan2(self.lin_y, abs(self.lin_x))  # Use abs(lin_x) to avoid ±π
            
            # Compute speed magnitude
            speed = math.sqrt(math.pow(self.lin_y, 2) + math.pow(self.lin_x, 2))
            
            # Reverse speed if lin_x < 0
            if self.lin_x < 0:
                speed = -speed

            steering_angles = {
                'fl': angle,
                'fr': angle,
                'rl': angle,
                'rr': angle
            }
            wheel_velocities = {
                'fl': speed,
                'fr': speed,
                'rl': speed,
                'rr': speed
            }
        return wheel_velocities, steering_angles

def simulate():
    # Load model and data
    model = mujoco.MjModel.from_xml_path("/home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml")
    data = mujoco.MjData(model)
    
    # Initialize controllers
    pos_pid = AnglePIDController(model, data)
    vel_pid = VelocityPIDController(model, data)
    steering_controller = FourWheelSteeringController()
    
    # Initialize viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    
    # Camera settings
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 5
    viewer.cam.lookat = np.array([0.0, 0.0, 0.0])
    
    # Simulation loop
    try:
        while viewer.is_running() and not rospy.is_shutdown():
            sim_start = time.time()
            
            # Calculate wheel velocities and angles using ROS cmd_vel
            wheel_velocities, steering_angles = steering_controller.calculate_wheel_commands()
            
            # Update controllers
            pos_pid.set_steering_angle(steering_angles)
            vel_pid.set_wheel_vel(wheel_velocities)
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer
            viewer.sync()
            
            # Real-time simulation
            time_until_next_step = model.opt.timestep - (time.time() - sim_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()

if __name__ == "__main__":
    simulate()