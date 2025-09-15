import mujoco
import numpy as np
import time
import mujoco.viewer
import cv2

class SteeringPIDController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # PID parameters for steering (tune these as needed)
        self.kp = 10.0  # Proportional gain
        self.ki = 0.01  # Integral gain
        self.kd = 1  # Derivative gain
        
        # Initialize error terms for each steering joint
        self.prev_error = {
            'fl': 0.0,
            'fr': 0.0,
            'rl': 0.0,
            'rr': 0.0
        }
        
        self.integral = {
            'fl': 0.0,
            'fr': 0.0,
            'rl': 0.0,
            'rr': 0.0
        }
        
        # Steering joint names in the model
        self.steering_joints = {
            'fl': 'fl_steering_joint',
            'fr': 'fr_steering_joint',
            'rl': 'rl_steering_joint',
            'rr': 'rr_steering_joint'
        }
        
        # Actuator names for steering
        self.steering_actuators = {
            'fl': 'fl_steering',
            'fr': 'fr_steering',
            'rl': 'rl_steering',
            'rr': 'rr_steering'
        }
        
        # Set control limits
        self.max_control = 2.1  # Maximum control output
        self.min_control = -2.1  # Minimum control output
        
    def compute_pid(self, wheel, setpoint):
        """Compute PID control for a single steering joint."""
        # Get current position
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.steering_joints[wheel])
        current_pos = self.data.joint(joint_id).qpos[0]
        
        # Calculate error
        error = setpoint - current_pos
        
        # Update integral term with anti-windup
        self.integral[wheel] += error
        integral_term = self.ki * self.integral[wheel]
        
        # Calculate derivative term
        derivative = error - self.prev_error[wheel]
        derivative_term = self.kd * derivative
        self.prev_error[wheel] = error
        
        # Compute PID output
        output = self.kp * error + integral_term + derivative_term
        
        # Apply output limits
        output = np.clip(output, self.min_control, self.max_control)
        
        return output
    
    def set_steering_angle(self, angles):
        """
        Set steering angles for all wheels.
        angles: dictionary with keys 'fl', 'fr', 'rl', 'rr' and desired angles in radians
        """
        for wheel in ['fl', 'fr', 'rl', 'rr']:
            if wheel in angles:
                # Compute PID control
                control = self.compute_pid(wheel, angles[wheel])
                
                # Apply control to actuator
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.steering_actuators[wheel])
                self.data.ctrl[actuator_id] = control

def create_sliders():
    """Create OpenCV window with sliders for each wheel's steering angle."""
    cv2.namedWindow("Steering Control", cv2.WINDOW_NORMAL)
    
    # Create sliders for each wheel
    cv2.createTrackbar("FL Angle", "Steering Control", 0, 210, lambda x: None)
    cv2.createTrackbar("FR Angle", "Steering Control", 0, 210, lambda x: None)
    cv2.createTrackbar("RL Angle", "Steering Control", 0, 210, lambda x: None)
    cv2.createTrackbar("RR Angle", "Steering Control", 0, 210, lambda x: None)
    
    # Set initial positions to middle (0 degrees)
    cv2.setTrackbarPos("FL Angle", "Steering Control", 105)  # 105 = middle of 0-210
    cv2.setTrackbarPos("FR Angle", "Steering Control", 105)
    cv2.setTrackbarPos("RL Angle", "Steering Control", 105)
    cv2.setTrackbarPos("RR Angle", "Steering Control", 105)

def get_slider_values():
    """Get current slider values and convert to radians."""
    # Read slider positions (0-210)
    fl_pos = cv2.getTrackbarPos("FL Angle", "Steering Control")
    fr_pos = cv2.getTrackbarPos("FR Angle", "Steering Control")
    rl_pos = cv2.getTrackbarPos("RL Angle", "Steering Control")
    rr_pos = cv2.getTrackbarPos("RR Angle", "Steering Control")
    
    # Convert from 0-210 range to -1.05 to 1.05 radians (-60 to 60 degrees)
    fl_angle = (fl_pos - 105) * 0.01
    fr_angle = (fr_pos - 105) * 0.01
    rl_angle = (rl_pos - 105) * 0.01
    rr_angle = (rr_pos - 105) * 0.01
    
    return {
        'fl': fl_angle,
        'fr': fr_angle,
        'rl': rl_angle,
        'rr': rr_angle
    }

def simulate():
    # Load model and data
    model = mujoco.MjModel.from_xml_path("/home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml")
    data = mujoco.MjData(model)
    
    # Initialize PID controller
    pid = SteeringPIDController(model, data)
    
    # Initialize the MuJoCo viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    
    # Create OpenCV sliders
    create_sliders()
    
    # Camera settings
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 5
    viewer.cam.lookat = np.array([0.0, 0.0, 0.0])
    
    # Simulation loop
    while viewer.is_running():
        sim_start = time.time()
        
        # Check for OpenCV window events
        cv2.waitKey(1)
        
        # Get desired angles from sliders
        desired_angles = get_slider_values()
        
        # Update PID controller
        pid.set_steering_angle(desired_angles)
        
        # Step the simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer with current state
        viewer.sync()
        
        # Real-time simulation
        time_until_next_step = model.opt.timestep - (time.time() - sim_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
    
    viewer.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    simulate()