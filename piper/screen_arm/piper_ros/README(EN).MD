# AgileX Robotic Arm (humble)

[CN](README.MD)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

python_version >= "3.10"

Test:

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 0 Note the URDF version

DH parameters before S-V1.6-3 firmware version are obtained by taking the contact limit of each joint as the initial coordinate

DH parameters after S-V1.6-3 firmware version offset the coordinate system of j2 and j3 by 2 degrees

The default URDF is now the latter

|firmware version |urdf|
|---|---|
|current version < S-V1.6-3|`piper_description_old.urdf`|
|current version >= S-V1.6-3|`piper_description.urdf`|

## 1 Installation

### 1.1 Install dependencies

```shell
pip3 install python-can
pip3 install scipy
```

```shell
pip3 install piper_sdk
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

## 2 Use CAN Module

Note: The CAN module here only supports the robot arm's built-in CAN module, and does not support other CAN modules.

Install CAN tools

```bash
sudo apt update && sudo apt install can-utils ethtool
```

These two tools are used to configure the CAN module.

If executing the bash script shows `ip: command not found`, install the `ip` command by running `sudo apt-get install iproute2`

### 2.1 Find CAN Modules

Run the following command:

```bash
bash find_all_can_port.sh
```

Enter the password, and if the CAN module is inserted into the computer and detected, it will output something like:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

If there are multiple modules, the output will look like this:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

For each CAN module, there will be a corresponding output like `Interface can1 is connected to USB port 3-1.1:1.0`

Where `can1` is the name of the CAN module detected by the system, and `3-1.1:1.0` is the USB port it is connected to.

If the CAN module has already been activated with a different name, for example `can_piper`, the output will look like:

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

If no CAN module is detected, only the following will be output:

```bash
Both ethtool and can-utils are installed.
```

### 2.2 Activate a Single CAN Module (use can_activate.sh script)

There are two situations for activating a single CAN module: one is when only one CAN module is connected to the PC, and the other is when multiple CAN modules are inserted, but only one is activated.

#### 2.2.1 PC has only one USB-to-CAN module connected

Simply execute:

```bash
bash can_activate.sh can0 1000000
```

Here, `can0` can be replaced with any name, and `1000000` is the baud rate, which cannot be changed.

#### 2.2.2 PC has multiple USB-to-CAN modules connected, but only one module is activated at a time

Note: This case applies when using both the robot arm and the chassis.

(1) Find the USB hardware address of the CAN module. Unplug all CAN modules and plug in only the one connected to the robot arm, then execute:

```shell
bash find_all_can_port.sh
```

Record the USB port value, for example, 3-1.4:1.0.

(2) Activate the CAN device. Assuming the USB port value is 3-1.4:1.0, run:

```bash
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

Explanation: **3-1.4:1.0 is the hardware-encoded USB port, and the CAN device inserted there is renamed as can_piper, with a baud rate of 1000000 and activated.**

(3) Check if activation was successful by running `ifconfig` to see if `can_piper` appears. If it does, the CAN module is configured successfully.

### 2.3 Activate Multiple CAN Modules Simultaneously (use can_muti_activate.sh script)

First, determine how many official CAN modules are plugged into the PC (assumed here as 2).

Note: **If the current computer has 5 CAN modules inserted, you can only activate the specified CAN module**

#### 2.3.1 Record the USB port hardware address of each CAN module

For each CAN module, unplug and reinsert it while recording the corresponding USB port address.

In the `can_muti_activate.sh` file, the `EXPECTED_CAN_COUNT` parameter should be set to the desired number of activated CAN modules (assumed here as 2).

(1) Plug in one CAN module and run:

```shell
bash find_all_can_port.sh
```

Record the `USB port` value, for example, `3-1.4:1.0`

(2) Plug in the next CAN module. Ensure **it is not inserted into the same USB port as the previous one** and run:

```shell
bash find_all_can_port.sh
```

Record the value of the second CAN module's `USB port`, for example `3-1.1:1.0`

Note: **If not previously activated, the first module will default to "can0," and the second will be "can1." If previously activated, the names will be the ones used before.**

#### 2.3.2 Predefine USB ports, target interface names, and their bitrates

Assume the recorded `USB port` values are `3-1.4:1.0` and `3-1.1:1.0`. Replace the values inside the brackets in `USB_PORTS["1-9:1.0"]="can_left:1000000"` with `3-1.4:1.0` and `3-1.1:1.0`.

The final result is:

```shell
USB_PORTS["3-1.4:1.0"]="can_left:1000000"
USB_PORTS["3-1.1:1.0"]="can_right:1000000"
```

Explanation: **3-1.4:1.0 is the hardware-encoded USB port, the CAN device inserted there is renamed to "can_left," with a baud rate of 1000000, and activated.**

#### 2.3.3 Activate multiple CAN modules

Run the script:

```bash
bash can_muti_activate.sh
```

#### 2.3.4 Check if multiple CAN modules were set up successfully

Run `ifconfig` to check if `can_left` and `can_right` are present.

## 3 Start the Node

```shell
cd piper_ros
colcon build
source install/setup.bash
```

### 3.1 Single robotics arm

Node name: `piper_single_ctrl`

param

```shell
can_port: name of the CAN route to open.

auto_enable: Whether to automatically enable the system. If True, the system will automatically enable upon starting the program.
#  Set this to False if you want to manually control the enable state. If the program is interrupted and then restarted, the robotic arm will maintain the state it had during the last run.
# If the arm was enabled, it will remain enabled after restarting.
# If the arm was disabled, it will remain disabled after restarting.

gripper_exist: Whether there is an end gripper. If True, it means there is an end gripper and the gripper control will be turned on.

rviz_ctrl_flag: Whether to use RViz to send joint angle messages. If True, the system will receive joint angle messages sent by rViz.

gripper_val_mutiple: Set the gripper control multiplier.  

# Since the range of joint7 in RViz is [0, 0.04], while the actual gripper travel is 0.08m, the gripper needs to be set to twice the value in RViz to control the real gripper correctly.
```

There are two launch files for starting the single-arm robot: `start_single_piper_rviz.launch.py` and `start_single_piper.launch.py`.

The first one can launch RViz together, allowing you to control the robotic arm by moving the sliders.

Start the control node.

```shell
# start the node
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
# start the launch
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=false gripper_val_mutiple:=2
# Or,the node can be started with default parameters
ros2 launch piper start_single_piper.launch.py
# You can also use RViz to enable control by adjusting the parameters as described above.
ros2 launch piper start_single_piper_rviz.launch.py
```

`ros2 topic list`

```shell
/arm_status #the status of arm
/enable_flag #Enable flag, sent to the node, send true to enable
/end_pose #end pose feedback
/joint_states #Subscribe to the joint message and send the joint position to this message to control the movement of the robot arm
/joint_states_single #joints feedback
/pos_cmd #end pose control
```

ros2 service list

```shell
/enable_srv #Enable service end
```

Enable robotic arm

```shell
# call service
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ true\ 
# pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ true\ 
```

Disable the robotic arm

```shell
# call service
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ false\ 
# pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ 
```

Publish joint messages

Note: The robotic arm will lift. Please ensure there are no obstacles within its working range.

The robotic arm will move at 10% of the default speed, and the gripper torque is set to 0.5N.

The speed range is limited from 1 to 100. Any value less than 1 will be treated as 1.

The position is as follows:

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}"

```

### 3.2 piper Custom Messages

ros package `piper_msgs`

 **Robotic Arm Status Feedback Message**: Corresponds to the feedback message with `id=0x2A1` in the CAN protocol.

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 Standby mode  
0x01 CAN command control mode
0x02 Teaching mode
0x03 Ethernet control mode
0x04 wifi control mode
0x05 Remote controller control mode
0x06 Linked teaching mode
0x07 Offline track mode
*/
uint8 arm_status
/*
0x00 Normal
0x01 Emergency stop
0x02 No solution
0x03 Singularity
0x04 Target angle exceeds limit
0x05 Joint communication error
0x06 Joint brake not released
0x07 Robotic arm collision detected
0x08 Overspeed during drag teaching
0x09 Joint status abnormal
0x0A Other abnormality
0x0B Teaching record
0x0C Teaching execution
0x0D Teaching paused
0x0E Main control NTC overheating
0x0F Discharge resistor NTC overheating
*/
uint8 mode_feedback
/*
0x00 MOVE P
0x01 MOVE J
0x02 MOVE L
0x03 MOVE C
*/
uint8 teach_status
/*
0x00 Close
0x01 Start teaching recording (enter drag teaching mode)
0x02 End teaching recording (exit drag teaching mode)
0x03 Execute teaching trajectory (reproduce drag teaching trajectory)
0x04 Pause
0x05 Resume (continue trajectory reproduction)
0x06 Terminate execution
0x07 Move to trajectory starting point
*/
uint8 motion_status
/*
0x00 Reached the specified point
0x01 Not reached the specified point
*/
uint8 trajectory_num
/* 0~255 (Feedback in offline trajectory mode) */
int64 err_code// Error code
bool joint_1_angle_limit// Joint 1 communication error (0: Normal, 1: Error)
bool joint_2_angle_limit// Joint 2 communication error (0: Normal, 1: Error)
bool joint_3_angle_limit// Joint 3 communication error (0: Normal, 1: Error)
bool joint_4_angle_limit// Joint 4 communication error (0: Normal, 1: Error)
bool joint_5_angle_limit// Joint 5 communication error (0: Normal, 1: Error)
bool joint_6_angle_limit// Joint 6 communication error (0: Normal, 1: Error)
bool communication_status_joint_1// Joint 1 angle exceeds limit (0: Normal, 1: Error)
bool communication_status_joint_2// Joint 2 angle exceeds limit (0: Normal, 1: Error)
bool communication_status_joint_3// Joint 3 angle exceeds limit (0: Normal, 1: Error)
bool communication_status_joint_4// Joint 4 angle exceeds limit (0: Normal, 1: Error)
bool communication_status_joint_5// Joint 5 angle exceeds limit (0: Normal, 1: Error)
bool communication_status_joint_6// Joint 6 angle exceeds limit (0: Normal, 1: Error)
```

End-effector pose control: Note that some singularities may be unreachable.

`PosCmd.msg`

```c
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper
int32 mode1
int32 mode2
```

## Note

- You need to activate the CAN device and set the correct baud rate before you can read messages from or control the robotic arm.

- If the following appears:

  ```shell
  Enable state: False
  <class 'can.exceptions.CanError'> Message NOT sent
  <class 'can.exceptions.CanError'> Message NOT sent
  ```

  This indicates that the robotic arm is not connected to the CAN module. Unplug and re-plug the USB, restart the robotic arm, reactivate the CAN module, and then try restarting the node.

- If automatic enabling is turned on and enabling is not successful within 5 seconds, the program will automatically exit.

## MoveIt Planner and Simulation Environment

| Description | Documentation |
|---|---|
| MoveIt | [MoveIt README](src/piper_moveit/README(EN).md) |
| Simulation | [Simulation README](src/piper_sim/README(EN).md) |

## Q&A

1.  

```shell
colcon build
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.31s]                                                                                                              
--- stderr: piper_description                                                                                            
CMake Error at CMakeLists.txt:5 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake

  Add the installation prefix of "ament_cmake" to CMAKE_PREFIX_PATH or set
  "ament_cmake_DIR" to a directory containing one of the above files.  If
  "ament_cmake" provides a separate development package or SDK, be sure it
  has been installed.


---
Failed   <<< piper_description [0.45s, exited with code 1]
Aborted  <<< piper_control [0.45s]
Aborted  <<< piper_msgs [0.45s]

Summary: 1 package finished [0.55s]
  1 package failed: piper_description
  2 packages aborted: piper_control piper_msgs
  3 packages had stderr output: piper_control piper_description piper_msgs
```

```shell
source /opt/ros/foxy/setup.bash
```

2. 

```shell
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.33s]                                                                                                              
Finished <<< piper_description [0.42s]                                                                                     
--- stderr: piper_control                                                               
CMake Error at CMakeLists.txt:7 (find_package):
  By not providing "Findcontroller_manager.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "controller_manager", but CMake did not find one.

  Could not find a package configuration file provided by
  "controller_manager" with any of the following names:

    controller_managerConfig.cmake
    controller_manager-config.cmake

  Add the installation prefix of "controller_manager" to CMAKE_PREFIX_PATH or
  set "controller_manager_DIR" to a directory containing one of the above
  files.  If "controller_manager" provides a separate development package or
  SDK, be sure it has been installed.


---
Failed   <<< piper_control [0.80s, exited with code 1]
Aborted  <<< piper_msgs [3.28s]                               

Summary: 2 packages finished [3.38s]
  1 package failed: piper_control
  1 package aborted: piper_msgs
  1 package had stderr output: piper_control
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

3. 

```shell
 "package 'joint_state_publisher_gui' not found
```

```shell
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
```

4. 

```shell
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
```

5. 

```shell
sudo apt install ros-foxy-xacro
```
