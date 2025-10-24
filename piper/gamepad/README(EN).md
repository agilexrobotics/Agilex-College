# Gamepad Teleoperation for Robotic Arm—A New Intuitive Control Experience

## Abstract

This project enables intuitive robotic arm control using a standard gamepad. With a conventional game controller, you can operate the PiPER robotic arm in a visual environment, delivering precise and intuitive control.

## Tags
PiPER Robotic Arm, Gamepad Teleoperation, Joint Control, Pose Control, Gripper Control, Forward/Inverse Kinematics

## Repository

- **Navigation**: https://github.com/agilexrobotics/Agilex-College
- **Project**: https://github.com/kehuanjack/Gamepad_PiPER

## Feature Demonstration

[![](https://i.ytimg.com/an_webp/smTTbOfdTlk/mqdefault_6s.webp?du=3000&sqp=CJXS7McG&rs=AOn4CLBfiWjTsfz7kRpplE8f4Wx6WtDRlg)](https://youtu.be/smTTbOfdTlk)

## Environment Setup
- Operating System: Ubuntu 20.04 or higher

- Python Environment: Python 3.9 or higher, recommended to use Anaconda or Miniconda

- Clone the project and navigate to the project root directory:

   ```bash
   git clone https://github.com/kehuanjack/Gamepad_PiPER.git
   cd Gamepad_PiPER
   ```

- Install the general dependency libraries and the dependency libraries for the kinematics module (choose one, cuRobo library is recommended):

   - Based on the [pinocchio](https://github.com/stack-of-tasks/pinocchio) library (Python == 3.9, requires installing [piper_ros](https://github.com/agilexrobotics/piper_ros) and sourcing the robotic arm's ROS workspace; otherwise, mesh files will not be found.):

      ```bash
      conda create -n test_pinocchio python=3.9.* -y
      conda activate test_pinocchio
      pip3 install -r requirements_common.txt --upgrade
      conda install pinocchio=3.6.0 -c conda-forge
      pip install meshcat
      pip install casadi
      ```

      You need to select `from src.gamepad_pin import RoboticArmController` in the `main.py` and `main_virtual.py` files.

   - Based on the [PyRoKi](https://github.com/chungmin99/pyroki) library (Python >= 3.10):

      ```bash
      conda create -n test_pyroki python=3.10.* -y
      conda activate test_pyroki
      pip3 install -r requirements_common.txt --upgrade
      pip3 install pyroki@git+https://github.com/chungmin99/pyroki.git@f234516
      ```

      You need to select `from src.gamepad_limit import RoboticArmController` or `from src.gamepad_no_limit import RoboticArmController` in the `main.py` and `main_virtual.py` files.

   - Based on the [cuRobo](https://github.com/NVlabs/curobo) library (Python >= 3.8, recommended CUDA version is 11.8):

      ```bash
      conda create -n test_curobo python=3.10.* -y
      conda activate test_curobo
      pip3 install -r requirements_common.txt --upgrade
      sudo apt install git-lfs && cd ../
      git clone https://github.com/NVlabs/curobo.git && cd curobo
      pip3 install "numpy<2.0" "torch==2.0.0" pytest lark
      pip3 install -e . --no-build-isolation
      python3 -m pytest .
      cd ../Gamepad_PiPER
      ```

      You need to select `from src.gamepad_curobo import RoboticArmController` in the `main.py` and `main_virtual.py` files.

## Execution Steps

1. **Connect the robotic arm and activate the CAN module**: `sudo ip link set can0 up type can bitrate 1000000`

2. **Connect the gamepad**: Connect the controller to the computer via USB or Bluetooth

3. **Enable control service**: Run `python3 main.py` or `python3 main_virtual.py` in the project directory. It is recommended to run `main_virtual.py` first for virtual arm testing

4. **Gamepad connection verification**: After the program starts, check the console output to confirm the gamepad is correctly recognized

5. **Web visualization**: Open a browser and enter `http://localhost:8080` to access the web interface and visualize the robotic arm status

6. **Start controlling**: Operate the robotic arm according to the gamepad mapping instructions

## Gamepad Control Instructions

### Button Function Mapping

| Button | Short Press Function | Long Press Function |
|--------|----------------------|---------------------|
| **HOME** | Connect/Disconnect robotic arm | None |
| **START** | Switch upper control mode (Joint/Pose) | Switch lower control mode (Joint/Pose) |
| **BACK** | Switch lower command mode (Position Velocity 0x00/Quick Response 0xAD) | None |
| **Y** | Return to zero position | None |
| **A** | Save current position | Clear current saved position |
| **B** | Restore last saved position | None |
| **X** | Toggle position playback order | Clear all saved positions |
| **LB** | Increase speed factor (upper) | Decrease speed factor (upper) |
| **RB** | Increase movement speed (lower) | Decrease movement speed (lower) |

### Joystick and Trigger Functions

| Control Element | Joint Mode Function | Pose Mode Function |
|-----------------|---------------------|--------------------|
| **Left Joystick** | J1 (Base rotation): Left/Right<br/>J2 (Shoulder): Up/Down | End-effector X/Y axis movement |
| **Right Joystick** | J3 (Elbow): Up/Down<br/>J6 (Wrist rotation): Left/Right | End-effector Z axis movement and rotation around Z axis |
| **D-pad** | J4 (Wrist yaw): Left/Right<br/>J5 (Wrist pitch): Up/Down | End-effector rotation around X/Y axes |
| **Left Trigger (LT)** | Close gripper | Close gripper |
| **Right Trigger (RT)** | Open gripper | Open gripper |

### Special Function Notes

1. **Gripper Control**:
   - Gripper opening range: 0-100%
   - Special toggle function: When the gripper is fully closed (0%) or fully open (100%), quickly pressing and releasing the trigger toggles the state

2. **Speed Control**:
   - Speed factors: 0.25x, 0.5x, 1.0x, 2.0x, 3.0x, 4.0x, 5.0x (switched with LB)
   - Movement speed: 10%-100% (switched with RB)

3. **Position Memory**:
   - Multiple position points can be saved
   - Supports sequential and reverse order playback

## Important Notes

- You can run `main_virtual.py` first for virtual robotic arm testing
- For first-time use, start with low-speed mode and increase speed after familiarizing yourself with the operation
- Maintain a safe distance during robotic arm operation. Do not approach the moving robotic arm. Proceed at your own risk.
- Numerical solutions may cause significant joint jumps near critical points. Maintain a safe distance. Proceed at your own risk.
- Quick response mode (0xAD) is dangerous. Use with extreme caution. Maintain a safe distance if used. Proceed at your own risk.
- If using the pinocchio library, you need to source the robotic arm's ROS workspace in advance; otherwise, mesh files will not be found.