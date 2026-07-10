# Teleoperation Data Collection with Piper and Nero in IsaacLab

This project implements a **cube-stacking task for the Piper and Nero robotic arms** in the [IsaacLab](https://github.com/isaac-sim/IsaacLab) environment. It supports the collection and replay of human demonstration data through teleoperation, and also provides an automated data collection script for efficiently gathering demonstration data. This project is built as an external project for IsaacLab.

![Teleop Demonstration](./assets/nero_ik.gif)


## Installation

### 1. Prerequisites
- Isaac Sim and IsaacLab have been installed and properly configured.
- The Conda virtual environment named `isaaclab` corresponding to the IsaacLab installation is available.

### 2. Install the Project
```bash
git clone https://github.com/szyzp/IsaacLab_Data_Collection.git
cd IsaacLab_Data_Collection
conda activate isaaclab
python -m pip install -e source/agx_teleop
```

## Usage Guide

Before collecting data, make sure the `datasets` folder has been created:

```bash
mkdir -p datasets
```

### Manually Collect Demonstration Data

Use the keyboard as the input device to manually record 10 successful demonstrations.

Available task environments:
- Piper robotic arm task environment: `Isaac-Stack-Cube-Piper-IK-Rel-v0`
- Nero robotic arm task environment: `Isaac-Stack-Cube-Nero-IK-Rel-v0`

```bash
python scripts/tools/record_demos.py \
    --task Isaac-Stack-Cube-Piper-IK-Rel-v0 \
    --device cpu \
    --teleop_device keyboard \
    --dataset_file ./datasets/dataset.hdf5 \
    --num_demos 10
```

*(Optional teleoperation devices for `--teleop_device`: `keyboard`, `spacemouse`, `handtracking`)*

### Automatically Collect Demonstration Data

```bash
python scripts/tools/record_ik_stack.py \
  --task Isaac-Stack-Cube-Piper-IK-Rel-v0 \
  --device cuda \
  --dataset_file ./datasets/ik_dataset.hdf5 \
  --num_demos 10
```

*(Optionally add the `--headless` argument to automatically collect demonstration data on a server without visualization.)*

### Replay Demonstration Data

Replay and verify the collected `.hdf5` dataset.

```bash
python scripts/tools/replay_demos.py \
    --task Isaac-Stack-Cube-Piper-IK-Rel-v0 \
    --device cpu \
    --dataset_file ./datasets/dataset.hdf5
```

## Keyboard Teleoperation Controls

If `--teleop_device keyboard` is used when starting the recording script, use the following keys to control the robotic arm while the running environment is active. The control is based on SE(3) control.

| Action | Shortcut |
| :--- | :---: |
| **Reset all commands** | `R` |
| **Open/close gripper** | `K` |
| **Move along the X-axis** | `W` / `S` |
| **Move along the Y-axis** | `A` / `D` |
| **Move along the Z-axis** | `Q` / `E` |
| **Rotate around the X-axis** | `Z` / `X` |
| **Rotate around the Y-axis** | `T` / `G` |
| **Rotate around the Z-axis** | `C` / `V` |

> **Shortcuts during replay**:
> - Pause replay: `B`
> - Resume replay: `N`


## References

1. [IsaacLab Official Documentation](https://isaac-sim.github.io/IsaacLab/main/index.html)  
2. [IsaacLab](https://github.com/isaac-sim/IsaacLab)