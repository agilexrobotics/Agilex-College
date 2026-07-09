# Nero使用案例--基于Isaac Lab 双臂Nero强化学习reach point

在机器人研究领域，机械臂的智能控制一直是具身智能研究的核心方向之一。传统的运动学控制方案虽然稳定，但面对复杂的非结构化环境往往束手无策，而强化学习技术的出现，为机械臂实现自主适应环境、完成复杂任务提供了全新的可能。

本项目在原有 [SO-ARM101](https://github.com/MuammerBay/isaac_so_arm101) 项目的基础上，适配了双臂nero机械臂，让开发者可以快速基于 NVIDIA Isaac Lab 平台，开展机械臂的强化学习训练与验证。

## 项目仓库

开源代码仓库

https://github.com/smalleha/isaac_so_arm101.git

## 一、前情提要

环境配置及其他IsaacLab案例



## 二、项目文件结构

相较于之前的项目，这次新增双臂的nero的URDF模型，rl_games强化学习算法以及相对于的一些配置文件

```
├── robots
│   ├── dual_nero								#新增双臂nero的URDF模型
│   │   ├── dual_nero.py
│   │   ├── __init__.py
│   │   ├── meshes
│   │   └── urdf
│   │       └── dual_nero.urdf
├── scripts
│   ├── rl_games								#新增rl_games强化学习算法框架
│   │   ├── play.py
│   │   └── train.py
│   └── zero_agent.py
├── tasks
│   ├── __init__.py
│   └── reach								
│       ├── agents
│       ├── dual_nero_joint_pos_env_cfg.py		新增nero双臂的配置文件
│       ├── dual_nero_reach_env_cfg.py
│       ├── __init__.py
│       ├── mdp
└── ui_extension_example.py
```

## 三、配置IsaacLab文件

### 修改URDF

和之前的步骤一样，将dual_nero.urdf修改一下索引路径，将所有link的mesh 文件的路径修改为相对路径；拿其中的base_link作为例子

```xml
  <link name="base_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0592395620981769 -0.068642440505388 0.0562764736144042"/>
      <mass value="4.46524857458863"/>
      <inertia ixx="0.0608289280191989" ixy="-2.54649959130438E-06" ixz="1.11851948046933E-07" iyy="0.0218722514454004" iyz="-0.000689477252402357" izz="0.0680524540174318"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="../meshes/base_link.dae"/>
      </geometry>
      <material name="">
        <color rgba="0.776470588235294 0.756862745098039 0.737254901960784 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="../meshes/base_link.dae"/>
      </geometry>
    </collision>
  </link>
```

### 导入URDF

修改完urdf之后，需要编写一个python文件导入URDF模型，设置机械臂的电机属性，刚度，阻尼等参数；文件名称为isaac_so_arm101/robots/dual_nero/dual_nero.py

```python
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

TEMPLATE_ASSETS_DATA_DIR = Path(__file__).resolve().parent

DUAL_NERO_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=True,
        merge_fixed_joints=False,
        replace_cylinders_with_capsules=True,
        asset_path=f"{TEMPLATE_ASSETS_DATA_DIR}/urdf/dual_nero.urdf",
        activate_contact_sensors=False, # set as false while waiting for capsule implementation
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
                "left_joint.*": 0.0,
                "right_joint.*": 0.0,
                "left_gripper_joint.*": 0.0,  
                "right_gripper_joint.*": 0.0,  
        },
        # Set initial joint velocities to zero
        joint_vel={".*": 0.0},
    ),
    actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["left_joint.*", "right_joint.*"],
                effort_limit=25.0, 
                velocity_limit=1.5,
               
                stiffness={
                    "left_joint1": 200.0, 
                    "left_joint2": 170.0,
                    "left_joint3": 120.0,
                    "left_joint4": 80.0,
                    "left_joint5": 50.0,
                    "left_joint6": 20.0,
                    "left_joint7": 10.0,
                    "right_joint1": 200.0, 
                    "right_joint2": 170.0,
                    "right_joint3": 120.0,
                    "right_joint4": 80.0,
                    "right_joint5": 50.0,
                    "right_joint6": 20.0,
                    "right_joint7": 10.0
                },
               
                damping={
                    "left_joint1": 100.0,
                    "left_joint2": 60.0,
                    "left_joint3": 70.0,
                    "left_joint4": 24.0,
                    "left_joint5": 20.0,
                    "left_joint6": 10.0,
                    "left_joint7": 5,
                    "right_joint1": 100.0,
                    "right_joint2": 60.0,
                    "right_joint3": 70.0,
                    "right_joint4": 24.0,
                    "right_joint5": 20.0,
                    "right_joint6": 10.0,
                    "right_joint7": 5,
                },
            ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["left_gripper_joint.*","right_gripper_joint.*"],
            effort_limit_sim=22,  # Increased from 1.9 to 2.5 for stronger grip
            velocity_limit_sim=1.5,
            stiffness=800.0,  # Increased from 25.0 to 60.0 for more reliable closing
            damping=20.0,  # Increased from 10.0 to 20.0 for stability
        ),

        },
    soft_joint_pos_limit_factor=0.9,
)
```

然后还需要创建一个__ int__.py文件，初始化文件，标记该目录为Python子模块

### 创建tasks任务

在tasks/reach目录下创建两个文件dual_nero_joint_pos_env_cfg.py和dual_nero_reach_env_cfg.py

dual_nero_joint_pos_env_cfg.py包含双臂关节位置控制的环境配置，其中需要明确可控制关节和机械臂末端link

```python
import math

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab.utils import configclass
from isaac_so_arm101.robots import DUAL_NERO_CFG # noqa: F401   
from isaac_so_arm101.tasks.reach.dual_nero_reach_env_cfg import Dual_NeroReachEnvCfg
from isaaclab.assets.articulation import ArticulationCfg

@configclass
class Dual_Nero_ReachEnvCfg(Dual_NeroReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to OpenArm
        self.scene.robot = DUAL_NERO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot",)

        # override rewards
        self.rewards.left_end_effector_position_tracking.params["asset_cfg"].body_names = ["left_gripper_base"]
        self.rewards.left_end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "left_gripper_base"
        ]
        self.rewards.left_end_effector_orientation_tracking.params["asset_cfg"].body_names = ["left_gripper_base"]

        self.rewards.right_end_effector_position_tracking.params["asset_cfg"].body_names = ["right_gripper_base"]
        self.rewards.right_end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = [
            "right_gripper_base"
        ]
        self.rewards.right_end_effector_orientation_tracking.params["asset_cfg"].body_names = ["right_gripper_base"]

        # override actions
        self.actions.left_arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "left_joint.*",
            ],
            scale=0.5,
            use_default_offset=True,
        )

        self.actions.right_arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "right_joint.*",
            ],
            scale=0.5,
            use_default_offset=True,
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.left_ee_pose.body_name = "left_gripper_base"
        self.commands.right_ee_pose.body_name = "right_gripper_base"

@configclass    
class Dual_Nero_ReachEnvCfg_PLAY(Dual_Nero_ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
```

dual_nero_reach_env_cfg.py包含任务的基础环境配置，任务奖励、惩罚、策略、目标点位置、都是在这里设置

```python
import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
##
# Scene definition
##
@configclass
class Dual_NeroReachSceneCfg(InteractiveSceneCfg):
    """Configuration for the scene with a robotic arm."""

    # world
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0)),
    )

    # robots
    robot: ArticulationCfg = MISSING

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )
##
# MDP settings
##
@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    left_ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.5, 0.5),
            pos_y=(0.15, 0.25),
            pos_z=(0.3, 0.5),
            roll=(-math.pi / 6, math.pi / 6),
            pitch=(3 * math.pi / 2, 3 * math.pi / 2),
            yaw=(8 * math.pi / 9, 10 * math.pi / 9),
        ),
    )

    right_ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.5, 0.5),
            pos_y=(-0.25, -0.15),
            pos_z=(0.3, 0.5),
            roll=(-math.pi / 6, math.pi / 6),
            pitch=(3 * math.pi / 2, 3 * math.pi / 2),
            yaw=(8 * math.pi / 9, 10 * math.pi / 9),
        ),
    )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    left_arm_action: ActionTerm = MISSING
    right_arm_action: ActionTerm = MISSING
@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        left_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=[
                        "left_joint.*",
                    ],
                )
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )

        right_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=[
                        "right_joint.*",
                    ],
                )
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )

        left_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=[
                        "left_joint.*",
                    ],
                )
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        right_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=[
                        "right_joint.*",
                    ],
                )
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        left_pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "left_ee_pose"})
        right_pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "right_ee_pose"})
        left_actions = ObsTerm(func=mdp.last_action, params={"action_name": "left_arm_action"})
        right_actions = ObsTerm(func=mdp.last_action, params={"action_name": "right_arm_action"})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    left_end_effector_position_tracking = RewTerm(
        func=mdp.position_command_error,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "left_ee_pose",
        },
    )
    right_end_effector_position_tracking = RewTerm(
        func=mdp.position_command_error,
        weight=-0.25,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "right_ee_pose",
        },
    )
    left_end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "std": 0.1,
            "command_name": "left_ee_pose",
        },
    )
    right_end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "std": 0.1,
            "command_name": "right_ee_pose",
        },
    )
    left_end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "left_ee_pose",
        },
    )

    right_end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "right_ee_pose",
        },
    )
    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.0001)
    left_joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "left_joint.*",
                ],
            )
        },
    )
    right_joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "right_joint.*",
                ],
            )
        },
    )
@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    action_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "action_rate", "weight": -0.005, "num_steps": 4500},
    )

    left_joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "left_joint_vel", "weight": -0.001, "num_steps": 4500},
    )

    right_joint_vel = CurrTerm(
        func=mdp.modify_reward_weight,
        params={"term_name": "right_joint_vel", "weight": -0.001, "num_steps": 4500},
    )
##
# Environment configuration
##
@configclass
class Dual_NeroReachEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the reach end-effector pose tracking environment."""
    # Scene settings
    scene: Dual_NeroReachSceneCfg = Dual_NeroReachSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.sim.render_interval = self.decimation
        self.episode_length_s = 24.0
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0 / 60.0

```

然后需要在src/isaac_so_arm101/tasks/reach/__ init__.py中注册Dual-Nero-Reach任务

```PYTHON
gym.register(
    id="Isaac-Dual-Nero-Reach-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point":f"{__name__}.dual_nero_joint_pos_env_cfg:Dual_Nero_ReachEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ReachPPORunnerCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",

    },
    disable_env_checker=True,
)
```

## 四、训练reach任务

激活conda 环境

```
conda activate env_isaaclab
```

进入isaac_so_arm101目录

```
cd isaac_so_arm101
```

执行训练指令，采用无头模式，减小资源消耗

**rsl_rl训练框架**

```
uv run train --task Isaac-Dual-Nero-Reach-v0 --headless
```

查看训练效果

```
uv run play --task Isaac-Dual-Nero-Reach-v0
```

![](./img/dual_nero_rsl_rl.gif)

**rl_games训练框架**

```
python3 scripts/rl_games/train.py --task Isaac-Dual-Nero-Reach-v0 --headless
```

查看训练效果

```
python3 scripts/rl_games/play.py --task Isaac-Dual-Nero-Reach-v0
```

![](./img/dual_nero_rl_games.gif)

从训练效果来说，rl_games框架训练出来的效果更好一些，对于比较复杂的机器人模型，建议使用rl_games。
