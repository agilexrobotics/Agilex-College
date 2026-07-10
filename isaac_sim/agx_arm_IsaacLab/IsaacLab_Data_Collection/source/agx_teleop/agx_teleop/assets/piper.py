import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from agx_teleop.assets import AGX_TELEOP_DESCRIPTION_DIR

PIPER_GRIPPER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{AGX_TELEOP_DESCRIPTION_DIR}/agx_arm_urdf/piper/usd/piper.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,  
            "joint2": 1.57,
            "joint3": -1.57,
            "joint4": 0.0,  
            "joint5": 1.22,
            "joint6": 0.0,
            "gripper_joint1": 0.0,
            "gripper_joint2": 0.0,
        },
        pos=(0.0, 0.0, 0.0)
    ),
    actuators={
        "piper_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-3]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=5.0,
            stiffness=80.0,
            damping=20.0,
        ),
        "piper_forearm": ImplicitActuatorCfg(
            joint_names_expr=["joint[4-6]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=5.0,
            stiffness=40.0,
            damping=10.0,
        ),
        "gripper": ImplicitActuatorCfg(     # 参考 https://github.com/smalleha/isaac_so_arm101/blob/main/src/isaac_so_arm101/robots/piper_description/piper.py
            joint_names_expr=["gripper_joint1","gripper_joint2"],
            effort_limit_sim=22,  # Increased from 1.9 to 2.5 for stronger grip
            velocity_limit_sim=1.5,
            stiffness=800.0,  # Increased from 25.0 to 60.0 for more reliable closing
            damping=20.0,  # Increased from 10.0 to 20.0 for stability
        ),
    },
    soft_joint_pos_limit_factor=0.9,
)
"""Configuration for the Piper robot with parallel gripper."""

PIPER_GRIPPER_HIGH_PD_CFG = PIPER_GRIPPER_CFG.copy()
PIPER_GRIPPER_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
PIPER_GRIPPER_HIGH_PD_CFG.actuators["piper_shoulder"].stiffness = 400.0
PIPER_GRIPPER_HIGH_PD_CFG.actuators["piper_shoulder"].damping = 80.0
PIPER_GRIPPER_HIGH_PD_CFG.actuators["piper_forearm"].stiffness = 200.0
PIPER_GRIPPER_HIGH_PD_CFG.actuators["piper_forearm"].damping = 40.0
PIPER_GRIPPER_HIGH_PD_CFG.actuators["gripper"].velocity_limit_sim = 0.5
"""Configuration of Piper with gripper robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
