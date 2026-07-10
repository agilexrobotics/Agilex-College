import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from agx_teleop.assets import AGX_TELEOP_DESCRIPTION_DIR

##
# Configuration
##

NERO_GRIPPER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{AGX_TELEOP_DESCRIPTION_DIR}/agx_arm_urdf/nero/usd/nero.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 1.22,
            "joint5": 0.0,
            "joint6": 0.0,
            "joint7": 1.31,
            "gripper_joint1": 0.05,
            "gripper_joint2": -0.05
        },
        # Set initial joint velocities to zero
        joint_vel={".*": 0.0},
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint.*"],
            effort_limit=25.0, # 稍微限制出力，防止瞬间冲击
            velocity_limit=1.5,
            
            # 刚度 (Stiffness)
            stiffness={
                "joint1": 200.0, 
                "joint2": 170.0,
                "joint3": 120.0,
                "joint4": 80.0,
                "joint5": 50.0,
                "joint6": 20.0,
                "joint7": 10.0
            },
            
            # 阻尼 (Damping)：采用临界阻尼思路，比例设在 10% 左右
            damping={
                "joint1": 100.0,
                "joint2": 60.0,
                "joint3": 70.0,
                "joint4": 24.0,
                "joint5": 20.0,
                "joint6": 10.0,
                "joint7": 5,
            },
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper_joint1","gripper_joint2"],
            effort_limit_sim=22,
            velocity_limit_sim=1.5,
            stiffness=800.0,
            damping=20.0,
        ),
    },
    soft_joint_pos_limit_factor=0.9,
)

NERO_GRIPPER_HIGH_PD_CFG = NERO_GRIPPER_CFG.copy()
NERO_GRIPPER_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True