# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym

# from isaaclab_tasks.manager_based.manipulation.stack.config.franka import agents
from agx_teleop.tasks.manager_based.manipulation.stack.config.nero import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Stack-Cube-Nero-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.nero_stack_ik_rel_env_cfg:NeroCubeStackEnvCfg",
        "robomimic_bc_cfg_entry_point": f"{agents.__name__}:robomimic/bc_rnn_low_dim.json",
    },
    disable_env_checker=True,
)