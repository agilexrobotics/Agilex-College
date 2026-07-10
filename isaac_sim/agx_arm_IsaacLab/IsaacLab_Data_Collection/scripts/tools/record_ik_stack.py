import argparse
import contextlib

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Record demonstrations for Isaac Lab environments using IK automation.")
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument(
    "--dataset_file", type=str, default="./datasets/ik_dataset.hdf5", help="File path to export recorded demos."
)
parser.add_argument("--step_hz", type=int, default=30, help="Environment stepping rate in Hz.")
parser.add_argument(
    "--num_demos", type=int, default=0, help="Number of demonstrations to record. Set to 0 for infinite."
)
parser.add_argument(
    "--num_success_steps",
    type=int,
    default=10,
    help="Number of continuous steps with task success for concluding a demo as successful. Default is 10.",
)
parser.add_argument(
    "--enable_pinocchio",
    action="store_true",
    default=False,
    help="Enable Pinocchio.",
)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# Validate required arguments
if args_cli.task is None:
    parser.error("--task is required")

app_launcher_args = vars(args_cli)

if args_cli.enable_pinocchio:
    import pinocchio  # noqa: F401

# launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


import logging
import os
import time

import gymnasium as gym
import torch

import isaaclab_mimic.envs  # noqa: F401

if args_cli.enable_pinocchio:
    import isaaclab_tasks.manager_based.locomanipulation.pick_place  # noqa: F401
    import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401

from isaaclab.envs import DirectRLEnvCfg, ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import DatasetExportMode

import agx_teleop.tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

logger = logging.getLogger(__name__)


class RateLimiter:
    """Convenience class for enforcing rates in loops."""

    def __init__(self, hz: int):
        self.hz = hz
        self.last_time = time.time()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.033, self.sleep_duration)

    def sleep(self, env: gym.Env):
        next_wakeup_time = self.last_time + self.sleep_duration
        while time.time() < next_wakeup_time:
            time.sleep(self.render_period)
            env.sim.render()
        self.last_time = self.last_time + self.sleep_duration
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration


class IKStackController:
    """Automatic pick-and-place controller for the Stack-Cube task.

    Generates smooth delta-pose trajectories (PCHIP interpolated) and feeds
    them to an ``IK-Rel`` environment whose action space is::

        [Δx, Δy, Δz, Δrx, Δry, Δrz, gripper_binary]   — shape (1, 7)

    The environment's built-in DifferentialIK action term handles the
    inverse kinematics internally, so this controller only needs to output
    Cartesian deltas.

    Gripper convention (from BinaryJointPositionActionCfg):
        > 0  →  open   (gripper_joint1=0.05, gripper_joint2=-0.05)
        < 0  →  close  (gripper_joint1=0.0,  gripper_joint2=0.0)
    """

    # gripper binary values
    GRIP_OPEN_VAL = 1.0
    GRIP_CLOSE_VAL = -1.0

    # z-offsets for each motion phase
    Z_PRE = 0.18
    Z_DOWN = 0.0
    Z_LIFT = 0.25
    Z_PLACE = 0.005
    Z_RETREAT = 0.20

    def __init__(self, env: gym.Env) -> None:
        self.env = env
        self.robot = env.scene["robot"]
        self.ee_frame = env.scene["ee_frame"]
        self.cubes = [env.scene[f"cube_{i}"] for i in range(1, 4)]

        # trajectory state (populated by plan())
        self._path: torch.Tensor | None = None  # [Total, 1, 3]
        self._gripper_timeline: list[float] = []
        self._total_steps: int = 0
        self._step_idx: int = 0
        self._last_gripper_val: float = self.GRIP_OPEN_VAL
        self._post_traj_wait_steps: int = 60
        self._post_traj_wait_count: int = 0

    def plan(self) -> None:
        """plan the full pick-and-place trajectory."""
        cube_height = self.cubes[0].data.root_pos_w[:, 2:3].clone() * 2.0

        all_waypoints: list[torch.Tensor] = []
        all_gripper_vals: list[float] = []
        all_durations: list[int] = []

        base_pos = self.cubes[0].data.root_pos_w.clone()

        for i in range(1, len(self.cubes)):
            src = self.cubes[i].data.root_pos_w.clone()
            place_pos = self._shift_z(base_pos, cube_height)

            sequence = [
                (self._shift_z(src, self.Z_PRE), self.GRIP_OPEN_VAL, 30),
                (self._shift_z(src, self.Z_DOWN), self.GRIP_OPEN_VAL, 30),
                (self._shift_z(src, self.Z_DOWN), self.GRIP_CLOSE_VAL, 30),
                (self._shift_z(src, self.Z_LIFT), self.GRIP_CLOSE_VAL, 30),
                (self._shift_z(place_pos, self.Z_PRE), self.GRIP_CLOSE_VAL, 30),
                (self._shift_z(place_pos, self.Z_PLACE), self.GRIP_CLOSE_VAL, 30),
                (self._shift_z(place_pos, self.Z_PLACE), self.GRIP_OPEN_VAL, 20),
                (self._shift_z(place_pos, self.Z_RETREAT), self.GRIP_OPEN_VAL, 30),
            ]

            for wp, gv, dur in sequence:
                all_waypoints.append(wp.squeeze(0))
                all_gripper_vals.append(gv)
                all_durations.append(dur)

            all_durations[-1] += 30
            base_pos = self._shift_z(base_pos, cube_height)

        # Generate smooth trajectory
        self._path = self._generate_smooth_trajectory(all_waypoints, all_durations)

        # Expand gripper timeline
        self._gripper_timeline = []
        for gv, dur in zip(all_gripper_vals, all_durations):
            self._gripper_timeline.extend([gv] * dur)

        self._total_steps = self._path.shape[0]
        self._gripper_timeline = self._gripper_timeline[:self._total_steps]

        self._step_idx = 0
        self._post_traj_wait_count = 0
        self._last_gripper_val = self.GRIP_OPEN_VAL

        print(f"[IKStackController] Trajectory planned: {self._total_steps} steps")

    def compute_action(self, env: gym.Env) -> torch.Tensor:
        """Return the action for the current step.

        Returns:
            torch.Tensor of shape ``(1, 7)``:
                ``[Δx, Δy, Δz, Δrx, Δry, Δrz, gripper_binary]``
        """
        # trajectory exhausted → hold position
        if self._step_idx >= self._total_steps:
            return torch.tensor(
                [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self._last_gripper_val]],
                device=env.device,
            )

        # delta position = target − current
        target_pos = self._path[self._step_idx]  # [1, 3]
        current_pos = self.ee_frame.data.target_pos_w[:, 0, :]  # [1, 3]
        delta_pos = target_pos - current_pos  # [1, 3]

        # delta rotation = zero (keep orientation)
        delta_rot = torch.zeros(1, 3, device=env.device)

        # gripper binary command
        gv = self._gripper_timeline[self._step_idx]
        grip_tensor = torch.tensor([[gv]], device=env.device)

        action = torch.cat([delta_pos, delta_rot, grip_tensor], dim=-1)  # (1, 7)

        self._step_idx += 1
        self._last_gripper_val = gv

        return action

    def is_complete(self) -> bool:
        """Whether the planned trajectory has been fully executed."""
        return self._step_idx >= self._total_steps

    def should_reset(self) -> bool:
        """Whether enough post-trajectory steps have passed to trigger a reset.

        Gives the physics sim time to settle and the success condition time
        to fire before requesting a reset.
        """
        if self.is_complete():
            self._post_traj_wait_count += 1
            if self._post_traj_wait_count >= self._post_traj_wait_steps:
                return True
        return False

    @staticmethod
    def _shift_z(pos: torch.Tensor, dz: float) -> torch.Tensor:
        out = pos.clone()
        out[:, 2:3] += dz
        return out

    @staticmethod
    def _generate_smooth_trajectory(
        waypoints: list[torch.Tensor], durations: list[int]
    ) -> torch.Tensor:
        """Interpolate waypoints with PCHIP-style tangents.

        Args:
            waypoints: list of ``(3,)`` tensors.
            durations: steps per segment (``len == len(waypoints) - 1``).

        Returns:
            ``torch.Tensor`` of shape ``(Total, 1, 3)``.
        """
        n = len(waypoints)
        wp = torch.stack(waypoints)  # [N, 3]

        tangents = torch.zeros_like(wp)
        if n > 1:
            tangents[0] = wp[1] - wp[0]
            tangents[-1] = wp[-1] - wp[-2]
            for i in range(1, n - 1):
                tangents[i] = (wp[i + 1] - wp[i - 1]) / 2.0

        segments = []
        for i in range(n - 1):
            p0, p1 = wp[i], wp[i + 1]
            m0, m1 = tangents[i], tangents[i + 1]
            steps = durations[i]

            if torch.norm(p1 - p0) < 1e-4:
                seg = p0.unsqueeze(0).repeat(steps, 1)
            else:
                s = torch.linspace(0, 1, steps, device=p0.device).unsqueeze(-1)
                h00 = 2 * s**3 - 3 * s**2 + 1
                h10 = s**3 - 2 * s**2 + s
                h01 = -2 * s**3 + 3 * s**2
                h11 = s**3 - s**2
                seg = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1

            segments.append(seg)

        return torch.cat(segments, dim=0).unsqueeze(1)  # [Total, 1, 3]


def setup_output_directories() -> tuple[str, str]:
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")
    return output_dir, output_file_name


def create_environment_config(
    output_dir: str, output_file_name: str
) -> tuple[ManagerBasedRLEnvCfg | DirectRLEnvCfg, object | None]:
    try:
        env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
        env_cfg.env_name = args_cli.task.split(":")[-1]
    except Exception as e:
        logger.error(f"Failed to parse environment configuration: {e}")
        exit(1)

    success_term = None
    if hasattr(env_cfg.terminations, "success"):
        success_term = env_cfg.terminations.success
        env_cfg.terminations.success = None
    else:
        logger.warning(
            "No success termination term was found in the environment."
            " Will not be able to mark recorded demos as successful."
        )

    env_cfg.terminations.time_out = None
    env_cfg.observations.policy.concatenate_terms = False

    env_cfg.recorders: ActionStateRecorderManagerCfg = ActionStateRecorderManagerCfg()
    env_cfg.recorders.dataset_export_dir_path = output_dir
    env_cfg.recorders.dataset_filename = output_file_name
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY

    return env_cfg, success_term


def create_environment(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg) -> gym.Env:
    try:
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        return env
    except Exception as e:
        logger.error(f"Failed to create environment: {e}")
        exit(1)


def process_success_condition(env: gym.Env, success_term: object | None, success_step_count: int) -> tuple[int, bool]:
    if success_term is None:
        return success_step_count, False
    if bool(success_term.func(env, **success_term.params)[0]):
        success_step_count += 1
        if success_step_count >= args_cli.num_success_steps:
            env.recorder_manager.record_pre_reset([0], force_export_or_skip=False)
            env.recorder_manager.set_success_to_episodes(
                [0], torch.tensor([[True]], dtype=torch.bool, device=env.device)
            )
            env.recorder_manager.export_episodes([0])
            print("Success condition met! Recording completed.")
            return success_step_count, True
    else:
        success_step_count = 0
    return success_step_count, False


def handle_reset(
    env: gym.Env, success_step_count: int
) -> int:
    print("Resetting environment...")
    env.sim.reset()
    env.recorder_manager.reset()
    env.reset()
    success_step_count = 0
    return success_step_count


def run_simulation_loop(
    env: gym.Env,
    success_term: object | None,
    rate_limiter: RateLimiter | None,
    ik_controller: IKStackController,
) -> int:
    current_recorded_demo_count = 0
    success_step_count = 0
    should_reset_recording_instance = False
    running_recording_instance = True  # IK is always "running"

    # initial reset + plan
    env.sim.reset()
    env.reset()
    ik_controller.plan()

    label_text = f"Recorded {current_recorded_demo_count} successful demonstrations."
    print(label_text)

    subtasks = {}

    with contextlib.suppress(KeyboardInterrupt) and torch.inference_mode():
        while simulation_app.is_running():
            # IK action
            actions = ik_controller.compute_action(env)

            # step environment
            if running_recording_instance:
                obv = env.step(actions)
                if subtasks is not None:
                    if subtasks == {}:
                        subtasks = obv[0].get("subtask_terms")
            else:
                env.sim.render()

            # success check
            success_step_count, success_reset_needed = process_success_condition(
                env, success_term, success_step_count
            )
            if success_reset_needed:
                should_reset_recording_instance = True

            # update label
            if env.recorder_manager.exported_successful_episode_count > current_recorded_demo_count:
                current_recorded_demo_count = env.recorder_manager.exported_successful_episode_count
                label_text = f"Recorded {current_recorded_demo_count} successful demonstrations."
                print(label_text)

            # enough demos?
            if args_cli.num_demos > 0 and env.recorder_manager.exported_successful_episode_count >= args_cli.num_demos:
                label_text = f"All {current_recorded_demo_count} demonstrations recorded.\nExiting the app."
                print(label_text)
                target_time = time.time() + 0.8
                while time.time() < target_time:
                    if rate_limiter:
                        rate_limiter.sleep(env)
                    else:
                        env.sim.render()
                break

            # auto-reset when trajectory done
            if ik_controller.should_reset():
                print("[IKStackController] Trajectory complete, triggering reset.")
                should_reset_recording_instance = True

            # reset
            if should_reset_recording_instance:
                success_step_count = handle_reset(env, success_step_count)
                ik_controller.plan()  # re-plan with new cube positions
                should_reset_recording_instance = False

            # sim stopped?
            if env.sim.is_stopped():
                break

            # rate limiting
            if rate_limiter:
                rate_limiter.sleep(env)

    return current_recorded_demo_count



def main() -> None:
    rate_limiter = RateLimiter(args_cli.step_hz)

    output_dir, output_file_name = setup_output_directories()

    global env_cfg
    env_cfg, success_term = create_environment_config(output_dir, output_file_name)

    env = create_environment(env_cfg)

    ik_controller = IKStackController(env)

    current_recorded_demo_count = run_simulation_loop(env, success_term, rate_limiter, ik_controller)

    env.close()
    print(f"Recording session completed with {current_recorded_demo_count} successful demonstrations")
    print(f"Demonstrations saved to: {args_cli.dataset_file}")


if __name__ == "__main__":
    main()
    simulation_app.close()