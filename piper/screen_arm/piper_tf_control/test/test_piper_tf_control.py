import math
from types import SimpleNamespace

from piper_tf_control.piper_tf_control_node import (
    make_pos_cmd,
    pose_to_target_values,
    smooth_pose_toward,
    quaternion_to_euler_xyz,
    transform_to_target_values,
)


def make_transform(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
            rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        )
    )


def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_quaternion_to_euler_xyz_identity():
    assert quaternion_to_euler_xyz(0.0, 0.0, 0.0, 1.0) == (0.0, 0.0, 0.0)


def test_transform_values_use_base_frame_translation():
    values = transform_to_target_values(
        make_transform(
            0.25,
            -0.02,
            0.30,
            qz=math.sin(math.pi / 4),
            qw=math.cos(math.pi / 4),
        )
    )
    assert values[:3] == (0.25, -0.02, 0.30)
    assert math.isclose(values[5], math.pi / 2.0, abs_tol=1e-9)


def test_pose_values_can_hold_feedback_pose():
    values = pose_to_target_values(
        make_pose(
            0.11,
            -0.12,
            0.21,
            qz=math.sin(math.pi / 4),
            qw=math.cos(math.pi / 4),
        )
    )
    assert values[:3] == (0.11, -0.12, 0.21)
    assert math.isclose(values[5], math.pi / 2.0, abs_tol=1e-9)


def test_make_pos_cmd_maps_pitch_and_yaw_fields():
    command = make_pos_cmd((0.1, 0.2, 0.3, 0.4, 0.5, 0.6), 0.0, 0, 0)
    assert command.x == 0.1
    assert command.y == 0.2
    assert command.z == 0.3
    assert command.roll == 0.4
    assert command.pitch == 0.5
    assert command.yaw == 0.6


def test_smooth_pose_toward_limits_translation_and_rotation():
    result = smooth_pose_toward(
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, -1.0, 0.5, math.pi, -math.pi, 1.0),
        0.1,
        0.2,
    )
    assert result[:3] == (0.1, -0.1, 0.1)
    assert math.isclose(result[3], 0.2, abs_tol=1e-9)
    assert math.isclose(result[4], -0.2, abs_tol=1e-9)
    assert math.isclose(result[5], 0.2, abs_tol=1e-9)


def test_smooth_pose_toward_holds_inside_deadband():
    result = smooth_pose_toward(
        (0.10, -0.10, 0.20, 0.30, -0.30, 0.10),
        (0.101, -0.099, 0.201, 0.31, -0.31, 0.11),
        0.1,
        0.2,
        position_deadband=0.002,
        angle_deadband=0.03,
    )
    expected = (0.10, -0.10, 0.20, 0.30, -0.30, 0.10)
    for actual, expected_value in zip(result, expected):
        assert math.isclose(actual, expected_value, abs_tol=1e-9)
