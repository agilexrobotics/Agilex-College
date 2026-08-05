import math

import pytest

from piper_x_face_follow.policy import (
    FaceStabilityFilter,
    deadband,
    enforce_j6_lock,
    ordered_joints,
    slew,
)


def test_j6_is_always_replaced_by_latched_angle():
    lock = math.radians(12.3)
    command = enforce_j6_lock([1, 2, 3, 4, 5, -99], lock)
    assert command[5] == lock


def test_invalid_joint_command_is_rejected():
    with pytest.raises(ValueError):
        enforce_j6_lock([0, 1], 0.0)
    with pytest.raises(ValueError):
        enforce_j6_lock([0, 0, 0, 0, 0, float("nan")], 0.0)


def test_feedback_is_reordered_by_joint_name():
    values = ordered_joints(
        ["joint6", "joint2", "joint1", "joint5", "joint3", "joint4"],
        [6, 2, 1, 5, 3, 4],
    )
    assert values == (1, 2, 3, 4, 5, 6)


def test_face_must_be_stable_for_full_window():
    filt = FaceStabilityFilter(3, 5.0)
    assert filt.add(100, 100) is None
    assert filt.add(102, 101) is None
    stable = filt.add(101, 99)
    assert stable is not None
    assert stable.u == pytest.approx(101.0)
    assert filt.add(120, 100) is None


def test_deadband_and_slew_limit():
    assert deadband(5.0, 10.0) == 0.0
    assert deadband(15.0, 10.0) == 5.0
    assert slew([0.0, 0.0], [2.0, -2.0], 0.1) == pytest.approx((0.1, -0.1))
