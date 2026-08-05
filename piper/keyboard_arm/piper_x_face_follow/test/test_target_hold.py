import numpy as np

from piper_x_face_follow.cartesian_hand_controller import (
    hand_loss_home_due,
    hold_target_changed,
)


def test_target_hold_ignores_small_hand_jitter():
    reference = np.asarray([0.02, -0.01, 0.27])
    current = np.asarray([0.06, 0.02, 0.31])

    assert not hold_target_changed(reference, current, 0.10, 0.10)


def test_target_hold_releases_for_large_lateral_change():
    reference = np.asarray([0.0, 0.0, 0.27])
    current = np.asarray([0.11, 0.0, 0.27])

    assert hold_target_changed(reference, current, 0.10, 0.10)


def test_target_hold_releases_for_large_depth_change():
    reference = np.asarray([0.0, 0.0, 0.27])
    current = np.asarray([0.0, 0.0, 0.38])

    assert hold_target_changed(reference, current, 0.10, 0.10)


def test_hand_loss_returns_home_only_after_acquired_timeout():
    assert not hand_loss_home_due(False, 10.0, 20.0, 6.0)
    assert not hand_loss_home_due(True, 10.0, 15.9, 6.0)
    assert hand_loss_home_due(True, 10.0, 16.0, 6.0)


def test_hand_loss_home_is_disabled_by_zero_timeout():
    assert not hand_loss_home_due(True, 10.0, 30.0, 0.0)
