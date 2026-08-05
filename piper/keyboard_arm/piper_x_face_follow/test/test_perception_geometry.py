import numpy as np
from types import SimpleNamespace

from piper_x_face_follow.mediapipe_body_hand_node import (
    deproject,
    exclusive_command_gesture,
    hand_layout_features,
    matrix_quaternion,
    median_depth_m,
    one_two_gesture,
    quaternion_matrix,
    robust_landmark_depth_m,
    typing_plane_rotation,
)


class FakeInfo:
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]


def test_depth_median_ignores_zero_and_outlier():
    depth = np.zeros((7, 7), dtype=np.uint16)
    depth[2:5, 2:5] = 1000
    depth[3, 3] = 5000
    assert median_depth_m(depth, "16UC1", 3, 3, 2, 0.2, 3.0) == 1.0


def test_deprojection_at_principal_point():
    point = deproject(FakeInfo(), 320.0, 240.0, 1.25)
    assert np.allclose(point, [0.0, 0.0, 1.25])


def test_identity_quaternion():
    assert np.allclose(quaternion_matrix(0.0, 0.0, 0.0, 1.0), np.eye(3))


def test_robust_landmark_depth_rejects_background_leak():
    depth = np.full((40, 40), 600, dtype=np.uint16)
    depth[28:33, 28:33] = 1800
    pixels = [(10, 10), (14, 10), (18, 10), (22, 10), (30, 30)]
    value = robust_landmark_depth_m(
        depth, "16UC1", pixels, 1, 0.2, 3.0, minimum_samples=3
    )
    assert np.isclose(value, 0.6)


def test_two_hand_typing_plane_has_lateral_forward_and_camera_facing_normal():
    rotation = typing_plane_rotation(
        [-0.12, 0.02, 0.50],
        [0.12, 0.02, 0.50],
        [0.0, -0.08, 0.02],
        [0.0, -0.08, 0.02],
    )
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
    assert np.linalg.det(rotation) > 0.999
    midpoint = np.asarray([0.0, 0.02, 0.50])
    assert np.dot(rotation[:, 2], -midpoint) > 0.0
    quaternion = matrix_quaternion(rotation)
    assert np.allclose(quaternion_matrix(*quaternion), rotation, atol=1e-7)


def _fake_hand(center_x, center_y):
    points = []
    for index in range(21):
        points.append(
            SimpleNamespace(
                x=center_x + 0.002 * ((index % 4) - 1.5),
                y=center_y - 0.003 * index,
            )
        )
    return SimpleNamespace(landmark=points)


def test_hand_layout_orders_screen_hands_and_requires_all_landmarks_visible():
    layout = hand_layout_features(
        _fake_hand(0.72, 0.60),
        _fake_hand(0.28, 0.59),
    )
    assert layout["visible_count"] == 2
    assert layout["both_complete"]
    assert layout["usable_pair"]
    assert layout["screen_left"]["x"] < layout["screen_right"]["x"]
    assert layout["separation_x"] > 0.3
    assert abs(layout["hand_line_angle_rad"]) < 0.1


def _gesture_hand(extended_fingers):
    points = [SimpleNamespace(x=0.5, y=0.8) for _ in range(21)]
    points[0] = SimpleNamespace(x=0.5, y=0.9)
    for mcp, pip, tip, x in (
        (5, 6, 8, 0.42),
        (9, 10, 12, 0.48),
        (13, 14, 16, 0.54),
        (17, 18, 20, 0.60),
    ):
        points[mcp] = SimpleNamespace(x=x, y=0.72)
        if tip in extended_fingers:
            points[pip] = SimpleNamespace(x=x, y=0.53)
            points[tip] = SimpleNamespace(x=x, y=0.31)
        else:
            points[pip] = SimpleNamespace(x=x, y=0.66)
            points[tip] = SimpleNamespace(x=x + 0.05, y=0.71)
    return SimpleNamespace(landmark=points)


def test_one_two_gesture_requires_folded_ring_and_little_fingers():
    assert one_two_gesture(_gesture_hand({8})) == "one"
    assert one_two_gesture(_gesture_hand({8, 12})) == "two"
    assert one_two_gesture(_gesture_hand({8, 12, 16, 20})) is None


def test_only_one_hand_may_present_a_command_gesture():
    assert exclusive_command_gesture(["one"]) == "one"
    assert exclusive_command_gesture(["two"]) == "two"
    assert exclusive_command_gesture(["one", "two"]) is None
    assert exclusive_command_gesture(["one", "one"]) is None
    assert exclusive_command_gesture([]) is None
