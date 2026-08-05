from pathlib import Path

import numpy as np

from piper_x_face_follow.kinematics import SerialChain, _rpy_matrix
from piper_x_face_follow.servo_math import (
    keyboard_rotation_from_typing_plane,
    vector_alignment_error,
)


URDF = (
    Path(__file__).parents[2]
    / "piper_x_description"
    / "urdf"
    / "piper_x_description_no_gripper.urdf"
)


def test_piper_x_chain_and_translational_jacobian():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.asarray([0.2, 0.8, -1.1, 0.1, -0.2, 1.0])
    position, rotation, jacobian = chain.pose_and_jacobian(joints)
    assert position.shape == (3,)
    assert rotation.shape == (3, 3)
    assert jacobian.shape == (6, 5)
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)

    epsilon = 1.0e-6
    for index in range(5):
        shifted = joints.copy()
        shifted[index] += epsilon
        shifted_position, _, _ = chain.pose_and_jacobian(shifted)
        numerical = (shifted_position - position) / epsilon
        assert np.allclose(jacobian[:3, index], numerical, atol=1e-5)


def test_joint6_is_not_an_active_jacobian_column():
    chain = SerialChain.from_urdf(URDF.read_text())
    _, _, jacobian = chain.pose_and_jacobian([0.0] * 6)
    assert jacobian.shape[1] == 5


def test_camera_offset_point_jacobian_matches_finite_difference():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.asarray([0.2, 0.8, -1.1, 0.1, -0.2, 1.0])
    offset = np.asarray([0.0, 0.0, 0.20])
    camera_position, camera_jacobian = chain.offset_point_and_jacobian(
        joints, offset
    )
    assert camera_jacobian.shape == (3, 5)

    epsilon = 1.0e-6
    for index in range(5):
        shifted = joints.copy()
        shifted[index] += epsilon
        shifted_position, _ = chain.offset_point_and_jacobian(shifted, offset)
        numerical = (shifted_position - camera_position) / epsilon
        assert np.allclose(camera_jacobian[:, index], numerical, atol=1e-5)


def test_virtual_camera_focus_jacobian_includes_self_aim_rotation():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.asarray([0.2, 0.8, -1.1, 0.1, -0.2, 1.0])
    camera_offset = np.asarray([0.0, 0.0, 0.20])
    mount_rotation = _rpy_matrix([0.0, 0.15, 0.0])
    optical_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    focus_in_link6 = camera_offset + mount_rotation @ optical_rotation @ np.asarray(
        [0.0, 0.0, 0.26]
    )
    focus_position, numerical_reference = chain.offset_point_and_jacobian(
        joints, focus_in_link6
    )

    _, link_rotation, geometric = chain.pose_and_jacobian(joints)
    camera_position, camera_jacobian = chain.offset_point_and_jacobian(
        joints, camera_offset
    )
    focus_offset_base = (
        link_rotation @ mount_rotation @ optical_rotation @ np.asarray([0.0, 0.0, 0.26])
    )
    constructed = camera_jacobian.copy()
    for index in range(5):
        constructed[:, index] += np.cross(
            geometric[3:, index], focus_offset_base
        )

    assert np.allclose(focus_position, camera_position + focus_offset_base)
    assert np.allclose(constructed, numerical_reference, atol=1e-9)


def test_virtual_focus_ik_reaches_hand_below_base_with_j6_fixed():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.asarray(
        [0.01343188, -0.012455016, 0.015141392, 0.326359796, -0.61306938, 1.317126664]
    )
    anchor = joints.copy()
    hand_base = np.asarray([0.35, 0.15, -0.42])
    desired_distance = 0.26
    camera_offset = np.asarray([0.0, 0.0, 0.20])
    mount_rotation = _rpy_matrix([0.0, 0.15, 0.0])
    optical_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    bounds = np.asarray(
        [
            [-2.568, 2.568],
            [-0.05, 3.09],
            [-2.9171, 0.05],
            [-1.52, 1.52],
            [-1.52, 1.52],
        ]
    )
    offsets = np.radians([45.0, 160.0, 165.0, 88.0, 88.0])

    for _ in range(800):
        camera_position, camera_jacobian = chain.offset_point_and_jacobian(
            joints, camera_offset
        )
        _, link_rotation, geometric = chain.pose_and_jacobian(joints)
        camera_rotation = link_rotation @ mount_rotation @ optical_rotation
        focus_offset = camera_rotation[:, 2] * desired_distance
        error = hand_base - (camera_position + focus_offset)
        if np.linalg.norm(error) < 0.006:
            break
        error *= min(1.0, 0.002 / np.linalg.norm(error))
        focus_jacobian = camera_jacobian.copy()
        for index in range(5):
            focus_jacobian[:, index] += np.cross(
                geometric[3:, index], focus_offset
            )
        inverse = focus_jacobian.T @ np.linalg.inv(
            focus_jacobian @ focus_jacobian.T + 0.012**2 * np.eye(3)
        )
        joints[:5] += inverse @ error
        for index in range(5):
            joints[index] = np.clip(
                joints[index],
                max(bounds[index, 0], anchor[index] - offsets[index]),
                min(bounds[index, 1], anchor[index] + offsets[index]),
            )

    camera_position, _ = chain.offset_point_and_jacobian(joints, camera_offset)
    _, link_rotation, _ = chain.pose_and_jacobian(joints)
    camera_rotation = link_rotation @ mount_rotation @ optical_rotation
    hand_camera = camera_rotation.T @ (hand_base - camera_position)
    assert camera_position[2] < -0.15
    assert np.linalg.norm(hand_camera - [0.0, 0.0, desired_distance]) < 0.006
    assert joints[5] == anchor[5]


def test_physical_camera_mount_points_outward_and_approach_does_not_retract():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.asarray(
        [0.01343188, -0.012455016, 0.015141392, 0.326359796, -0.61306938, 1.31976]
    )
    tcp = np.asarray([-0.19504, -0.02223, -0.03827])
    mount_rotation = _rpy_matrix([1.64437, -1.17497, -1.45575])
    optical_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    link_position, link_rotation, _ = chain.pose_and_jacobian(joints)
    camera_position, camera_jacobian = chain.offset_point_and_jacobian(
        joints, tcp
    )
    optical_forward = link_rotation @ mount_rotation @ optical_rotation[:, 2]
    outward = link_position.copy()
    outward[2] = 0.0
    outward /= np.linalg.norm(outward)

    assert np.isclose(camera_position[2] - link_position[2], 0.20, atol=1e-4)
    assert np.dot(optical_forward[:2], outward[:2]) > 0.95
    assert -0.30 < optical_forward[2] < 0.0

    error = optical_forward * 0.002
    inverse = camera_jacobian.T @ np.linalg.inv(
        camera_jacobian @ camera_jacobian.T + 0.012**2 * np.eye(3)
    )
    predicted_camera_motion = camera_jacobian @ (inverse @ error)
    assert np.dot(predicted_camera_motion, optical_forward) > 0.0018


def test_close_hand_retreats_and_typing_plane_uses_wrist_nullspace():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.radians([24.4, 99.9, -46.4, 27.4, 3.6, 75.632])
    tcp = np.asarray([-0.19504, -0.02223, -0.03827])
    mount_rotation = _rpy_matrix([1.64437, -1.17497, -1.45575])
    optical_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    )
    _, camera_jacobian = chain.offset_point_and_jacobian(joints, tcp)
    _, link_rotation, geometric = chain.pose_and_jacobian(joints)
    camera_rotation = link_rotation @ mount_rotation @ optical_rotation

    retreat_error = camera_rotation @ np.asarray([0.0, 0.0, 0.203 - 0.26])
    inverse = camera_jacobian.T @ np.linalg.inv(
        camera_jacobian @ camera_jacobian.T + 0.012**2 * np.eye(3)
    )
    retreat_motion_camera = camera_rotation.T @ (
        camera_jacobian @ (inverse @ retreat_error)
    )
    assert retreat_motion_camera[2] < -0.05

    typing_rotation = np.column_stack(
        ([0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0])
    )
    desired_rotation = keyboard_rotation_from_typing_plane(
        typing_rotation, link_rotation, np.radians(25.0)
    )
    normal_error = vector_alignment_error(
        -link_rotation[:, 0], -desired_rotation[:, 0]
    )
    position_nullspace = np.eye(5) - np.linalg.pinv(
        camera_jacobian, rcond=0.02
    ) @ camera_jacobian
    null_angular = geometric[3:, :] @ position_nullspace
    inverse = null_angular.T @ np.linalg.inv(
        null_angular @ null_angular.T + 0.10**2 * np.eye(3)
    )
    joint_delta = position_nullspace @ inverse @ normal_error
    assert np.linalg.norm(joint_delta[3:5]) > np.radians(1.0)
    # First-order tray orientation is corrected without commanding the camera
    # to descend or translate.
    assert np.linalg.norm(camera_jacobian @ joint_delta) < 1.0e-8
    corrected_error = normal_error - geometric[3:, :] @ joint_delta
    assert np.linalg.norm(corrected_error) < np.linalg.norm(normal_error)


def test_real_logged_low_head_pose_requires_negative_j4_and_j5_to_level():
    chain = SerialChain.from_urdf(URDF.read_text())
    joints = np.radians([8.5, 56.6, -0.5, 29.6, -9.2, 75.486])
    _, rotation, geometric = chain.pose_and_jacobian(joints)
    tray_normal = -rotation[:, 0]
    level_error = vector_alignment_error(tray_normal, [0.0, 0.0, 1.0])
    wrist = geometric[3:, 3:5]
    inverse = wrist.T @ np.linalg.inv(
        wrist @ wrist.T + 0.08**2 * np.eye(3)
    )
    correction = inverse @ level_error
    assert correction[0] < 0.0
    assert correction[1] < 0.0
