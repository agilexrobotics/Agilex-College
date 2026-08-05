import numpy as np

from piper_x_face_follow.servo_math import (
    bounded_single_hand_j5_step,
    camera_gaze_error,
    camera_ray_alignment_error,
    camera_tracking_error,
    demonstrated_j5_layout_step,
    enforce_base_downward_trend,
    keyboard_rotation_from_typing_plane,
    position_level_ik_step,
    physical_j4_vertical_search_step,
    single_joint_axis_alignment_step,
    staged_camera_error,
    visual_search_command,
    workspace_limited_step,
)


def test_occlusion_search_forces_base_z_down_without_changing_xy():
    error = enforce_base_downward_trend(
        [0.0015, -0.0002, 0.0008], True, 0.00035
    )
    assert np.allclose(error, [0.0015, -0.0002, -0.00035])


def test_inactive_occlusion_search_preserves_cartesian_error():
    error = enforce_base_downward_trend(
        [0.001, 0.002, 0.003], False, 0.00035
    )
    assert np.allclose(error, [0.001, 0.002, 0.003])


def test_far_centered_hands_request_approach():
    error, action, distance = camera_tracking_error(
        [0.0, 0.0, 0.50], [0.0, 0.0], 0.22, 0.30
    )
    assert action == "approach"
    assert np.isclose(distance, 0.50)
    assert np.allclose(error, [0.0, 0.0, 0.20])


def test_close_centered_hands_request_retreat():
    error, action, distance = camera_tracking_error(
        [0.0, 0.0, 0.15], [0.0, 0.0], 0.22, 0.30
    )
    assert action == "retreat"
    assert np.isclose(distance, 0.15)
    assert np.allclose(error, [0.0, 0.0, -0.07])


def test_in_range_off_axis_hands_only_request_centering():
    error, action, distance = camera_tracking_error(
        [0.10, -0.02, 0.25], [0.0, 0.0], 0.22, 0.30
    )
    assert action == "hold_range"
    assert 0.22 < distance < 0.30
    assert np.allclose(error, [0.10, -0.02, 0.0])


def test_workspace_allows_below_base_but_guards_floor_and_base_radius():
    current = np.asarray([0.30, 0.0, -0.20])
    step = workspace_limited_step(
        current, [-0.20, 0.0, -0.30], 0.07, -0.35, 0.75, 0.18
    )
    candidate = current + step
    assert np.isclose(candidate[2], -0.35)
    assert np.isclose(np.linalg.norm(candidate[:2]), 0.18)


def test_camera_gaze_error_has_self_aim_sign():
    centered = camera_gaze_error([0.0, 0.0, 1.0])
    right = camera_gaze_error([0.2, 0.0, 1.0])
    down = camera_gaze_error([0.0, 0.2, 1.0])
    assert np.allclose(centered, np.zeros(3))
    assert right[1] > 0.0
    assert down[0] < 0.0


def test_distance_stage_ignores_lateral_noise_and_approaches_far_hand():
    error = staged_camera_error(
        0.65, 0.26, "distance_acquire", midpoint_xy=[0.2, 0.2]
    )
    assert np.allclose(error, [0.0, 0.0, 0.39])


def test_align_stage_moves_down_after_distance_is_acquired():
    error = staged_camera_error(
        0.27, 0.26, "hand_align", midpoint_xy=[0.0, 0.18], xy_gain=0.6
    )
    assert np.allclose(error, [0.0, 0.108, 0.01])


def test_align_stage_respects_below_camera_typing_offset():
    error = staged_camera_error(
        0.26,
        0.26,
        "hand_align",
        midpoint_xy=[0.0, 0.065],
        desired_xy=[0.0, 0.065],
        xy_gain=0.18,
    )
    assert np.allclose(error, 0.0)


def test_camera_ray_alignment_tilts_toward_hand():
    desired = np.asarray([0.0, 0.065, 0.26])
    measured = np.asarray([0.0, 0.12, 0.26])
    error = camera_ray_alignment_error(measured, desired)
    assert error[0] < 0.0
    assert np.isclose(error[1], 0.0)
    assert np.isclose(error[2], 0.0)


def test_keyboard_plane_mapping_keeps_tray_up_and_maps_lateral_axis():
    tilt = np.radians(40.0)
    typing = np.column_stack(
        (
            [0.0, 1.0, 0.0],
            [np.cos(tilt), 0.0, -np.sin(tilt)],
            [-np.sin(tilt), 0.0, -np.cos(tilt)],
        )
    )
    current = np.eye(3)
    desired = keyboard_rotation_from_typing_plane(
        typing, current, np.radians(25.0)
    )
    tray_normal = -desired[:, 0]
    assert np.allclose(desired.T @ desired, np.eye(3), atol=1e-7)
    assert np.linalg.det(desired) > 0.999
    assert tray_normal[2] > 0.0
    assert np.isclose(
        np.degrees(np.arccos(tray_normal[2])), 25.0, atol=1e-6
    )
    assert np.dot(desired[:, 1], current[:, 1]) > 0.0


def test_visual_search_centers_before_approach_and_reacquires_single_hand():
    command, phase, action = visual_search_command(
        [0.12, -0.04, 0.60], 2, False, 0.26, 0.22, 0.30, 0.035
    )
    assert np.allclose(command, [0.12, -0.04, 0.0])
    assert phase == "visual_center"
    assert action == "center_both_hands"

    command, phase, action = visual_search_command(
        [0.10, 0.03, 0.40], 1, False, 0.26, 0.22, 0.30, 0.035
    )
    assert np.allclose(command, [0.10, 0.03, -0.035])
    assert phase == "single_reacquire"
    assert action == "single_hand_search"


def test_visual_search_approaches_only_after_both_hands_are_centered():
    command, phase, action = visual_search_command(
        [0.01, -0.01, 0.55], 2, True, 0.26, 0.22, 0.30, 0.035
    )
    assert command[2] > 0.0
    assert phase == "depth_approach"
    assert action == "centered_approach"

    command, phase, action = visual_search_command(
        [0.01, -0.01, 0.19], 1, False, 0.26, 0.22, 0.30, 0.035
    )
    assert np.allclose(command[:2], 0.0)
    assert command[2] < 0.0
    assert action == "distance_retreat"


def test_position_level_ik_uses_wrist_to_escape_translation_singularity():
    position_jacobian = np.asarray(
        [
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.3, 0.0],
            [0.0, 0.0, 0.0, 0.6, 0.0],
        ]
    )
    angular_jacobian = np.asarray(
        [
            [0.0, 0.0, 1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0, 0.0],
        ]
    )
    delta = position_level_ik_step(
        position_jacobian,
        angular_jacobian,
        [0.0, 0.0, -0.01],
        [0.0, 0.0, 1.0],
    )
    assert delta.shape == (5,)
    assert delta[3] < 0.0
    assert np.all(np.isfinite(delta))


def test_position_level_ik_penalizes_unnecessary_j5_rotation():
    position_jacobian = np.hstack((np.eye(3), np.zeros((3, 2))))
    angular_jacobian = np.asarray(
        [
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0, 0.0],
        ]
    )
    delta = position_level_ik_step(
        position_jacobian,
        angular_jacobian,
        [0.01, -0.02, 0.03],
        [0.0, 0.0, 1.0],
    )
    assert abs(delta[4]) < 1.0e-8


def test_j5_alignment_uses_urdf_axis_for_sign():
    positive, error = single_joint_axis_alignment_step(
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        gain=0.5,
        maximum_error_rad=np.radians(20.0),
    )
    negative, _ = single_joint_axis_alignment_step(
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
        gain=0.5,
        maximum_error_rad=np.radians(20.0),
    )
    assert np.isclose(error, np.radians(20.0))
    assert positive > 0.0
    assert negative < 0.0


def test_j5_alignment_treats_lateral_axis_as_undirected():
    step, error = single_joint_axis_alignment_step(
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    )
    assert np.isclose(step, 0.0)
    assert np.isclose(error, 0.0)


def test_physical_j4_search_moves_down_for_hand_below_image_center():
    down = physical_j4_vertical_search_step(
        np.radians(12.0), 0.25, 1.0, np.radians(4.0), np.radians(3.0)
    )
    up = physical_j4_vertical_search_step(
        np.radians(-12.0), 0.25, 1.0, np.radians(4.0), np.radians(3.0)
    )
    centered = physical_j4_vertical_search_step(
        np.radians(2.0), 0.25, 1.0, np.radians(4.0), np.radians(3.0)
    )
    assert np.isclose(down, np.radians(3.0))
    assert np.isclose(up, np.radians(-3.0))
    assert centered == 0.0


def test_real_teaching_calibration_turns_j5_toward_level_hand_line():
    # Real recording start: line=+0.219 rad and J5=3.86 deg.  The demonstrated
    # centered pose later had line=-0.020 rad and J5=48.8 deg, establishing
    # that a positive line error needs a positive J5 correction.
    start_step = demonstrated_j5_layout_step(
        0.219, 0.35, 1.0, np.radians(2.5), np.radians(8.0)
    )
    demonstrated_step = demonstrated_j5_layout_step(
        -0.020, 0.35, 1.0, np.radians(2.5), np.radians(8.0)
    )
    opposite_step = demonstrated_j5_layout_step(
        -0.150, 0.35, 1.0, np.radians(2.5), np.radians(8.0)
    )
    assert start_step > 0.0
    assert demonstrated_step == 0.0
    assert opposite_step < 0.0


def test_single_hand_j5_search_is_bounded_to_one_arc():
    step, at_limit = bounded_single_hand_j5_step(
        np.radians(47.0),
        np.radians(47.0),
        1.0,
        np.radians(6.0),
        np.radians(20.0),
    )
    assert np.isclose(step, np.radians(6.0))
    assert not at_limit

    step, at_limit = bounded_single_hand_j5_step(
        np.radians(66.0),
        np.radians(47.0),
        1.0,
        np.radians(6.0),
        np.radians(20.0),
    )
    assert np.isclose(step, np.radians(1.0))
    assert not at_limit

    step, at_limit = bounded_single_hand_j5_step(
        np.radians(67.0),
        np.radians(47.0),
        1.0,
        np.radians(6.0),
        np.radians(20.0),
    )
    assert step == 0.0
    assert at_limit
