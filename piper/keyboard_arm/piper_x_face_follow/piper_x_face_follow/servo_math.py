"""Pure math helpers for the J6-locked eye-in-hand servo."""

import math
from typing import Sequence, Tuple

import numpy as np


def camera_tracking_error(
    hand_point: Sequence[float],
    desired_xy: Sequence[float],
    minimum_distance: float,
    maximum_distance: float,
) -> Tuple[np.ndarray, str, float]:
    """Return camera translation that centers the hands and enforces a range.

    The point is expressed in the camera optical frame.  Lateral components
    center the midpoint.  Outside the 3D distance interval, the final centered
    target is placed on the optical axis at the nearest distance boundary.
    """
    point = np.asarray(hand_point, dtype=float)
    center = np.asarray(desired_xy, dtype=float)
    if point.shape != (3,) or center.shape != (2,):
        raise ValueError("hand_point must be xyz and desired_xy must contain x/y")
    distance = float(np.linalg.norm(point))
    error = np.asarray(
        [point[0] - center[0], point[1] - center[1], 0.0], dtype=float
    )
    if distance > maximum_distance:
        error[2] = point[2] - maximum_distance
        action = "approach"
    elif distance < minimum_distance:
        error[2] = point[2] - minimum_distance
        action = "retreat"
    else:
        action = "hold_range"
    return error, action, distance


def workspace_limited_step(
    current_position: Sequence[float],
    requested_step: Sequence[float],
    minimum_radial_distance: float,
    minimum_z: float,
    maximum_z: float,
    below_base_minimum_radial: float = None,
) -> np.ndarray:
    """Project one camera-position step into a simple table-edge workspace."""
    current = np.asarray(current_position, dtype=float)
    requested = np.asarray(requested_step, dtype=float)
    if current.shape != (3,) or requested.shape != (3,):
        raise ValueError("positions must be xyz vectors")
    candidate = current + requested
    candidate[2] = float(np.clip(candidate[2], minimum_z, maximum_z))
    required_radial = float(minimum_radial_distance)
    if candidate[2] < 0.0 and below_base_minimum_radial is not None:
        required_radial = max(required_radial, float(below_base_minimum_radial))
    radial = float(np.linalg.norm(candidate[:2]))
    if radial < required_radial:
        direction = candidate[:2]
        direction_norm = radial
        if direction_norm < 1.0e-9:
            direction = current[:2]
            direction_norm = float(np.linalg.norm(direction))
        if direction_norm < 1.0e-9:
            direction = np.asarray([1.0, 0.0])
            direction_norm = 1.0
        candidate[:2] = direction * (required_radial / direction_norm)
    return candidate - current


def camera_gaze_error(hand_point: Sequence[float]) -> np.ndarray:
    """Axis-angle rotation in optical coordinates that points +Z at the hand."""
    point = np.asarray(hand_point, dtype=float)
    if point.shape != (3,):
        raise ValueError("hand_point must be xyz")
    norm = float(np.linalg.norm(point))
    if norm < 1.0e-9:
        return np.zeros(3)
    direction = point / norm
    optical_axis = np.asarray([0.0, 0.0, 1.0])
    cross = np.cross(optical_axis, direction)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(direction[2], -1.0, 1.0))
    if sine < 1.0e-9:
        return np.zeros(3)
    return cross * (float(np.arctan2(sine, cosine)) / sine)


def camera_ray_alignment_error(
    hand_point: Sequence[float], desired_point: Sequence[float]
) -> np.ndarray:
    """Axis-angle camera motion that aligns measured and desired image rays."""
    measured = np.asarray(hand_point, dtype=float)
    desired = np.asarray(desired_point, dtype=float)
    if measured.shape != (3,) or desired.shape != (3,):
        raise ValueError("hand_point and desired_point must be xyz")
    measured_norm = float(np.linalg.norm(measured))
    desired_norm = float(np.linalg.norm(desired))
    if measured_norm < 1.0e-9 or desired_norm < 1.0e-9:
        return np.zeros(3)
    measured /= measured_norm
    desired /= desired_norm
    cross = np.cross(desired, measured)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(desired, measured), -1.0, 1.0))
    if sine < 1.0e-9:
        return np.zeros(3)
    return cross * (float(np.arctan2(sine, cosine)) / sine)


def vector_alignment_error(
    current_vector: Sequence[float], desired_vector: Sequence[float]
) -> np.ndarray:
    """Axis-angle world rotation that maps one direction onto another."""
    current = np.asarray(current_vector, dtype=float)
    desired = np.asarray(desired_vector, dtype=float)
    if current.shape != (3,) or desired.shape != (3,):
        raise ValueError("current and desired vectors must be xyz")
    current_norm = float(np.linalg.norm(current))
    desired_norm = float(np.linalg.norm(desired))
    if current_norm < 1.0e-9 or desired_norm < 1.0e-9:
        return np.zeros(3)
    current /= current_norm
    desired /= desired_norm
    cross = np.cross(current, desired)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(current, desired), -1.0, 1.0))
    if sine < 1.0e-9:
        return np.zeros(3)
    return cross * (float(np.arctan2(sine, cosine)) / sine)


def position_level_ik_step(
    position_jacobian: np.ndarray,
    angular_jacobian: np.ndarray,
    position_error: Sequence[float],
    current_tray_normal: Sequence[float],
    desired_tray_normal: Sequence[float] = (0.0, 0.0, 1.0),
    orientation_weight_m_per_rad: float = 0.12,
    orientation_gain: float = 0.8,
    damping: float = 0.02,
    joint_penalties: Sequence[float] = (1.0, 1.0, 1.0, 2.0),
) -> np.ndarray:
    """Coordinate J1-J4 to translate the TCP while keeping the tray level.

    This is one weighted task solve, so J2/J3 may lower the whole endpoint and
    J4 can compensate its pitch without stealing the image error. J5 is
    deliberately excluded: on the real arm it controls keyboard facing and
    must never be consumed as a J2 pitch-cancellation axis.
    """
    position_jacobian = np.asarray(position_jacobian, dtype=float)[:, :4]
    angular_jacobian = np.asarray(angular_jacobian, dtype=float)[:, :4]
    error = np.asarray(position_error, dtype=float)
    normal = np.asarray(current_tray_normal, dtype=float)
    desired = np.asarray(desired_tray_normal, dtype=float)
    penalties = np.asarray(joint_penalties, dtype=float)
    if position_jacobian.shape != (3, 4) or angular_jacobian.shape != (3, 4):
        raise ValueError("J1-J4 Jacobians must provide at least four columns")
    if (
        error.shape != (3,)
        or normal.shape != (3,)
        or desired.shape != (3,)
    ):
        raise ValueError("position and normal vectors must be xyz")
    if penalties.shape != (4,) or np.any(penalties <= 0.0):
        raise ValueError("four positive joint penalties are required")
    normal /= max(float(np.linalg.norm(normal)), 1.0e-9)
    desired /= max(float(np.linalg.norm(desired)), 1.0e-9)
    orientation_error = vector_alignment_error(normal, desired)
    limit = math.radians(20.0)
    orientation_norm = float(np.linalg.norm(orientation_error))
    if orientation_norm > limit:
        orientation_error *= limit / orientation_norm
    weight = max(0.0, float(orientation_weight_m_per_rad))
    inverse_penalties = np.diag(1.0 / penalties)
    damping_squared = max(float(damping), 1.0e-5) ** 2
    position_inverse = (
        inverse_penalties
        @ position_jacobian.T
        @ np.linalg.solve(
            position_jacobian
            @ inverse_penalties
            @ position_jacobian.T
            + damping_squared * np.eye(3),
            np.eye(3),
        )
    )
    position_delta = position_inverse @ error
    # Keep position strictly primary. Level correction is projected into the
    # remaining two-dimensional J1-J5 nullspace, which is exactly what lets
    # J4 compensate a J2/J3 endpoint descent without turning image error into
    # camera pitch.
    nullspace = np.eye(4) - position_inverse @ position_jacobian
    level_jacobian = weight * angular_jacobian @ nullspace
    remaining_level_error = weight * (
        float(orientation_gain) * orientation_error
        - angular_jacobian @ position_delta
    )
    if weight <= 0.0:
        return np.hstack((position_delta, 0.0))
    correction = (
        nullspace
        @ level_jacobian.T
        @ np.linalg.solve(
            level_jacobian @ level_jacobian.T
            + damping_squared * np.eye(3),
            remaining_level_error,
        )
    )
    return np.hstack((position_delta + correction, 0.0))


def single_joint_axis_alignment_step(
    current_axis: Sequence[float],
    desired_axis: Sequence[float],
    joint_angular_axis: Sequence[float],
    gain: float = 0.2,
    maximum_error_rad: float = math.radians(12.0),
) -> Tuple[float, float]:
    """Model-based signed J5 step for aligning the keyboard lateral axis.

    The sign comes from the URDF Jacobian rather than a hand-tuned image sign.
    The lateral axis is undirected, so the nearest equivalent orientation is
    selected and a mirrored camera cannot request a 180-degree wrist turn.
    Returns ``(joint_step_rad, remaining_axis_error_rad)``.
    """
    current = np.asarray(current_axis, dtype=float)
    desired = np.asarray(desired_axis, dtype=float)
    joint_axis = np.asarray(joint_angular_axis, dtype=float)
    if current.shape != (3,) or desired.shape != (3,) or joint_axis.shape != (3,):
        raise ValueError("alignment inputs must be xyz vectors")
    current_norm = float(np.linalg.norm(current))
    desired_norm = float(np.linalg.norm(desired))
    joint_norm = float(np.linalg.norm(joint_axis))
    if min(current_norm, desired_norm, joint_norm) < 1.0e-9:
        return 0.0, 0.0
    current /= current_norm
    desired /= desired_norm
    joint_axis /= joint_norm
    if float(np.dot(current, desired)) < 0.0:
        desired = -desired
    axis_error = vector_alignment_error(current, desired)
    error_norm = float(np.linalg.norm(axis_error))
    limit = max(0.0, float(maximum_error_rad))
    if error_norm > limit > 0.0:
        axis_error *= limit / error_norm
        error_norm = limit
    # Project the desired world angular correction onto J5's actual world axis.
    joint_step = float(gain) * float(np.dot(joint_axis, axis_error))
    return joint_step, error_norm


def physical_j4_vertical_search_step(
    vertical_angle_rad: float,
    gain: float,
    physical_sign: float,
    deadband_rad: float,
    maximum_step_rad: float,
) -> float:
    """Return the empirically calibrated J4 camera-pitch search correction."""
    angle = float(vertical_angle_rad)
    if abs(angle) <= max(0.0, float(deadband_rad)):
        return 0.0
    requested = float(physical_sign) * float(gain) * angle
    limit = max(0.0, float(maximum_step_rad))
    return float(np.clip(requested, -limit, limit))


def demonstrated_j5_layout_step(
    hand_line_angle_rad: float,
    gain: float,
    physical_sign: float,
    deadband_rad: float,
    maximum_step_rad: float,
) -> float:
    """Correct keyboard facing from the measured two-hand image line.

    The 2026-07-25 real PIPER-X teaching run established the physical sign:
    increasing J5 reduced a positive hand-line angle.  Therefore a positive
    image-line error requests a positive J5 step.  Keeping this calibration in
    image space avoids relying on an approximate camera mount transform.
    """
    error = float(hand_line_angle_rad)
    if abs(error) <= max(0.0, float(deadband_rad)):
        return 0.0
    requested = float(physical_sign) * float(gain) * error
    limit = max(0.0, float(maximum_step_rad))
    return float(np.clip(requested, -limit, limit))


def bounded_single_hand_j5_step(
    current_joint_rad: float,
    search_origin_rad: float,
    direction: float,
    requested_step_rad: float,
    search_limit_rad: float,
) -> Tuple[float, bool]:
    """Continue J5 in one physical direction without exceeding one search arc."""
    signed_direction = math.copysign(1.0, float(direction))
    searched = signed_direction * (
        float(current_joint_rad) - float(search_origin_rad)
    )
    remaining = max(0.0, float(search_limit_rad) - max(0.0, searched))
    if remaining <= 0.0:
        return 0.0, True
    step = signed_direction * min(
        max(0.0, float(requested_step_rad)), remaining
    )
    return step, False


def enforce_base_downward_trend(
    position_error_base: Sequence[float],
    active: bool,
    minimum_downward_step_m: float,
) -> np.ndarray:
    """Ensure an occluded low hand cannot make the TCP rise in base Z."""
    error = np.asarray(position_error_base, dtype=float).copy()
    if error.shape != (3,):
        raise ValueError("position_error_base must be xyz")
    if active:
        error[2] = min(
            float(error[2]),
            -abs(float(minimum_downward_step_m)),
        )
    return error


def keyboard_rotation_from_typing_plane(
    typing_rotation_base: np.ndarray,
    current_link_rotation: np.ndarray,
    maximum_tilt_rad: float,
) -> np.ndarray:
    """Map a two-hand typing plane to the fixed keyboard axes on link6.

    The physical tray convention is ``-link6 X`` = tray normal,
    ``link6 Y`` = keyboard left/right and ``link6 Z`` = keyboard forward.
    The normal is kept in the upward hemisphere and limited in world tilt so
    noisy depth can never command the keyboard to point downward.
    """
    typing = np.asarray(typing_rotation_base, dtype=float)
    current = np.asarray(current_link_rotation, dtype=float)
    if typing.shape != (3, 3) or current.shape != (3, 3):
        raise ValueError("typing and current rotations must be 3x3")
    up = np.asarray([0.0, 0.0, 1.0])
    normal = typing[:, 2].copy()
    normal /= np.linalg.norm(normal)
    if float(np.dot(normal, up)) < 0.0:
        normal = -normal
    cosine = float(np.clip(np.dot(normal, up), -1.0, 1.0))
    tilt = math.acos(cosine)
    maximum = max(0.0, float(maximum_tilt_rad))
    if tilt > maximum:
        horizontal = normal - up * cosine
        horizontal_norm = float(np.linalg.norm(horizontal))
        if horizontal_norm < 1.0e-9:
            normal = up.copy()
        else:
            horizontal /= horizontal_norm
            normal = math.cos(maximum) * up + math.sin(maximum) * horizontal
    lateral = typing[:, 0] - normal * float(np.dot(typing[:, 0], normal))
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm < 1.0e-6:
        lateral = current[:, 1] - normal * float(
            np.dot(current[:, 1], normal)
        )
        lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm < 1.0e-6:
        raise ValueError("typing plane lateral axis is degenerate")
    lateral /= lateral_norm
    if float(np.dot(lateral, current[:, 1])) < 0.0:
        lateral = -lateral
    link_x = -normal
    link_y = lateral
    link_z = np.cross(link_x, link_y)
    link_z /= np.linalg.norm(link_z)
    link_y = np.cross(link_z, link_x)
    link_y /= np.linalg.norm(link_y)
    return np.column_stack((link_x, link_y, link_z))


def staged_camera_error(
    depth_m: float,
    desired_depth_m: float,
    phase: str,
    midpoint_xy: Sequence[float] = (0.0, 0.0),
    desired_xy: Sequence[float] = (0.0, 0.0),
    xy_gain: float = 0.65,
) -> np.ndarray:
    """Distance-first camera-frame error used by the real hand servo."""
    xy = np.asarray(midpoint_xy, dtype=float)
    desired = np.asarray(desired_xy, dtype=float)
    if xy.shape != (2,) or desired.shape != (2,):
        raise ValueError("midpoint_xy and desired_xy must contain x/y")
    if phase == "distance_acquire":
        return np.asarray([0.0, 0.0, depth_m - desired_depth_m])
    if phase == "hand_align":
        return np.asarray(
            [
                xy_gain * (xy[0] - desired[0]),
                xy_gain * (xy[1] - desired[1]),
                depth_m - desired_depth_m,
            ]
        )
    raise ValueError("unknown servo phase")


def visual_search_command(
    hand_point: Sequence[float],
    visible_hand_count: int,
    both_hands_centered: bool,
    desired_depth_m: float,
    minimum_depth_m: float,
    maximum_depth_m: float,
    single_hand_retreat_m: float,
) -> Tuple[np.ndarray, str, str]:
    """Prioritized find-centre-approach command in camera coordinates."""
    point = np.asarray(hand_point, dtype=float)
    if point.shape != (3,) or point[2] <= 0.0:
        raise ValueError("hand_point must be valid camera xyz")
    depth = float(point[2])
    if depth < minimum_depth_m:
        return (
            np.asarray([0.0, 0.0, depth - desired_depth_m]),
            "distance_acquire",
            "distance_retreat",
        )
    if visible_hand_count < 2:
        return (
            np.asarray(
                [point[0], point[1], -max(0.0, single_hand_retreat_m)]
            ),
            "single_reacquire",
            "single_hand_search",
        )
    if not both_hands_centered:
        return (
            np.asarray([point[0], point[1], 0.0]),
            "visual_center",
            "center_both_hands",
        )
    action = (
        "centered_approach"
        if depth > maximum_depth_m
        else "centered_hold_range"
    )
    return (
        np.asarray([point[0], point[1], depth - desired_depth_m]),
        "depth_approach",
        action,
    )
