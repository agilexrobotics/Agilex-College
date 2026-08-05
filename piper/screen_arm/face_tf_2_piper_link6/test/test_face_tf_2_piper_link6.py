import math

from face_tf_2_piper_link6.face_tf_2_piper_link6_node import (
    compute_arm_pose,
    euler_xyz_to_quaternion,
    rotate_vector_by_quaternion,
)


def quaternion_to_euler_xyz(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def test_rotate_vector_identity_quaternion():
    assert rotate_vector_by_quaternion((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)) == (
        1.0,
        2.0,
        3.0,
    )


def test_euler_xyz_to_quaternion_round_trip():
    roll, pitch, yaw = 0.306, 1.403, 0.280
    quaternion = euler_xyz_to_quaternion(roll, pitch, yaw)
    actual = quaternion_to_euler_xyz(*quaternion)
    assert math.isclose(actual[0], roll, abs_tol=1e-12)
    assert math.isclose(actual[1], pitch, abs_tol=1e-12)
    assert math.isclose(actual[2], yaw, abs_tol=1e-12)


def test_default_sample_computes_arm_pose_near_link6_translation():
    translation, quaternion = compute_arm_pose(
        (0.800014, -0.012814, 0.402130),
        (0.685, 0.251, 0.495, 0.473),
        (-0.522711, 0.009250, -0.124325),
        (0.306, 1.403, 0.280),
    )

    assert math.isclose(translation[0], 0.277303, abs_tol=1e-6)
    assert math.isclose(translation[1], -0.003564, abs_tol=1e-6)
    assert math.isclose(translation[2], 0.277805, abs_tol=1e-6)

    actual_rpy = quaternion_to_euler_xyz(*quaternion)
    assert math.isclose(actual_rpy[0], 0.306, abs_tol=1e-12)
    assert math.isclose(actual_rpy[1], 1.403, abs_tol=1e-12)
    assert math.isclose(actual_rpy[2], 0.280, abs_tol=1e-12)
