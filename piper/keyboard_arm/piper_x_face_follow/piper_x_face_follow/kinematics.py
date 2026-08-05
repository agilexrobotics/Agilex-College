"""Small URDF serial-chain kinematics helper used by the safe hand servo."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text: str, default: str) -> np.ndarray:
    return np.asarray([float(value) for value in (text or default).split()], dtype=float)


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _axis_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


@dataclass(frozen=True)
class Joint:
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class SerialChain:
    """Forward pose and geometric Jacobian for joint1..joint6."""

    def __init__(self, joints: Sequence[Joint]):
        if len(joints) != 6:
            raise ValueError("PIPER-X chain must contain exactly six joints")
        self.joints = tuple(joints)

    @classmethod
    def from_urdf(cls, robot_description: str) -> "SerialChain":
        root = ET.fromstring(robot_description)
        by_name = {element.attrib.get("name"): element for element in root.findall("joint")}
        joints = []
        for name in (f"joint{index}" for index in range(1, 7)):
            element = by_name.get(name)
            if element is None or element.attrib.get("type") != "revolute":
                raise ValueError(f"missing revolute {name} in robot_description")
            origin = element.find("origin")
            axis = element.find("axis")
            limit = element.find("limit")
            joints.append(
                Joint(
                    xyz=_vector(origin.attrib.get("xyz") if origin is not None else "", "0 0 0"),
                    rpy=_vector(origin.attrib.get("rpy") if origin is not None else "", "0 0 0"),
                    axis=_vector(axis.attrib.get("xyz") if axis is not None else "", "0 0 1"),
                    lower=float(limit.attrib["lower"]),
                    upper=float(limit.attrib["upper"]),
                )
            )
        return cls(joints)

    def pose_and_jacobian(
        self, positions: Sequence[float], active_joints: int = 5
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if len(positions) != 6:
            raise ValueError("six joint positions required")
        transform = np.eye(4)
        axes = []
        origins = []
        for joint, angle in zip(self.joints, positions):
            origin = np.eye(4)
            origin[:3, :3] = _rpy_matrix(joint.rpy)
            origin[:3, 3] = joint.xyz
            transform = transform @ origin
            axes.append(transform[:3, :3] @ joint.axis)
            origins.append(transform[:3, 3].copy())
            rotation = np.eye(4)
            rotation[:3, :3] = _axis_matrix(joint.axis, float(angle))
            transform = transform @ rotation
        end_position = transform[:3, 3].copy()
        jacobian = np.zeros((6, active_joints))
        for index in range(active_joints):
            jacobian[:3, index] = np.cross(axes[index], end_position - origins[index])
            jacobian[3:, index] = axes[index]
        return end_position, transform[:3, :3].copy(), jacobian

    def offset_point_and_jacobian(
        self,
        positions: Sequence[float],
        offset: Sequence[float],
        active_joints: int = 5,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Position/Jacobian for a rigid point offset from the joint-6 origin.

        ``offset`` is expressed in link6 coordinates.  This is the quantity
        needed by the eye-in-hand controller because the camera optical
        assembly is mounted above J6 instead of at the URDF link6 origin.
        """
        position, rotation, geometric = self.pose_and_jacobian(
            positions, active_joints
        )
        offset_base = rotation @ np.asarray(offset, dtype=float)
        point_position = position + offset_base
        point_jacobian = geometric[:3, :].copy()
        for index in range(active_joints):
            point_jacobian[:, index] += np.cross(
                geometric[3:, index], offset_base
            )
        return point_position, point_jacobian
