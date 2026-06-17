"""
omx_kinematics.py
=================

Forward kinematics and geometric Jacobian for the OpenManipulator-X (OMX-X)
as mounted on the TurtleBot3 Waffle, matching the ROBOTIS URDF:

  turtlebot3_manipulation_description (noetic):
    turtlebot3_manipulation_robot.urdf.xacro  -> base_link -> link1 mount
    open_manipulator_x.urdf.xacro             -> joint1..joint4, end_effector

Joint origins (parent->child, xyz; all rpy = 0):
  base mount  base_link -> link1 : (-0.092, 0,     0.091)   fixed
  joint1      link1     -> link2 : ( 0.012, 0,     0.017)   axis z
  joint2      link2     -> link3 : ( 0.0,   0,     0.058)   axis y
  joint3      link3     -> link4 : ( 0.024, 0,     0.128)   axis y
  joint4      link4     -> link5 : ( 0.124, 0,     0.0  )   axis y
  end_effector link5    -> ee    : ( 0.126, 0,     0.0  )   fixed

The TurtleBot3 base_footprint -> base_link adds a fixed +0.010 m in z, so
the mount expressed in the base_footprint frame (the frame /base_pose /odom
reports) is (-0.092, 0, 0.101). All transforms here are built in the
base_footprint frame; fk_world / jacobian_world then compose with the live
base pose.

Validated numerically against /gazebo/link_states (link5 world position
agrees to < 5 mm).
"""

from __future__ import annotations
import numpy as np

# Mount: base_footprint -> link1 (folds base_footprint->base_link +0.010 z).
_MOUNT_XYZ = np.array([-0.092, 0.0, 0.101])

# Joint origins (parent frame translation) and rotation axes.
_J1_XYZ = np.array([0.012, 0.0, 0.017]); _J1_AXIS = "z"
_J2_XYZ = np.array([0.0,   0.0, 0.058]); _J2_AXIS = "y"
_J3_XYZ = np.array([0.024, 0.0, 0.128]); _J3_AXIS = "y"
_J4_XYZ = np.array([0.124, 0.0, 0.0]);   _J4_AXIS = "y"
_EE_XYZ = np.array([0.126, 0.0, 0.0])

N_JOINTS = 4


def _trans(xyz: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = xyz
    return T


def _rot(axis: str, theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    if axis == "z":
        T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    elif axis == "y":
        T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    else:  # x
        T[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    return T


def _joint_frames(q: np.ndarray):
    """
    Returns (frames, axes_dirs, axis_origins, T_ee) all in base_footprint frame.

    frames[i]    : SE(3) frame at joint i AFTER its rotation (child link i+2)
    axis_dirs[i] : unit rotation axis of joint i in base_footprint frame
    axis_org[i]  : 3-vector origin of joint i axis in base_footprint frame
    T_ee         : SE(3) of the end-effector
    """
    specs = [
        (_J1_XYZ, _J1_AXIS, q[0]),
        (_J2_XYZ, _J2_AXIS, q[1]),
        (_J3_XYZ, _J3_AXIS, q[2]),
        (_J4_XYZ, _J4_AXIS, q[3]),
    ]
    axis_vec = {"x": np.array([1.0, 0, 0]),
                "y": np.array([0, 1.0, 0]),
                "z": np.array([0, 0, 1.0])}

    T = _trans(_MOUNT_XYZ)            # base_footprint -> link1
    axis_dirs, axis_org, frames = [], [], []
    for xyz, ax, theta in specs:
        # Frame at the joint axis (before applying the joint rotation).
        F = T @ _trans(xyz)
        axis_org.append(F[:3, 3].copy())
        axis_dirs.append(F[:3, :3] @ axis_vec[ax])
        # Apply joint rotation -> child link frame.
        T = F @ _rot(ax, theta)
        frames.append(T.copy())
    T_ee = T @ _trans(_EE_XYZ)
    return frames, axis_dirs, axis_org, T_ee


# ---------------------------------------------------------------------------
# Public API (base_footprint frame)
# ---------------------------------------------------------------------------

def fk(q: np.ndarray) -> np.ndarray:
    """Forward kinematics: base_footprint frame -> EE. Returns 4x4 SE(3)."""
    _, _, _, T_ee = _joint_frames(q)
    return T_ee


def link5_fk(q: np.ndarray) -> np.ndarray:
    """FK to link5 (wrist) in base_footprint frame; used for validation."""
    frames, _, _, _ = _joint_frames(q)
    return frames[-1]


def jacobian(q: np.ndarray) -> np.ndarray:
    """Geometric Jacobian (6 x 4) in base_footprint frame."""
    frames, axis_dirs, axis_org, T_ee = _joint_frames(q)
    p_ee = T_ee[:3, 3]
    J = np.zeros((6, N_JOINTS))
    for i in range(N_JOINTS):
        z = axis_dirs[i]
        J[:3, i] = np.cross(z, p_ee - axis_org[i])
        J[3:, i] = z
    return J


# ---------------------------------------------------------------------------
# World-frame variants (compose with live base pose)
# ---------------------------------------------------------------------------

def fk_world(q: np.ndarray, T_base: np.ndarray) -> np.ndarray:
    """FK in world frame given base_footprint SE(3) pose T_base."""
    return T_base @ fk(q)


def jacobian_world(q: np.ndarray, T_base: np.ndarray) -> np.ndarray:
    """Geometric Jacobian (6 x 4) rotated into the world frame."""
    R = T_base[:3, :3]
    J = jacobian(q)
    J_w = np.zeros_like(J)
    J_w[:3] = R @ J[:3]
    J_w[3:] = R @ J[3:]
    return J_w


def base_jacobian_world(
    theta_b: float,
    p_base: np.ndarray,
    p_ee: np.ndarray,
) -> np.ndarray:
    """
    Diff-drive base Jacobian in world frame (6 x 2).

    Virtual joints: col 0 = forward velocity v [m/s],
                    col 1 = angular velocity omega [rad/s].
    """
    J = np.zeros((6, 2))
    J[:3, 0] = [np.cos(theta_b), np.sin(theta_b), 0.0]
    r = p_ee - p_base
    J[:3, 1] = [-r[1], r[0], 0.0]
    J[3:, 1] = [0.0, 0.0, 1.0]
    return J
