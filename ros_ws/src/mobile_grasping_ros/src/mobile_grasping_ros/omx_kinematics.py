"""
omx_kinematics.py
=================

Forward kinematics and geometric Jacobian for the OpenManipulator-X (OMX-X),
4-DOF revolute arm by ROBOTIS.

DH parameters follow the Craig (standard) convention, where row i describes
the transform from frame i-1 to frame i:

    T_{i-1,i} parametrised by (a_{i-1}, alpha_{i-1}, d_i, theta_i)

Link lengths from the ROBOTIS OMX-X technical specification:
  L1 = 0.077 m  (base to joint2 along z)
  L2 = 0.130 m  (joint2 to joint3 along x)
  L3 = 0.124 m  (joint3 to joint4 along x)
  L4 = 0.126 m  (joint4 to EE centre along x)

If the arm moves in unexpected directions after first bring-up, verify
that q=[0,0,0,0] corresponds to the arm pointing straight up
(joint2 axis horizontal, all subsequent links aligned). Theta offsets
can be added to the _DH table as the fourth element if needed.
"""

from __future__ import annotations
import numpy as np

# (a_prev [m], alpha_prev [rad], d [m], theta_offset [rad])
_DH = [
    (0.000,       0.0,    0.077, 0.0),   # joint1 — base rotation about z
    (0.000,  np.pi/2,    0.000, 0.0),   # joint2 — shoulder, alpha rotates z->y
    (0.130,       0.0,    0.000, 0.0),   # joint3 — elbow
    (0.124,       0.0,    0.000, 0.0),   # joint4 — wrist
]
_EE_OFFSET = 0.126   # metres along local x4 to gripper centre
N_JOINTS = 4


def _dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Standard DH (Craig convention) 4x4 homogeneous transform."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,      -st,      0.0,   a    ],
        [st * ca,  ct * ca, -sa,  -sa*d ],
        [st * sa,  ct * sa,  ca,   ca*d ],
        [0.0,      0.0,      0.0,  1.0  ],
    ])


def _frames(q: np.ndarray) -> list:
    """
    Return [T0, T1, T2, T3, T4] — transforms from arm base to each joint frame.
    T0 = eye(4) (arm base frame itself).
    """
    T_list = [np.eye(4)]
    for i, (a, alpha, d, t_off) in enumerate(_DH):
        T_list.append(T_list[-1] @ _dh(a, alpha, d, q[i] + t_off))
    return T_list


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def fk(q: np.ndarray) -> np.ndarray:
    """
    Forward kinematics: arm-base frame -> EE centre.

    Parameters
    ----------
    q : (4,) joint angles [rad]

    Returns
    -------
    T_ee : (4, 4) SE(3)
    """
    T_list = _frames(q)
    T_ee = np.eye(4)
    T_ee[0, 3] = _EE_OFFSET
    return T_list[-1] @ T_ee


def jacobian(q: np.ndarray) -> np.ndarray:
    """
    Geometric Jacobian in arm-base frame (6 x 4).
    Rows 0:3 = linear velocity part, rows 3:6 = angular velocity part.
    """
    T_list = _frames(q)
    T_ee_xfm = np.eye(4)
    T_ee_xfm[0, 3] = _EE_OFFSET
    p_ee = (T_list[-1] @ T_ee_xfm)[:3, 3]

    J = np.zeros((6, N_JOINTS))
    for i in range(N_JOINTS):
        # Modified (Craig) DH: joint i rotates about z of frame i+1.
        z = T_list[i + 1][:3, 2]       # joint axis in arm-base frame
        p = T_list[i + 1][:3, 3]       # frame origin in arm-base frame
        J[:3, i] = np.cross(z, p_ee - p)
        J[3:, i] = z
    return J


def fk_world(q: np.ndarray, T_base: np.ndarray) -> np.ndarray:
    """FK in world frame given current base SE(3) pose."""
    return T_base @ fk(q)


def jacobian_world(q: np.ndarray, T_base: np.ndarray) -> np.ndarray:
    """
    Geometric Jacobian in world frame (6 x 4).
    Rotates the arm-base-frame Jacobian by the base rotation R.
    """
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

    Parameters
    ----------
    theta_b : float   base heading [rad] in world frame
    p_base  : (3,)    base origin in world frame [m]
    p_ee    : (3,)    EE position in world frame [m]
    """
    J = np.zeros((6, 2))
    # col 0: EE translates with base at heading theta_b (no rotation)
    J[:3, 0] = [np.cos(theta_b), np.sin(theta_b), 0.0]
    # col 1: rotation about world z at p_base
    r = p_ee - p_base
    J[:3, 1] = [-r[1], r[0], 0.0]   # cross([0,0,1], r)
    J[3:, 1] = [0.0, 0.0, 1.0]
    return J
