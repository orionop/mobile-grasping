#!/usr/bin/env python3
"""
controller_node.py
==================

Reactive QP controller node. Subscribes to the predicted grasp pose
(world frame) and the live base pose estimate, and publishes arm joint
velocity commands at the configured control rate.

Topics
------
Subscribed:
  /grasp_pose      (geometry_msgs/PoseStamped)  -- target grasp pose
  /base_pose       (geometry_msgs/PoseStamped)  -- fused base pose
  /joint_states    (sensor_msgs/JointState)     -- current arm joint state

Published:
  <arm_cmd_topic>  (std_msgs/Float64MultiArray) -- joint velocity command
                   Default: /arm_controller/command
                   Check via: rosservice call /controller_manager/list_controllers

Parameters (from controller_params.yaml + inline)
--------------------------------------------------
  control_rate_hz  : float, target control loop rate (default 200)
  n_arm            : int, arm DOF (4 for OMX-X)
  n_base           : int, base virtual joints (2 for diff-drive)
  v_max_arm        : float, arm joint velocity bound, rad/s
  v_max_base       : float, base linear velocity bound, m/s
  slack_penalty    : float, slack penalty in the QP cost
  servoing_gain    : float, PBS gain (default 1.0)
  arm_cmd_topic    : str, topic to publish arm velocity command
  use_base_jacobian: bool, False=M1 (base stationary), True=M2 (base moving)
"""

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_matrix

from mobile_grasping_ros.qp_solver import HolisticQPSolver, HolisticQPConfig
import mobile_grasping_ros.omx_kinematics as omx

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


def _pose_to_T(pose) -> np.ndarray:
    """geometry_msgs/Pose -> 4x4 SE(3) numpy array."""
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    T = quaternion_matrix(q)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


def _yaw_from_T(T: np.ndarray) -> float:
    """Extract yaw angle from rotation matrix."""
    return np.arctan2(T[1, 0], T[0, 0])


class ControllerNode:
    def __init__(self):
        rospy.init_node("mobile_grasping_controller", anonymous=False)

        # Parameters
        self.control_rate = rospy.get_param("~control_rate_hz", 200.0)
        self.servoing_gain = rospy.get_param("~servoing_gain", 1.0)
        # M1: False (J_base = 0).  M2: True (real diff-drive J_base).
        self.use_base_jacobian = rospy.get_param("~use_base_jacobian", False)
        self.arm_cmd_topic = rospy.get_param(
            "~arm_cmd_topic", "/arm_controller/command"
        )

        n_arm = rospy.get_param("~n_arm", 4)
        n_base = rospy.get_param("~n_base", 2)
        v_max_arm = rospy.get_param("~v_max_arm", 1.5)
        v_max_base = rospy.get_param("~v_max_base", 0.26)
        slack_penalty = rospy.get_param("~slack_penalty", 1.0e6)

        self.cfg = HolisticQPConfig(
            n_arm=n_arm,
            n_base=n_base,
            v_max_arm=v_max_arm,
            v_max_base=v_max_base,
            slack_penalty=slack_penalty,
        )
        self.solver = HolisticQPSolver(self.cfg)

        # State buffers
        self.target_pose = None
        self.base_pose = None
        self.joint_state = None

        # Subscribers
        rospy.Subscriber("/grasp_pose", PoseStamped, self._on_grasp_pose)
        rospy.Subscriber("/base_pose", PoseStamped, self._on_base_pose)
        rospy.Subscriber("/joint_states", JointState, self._on_joint_state)

        # Publisher
        self.cmd_pub = rospy.Publisher(
            self.arm_cmd_topic, Float64MultiArray, queue_size=10
        )

        rospy.loginfo(
            "Controller node up: %d-DOF arm, %d-DOF base, %.0f Hz, "
            "cmd→%s, use_base_J=%s",
            n_arm, n_base, self.control_rate,
            self.arm_cmd_topic, self.use_base_jacobian,
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_grasp_pose(self, msg):
        self.target_pose = msg

    def _on_base_pose(self, msg):
        self.base_pose = msg

    def _on_joint_state(self, msg):
        self.joint_state = msg

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self._ready():
                cmd = self._compute_command()
                self.cmd_pub.publish(cmd)
            rate.sleep()

    def _ready(self):
        return all(x is not None for x in [
            self.target_pose, self.base_pose, self.joint_state
        ])

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _extract_arm_joints(self):
        name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
        try:
            return np.array([name_to_pos[n] for n in ARM_JOINT_NAMES])
        except KeyError:
            rospy.logwarn_throttle(
                5.0,
                "Arm joints %s not found in JointState. Got: %s",
                ARM_JOINT_NAMES, list(name_to_pos.keys()),
            )
            return None

    def _compute_desired_twist(
        self, T_ee: np.ndarray, T_target: np.ndarray
    ) -> np.ndarray:
        """
        Position-Based Servoing: v_desired = gain * [delta_p; delta_r].

        delta_r from SO3 log map of R_ee^T @ R_target.
        Returns 6D spatial velocity in world frame.
        """
        delta_p = T_target[:3, 3] - T_ee[:3, 3]

        R_err = T_ee[:3, :3].T @ T_target[:3, :3]
        cos_angle = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        if abs(angle) < 1e-6:
            delta_r = np.zeros(3)
        else:
            s = 2.0 * np.sin(angle)
            delta_r = (angle / s) * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ])

        return self.servoing_gain * np.concatenate([delta_p, delta_r])

    # ------------------------------------------------------------------
    # Main compute
    # ------------------------------------------------------------------
    def _compute_command(self) -> Float64MultiArray:
        q = self._extract_arm_joints()
        if q is None:
            msg = Float64MultiArray()
            msg.data = [0.0] * self.cfg.n_arm
            return msg

        T_base = _pose_to_T(self.base_pose.pose)
        T_ee = omx.fk_world(q, T_base)
        T_target = _pose_to_T(self.target_pose.pose)

        v_desired = self._compute_desired_twist(T_ee, T_target)
        J_arm = omx.jacobian_world(q, T_base)          # (6, 4)

        if self.use_base_jacobian:
            # M2: account for base motion in the Holistic Jacobian
            theta_b = _yaw_from_T(T_base)
            p_base = T_base[:3, 3]
            p_ee = T_ee[:3, 3]
            J_base = omx.base_jacobian_world(theta_b, p_base, p_ee)  # (6, 2)
        else:
            # M1: base stationary, no base contribution
            J_base = np.zeros((6, self.cfg.n_base))

        _, qdot_arm, slack = self.solver.solve(J_arm, J_base, v_desired)

        rospy.logdebug(
            "EE err pos=%.4f m  |slack|=%.2e",
            np.linalg.norm(T_target[:3, 3] - T_ee[:3, 3]),
            np.linalg.norm(slack),
        )

        msg = Float64MultiArray()
        msg.data = qdot_arm.tolist()
        return msg


if __name__ == "__main__":
    try:
        node = ControllerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
