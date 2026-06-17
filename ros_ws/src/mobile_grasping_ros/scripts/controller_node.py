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

Published (depends on cmd_interface):
  trajectory mode (default, for position_controllers/JointTrajectoryController):
    /arm_controller/command (trajectory_msgs/JointTrajectory)
  velocity mode (for velocity_controllers/* or hardware velocity interface):
    <arm_cmd_topic> (std_msgs/Float64MultiArray)

The stock ROBOTIS TB3+OMX-X Gazebo bring-up loads arm_controller as a
position_controllers/JointTrajectoryController on a PositionJointInterface,
so trajectory mode is the default: the QP joint *velocities* are integrated
to a position setpoint (q_cmd = q + qdot*cmd_lookahead) and streamed as a
one-point JointTrajectory each cycle.

Parameters (from controller_params.yaml + inline)
--------------------------------------------------
  control_rate_hz  : float, target control loop rate (default 200)
  n_arm            : int, arm DOF (4 for OMX-X)
  n_base           : int, base virtual joints (2 for diff-drive)
  v_max_arm        : float, arm joint velocity bound, rad/s
  v_max_base       : float, base linear velocity bound, m/s
  slack_penalty    : float, slack penalty in the QP cost
  servoing_gain    : float, PBS gain (default 1.0)
  cmd_interface    : str, "trajectory" (default) or "velocity"
  cmd_lookahead    : float, integration horizon for trajectory mode, s (default 0.1)
  arm_cmd_topic    : str, command topic (default /arm_controller/command)
  use_base_jacobian: bool, False=M1 (base stationary), True=M2 (base moving)
"""

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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

        # Parameters. Coerce numeric types explicitly: PyYAML resolves
        # unsigned-exponent floats like "1.0e6" as strings, so rosparam can
        # hand back str for slack_penalty etc. float()/int() makes it robust.
        self.control_rate = float(rospy.get_param("~control_rate_hz", 200.0))
        self.servoing_gain = float(rospy.get_param("~servoing_gain", 0.5))
        # M1: False (J_base = 0).  M2: True (real diff-drive J_base).
        self.use_base_jacobian = bool(rospy.get_param("~use_base_jacobian", False))
        # "trajectory" (position JointTrajectoryController) or "velocity".
        self.cmd_interface = rospy.get_param("~cmd_interface", "trajectory")
        self.cmd_lookahead = float(rospy.get_param("~cmd_lookahead", 0.08))
        # Position JointTrajectoryControllers can't track goals restreamed at
        # 200 Hz (each preempt resets the interpolation before it accelerates,
        # giving jitter + near-zero net motion). Keep the QP at control_rate
        # but throttle trajectory publishing to this rate.
        self.traj_pub_rate = float(rospy.get_param("~traj_pub_rate", 25.0))
        self._last_pub_t = 0.0
        self.arm_cmd_topic = rospy.get_param(
            "~arm_cmd_topic", "/arm_controller/command"
        )

        n_arm = int(rospy.get_param("~n_arm", 4))
        n_base = int(rospy.get_param("~n_base", 2))
        v_max_arm = float(rospy.get_param("~v_max_arm", 1.5))
        v_max_base = float(rospy.get_param("~v_max_base", 0.26))
        slack_penalty = float(rospy.get_param("~slack_penalty", 1.0e6))
        # Task-space relaxation: hard position, soft orientation. A 4-DOF arm
        # cannot satisfy a full 6-D twist; relaxing the 3 orientation axes
        # stops the QP from saturating joints and chattering.
        slack_weights = [float(w) for w in rospy.get_param(
            "~slack_weights", [1.0e4, 1.0e4, 1.0e4, 1.0e1, 1.0e1, 1.0e1]
        )]

        self.cfg = HolisticQPConfig(
            n_arm=n_arm,
            n_base=n_base,
            v_max_arm=v_max_arm,
            v_max_base=v_max_base,
            slack_penalty=slack_penalty,
            slack_weights=tuple(slack_weights),
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

        # Publisher (message type depends on cmd_interface)
        if self.cmd_interface == "trajectory":
            self.cmd_pub = rospy.Publisher(
                self.arm_cmd_topic, JointTrajectory, queue_size=1
            )
        else:
            self.cmd_pub = rospy.Publisher(
                self.arm_cmd_topic, Float64MultiArray, queue_size=10
            )

        # Diagnostics for live tracking (rqt_plot) and debugging.
        self.ee_pose_pub = rospy.Publisher("/ee_pose", PoseStamped, queue_size=1)
        self.ee_err_pub = rospy.Publisher("/ee_pos_error", Float64, queue_size=1)

        rospy.loginfo(
            "Controller node up: %d-DOF arm, %d-DOF base, %.0f Hz, "
            "cmd→%s (%s), use_base_J=%s",
            n_arm, n_base, self.control_rate,
            self.arm_cmd_topic, self.cmd_interface, self.use_base_jacobian,
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_grasp_pose(self, msg):
        self.target_pose = msg

    def _on_base_pose(self, msg):
        self.base_pose = msg

    def _on_joint_state(self, msg):
        # Gazebo publishes TWO /joint_states streams: the DiffDrive plugin
        # (wheels only, high rate) and the arm joint_state_controller. Only
        # keep messages that actually contain the arm joints, else the buffer
        # is constantly clobbered by wheel-only frames -> stale q -> jitter.
        if all(j in msg.name for j in ARM_JOINT_NAMES):
            self.joint_state = msg

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self._ready():
                cmd = self._compute_command()
                if cmd is not None and self._should_publish():
                    self.cmd_pub.publish(cmd)
                    self._last_pub_t = rospy.Time.now().to_sec()
            rate.sleep()

    def _should_publish(self):
        """Throttle trajectory streaming; velocity mode publishes every cycle."""
        if self.cmd_interface != "trajectory":
            return True
        return (rospy.Time.now().to_sec() - self._last_pub_t) >= (
            1.0 / self.traj_pub_rate
        )

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

    def _build_msg(self, qdot_arm: np.ndarray, q: np.ndarray):
        """Pack QP velocity output into the configured message type."""
        if self.cmd_interface == "trajectory":
            # Integrate velocity to a position setpoint and stream a
            # single-point JointTrajectory for the position controller.
            q_cmd = q + qdot_arm * self.cmd_lookahead
            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = ARM_JOINT_NAMES
            pt = JointTrajectoryPoint()
            pt.positions = q_cmd.tolist()
            # No velocity endpoint: commanding the (saturated, sign-flipping)
            # qdot as an arrival velocity makes the controller overshoot and
            # vibrate. Position-only waypoints come to rest smoothly.
            pt.time_from_start = rospy.Duration(self.cmd_lookahead)
            traj.points = [pt]
            return traj
        msg = Float64MultiArray()
        msg.data = qdot_arm.tolist()
        return msg

    # ------------------------------------------------------------------
    # Main compute
    # ------------------------------------------------------------------
    def _compute_command(self):
        q = self._extract_arm_joints()
        if q is None:
            return None   # don't command a setpoint we can't anchor to current q

        T_base = _pose_to_T(self.base_pose.pose)
        T_ee = omx.fk_world(q, T_base)
        T_target = _pose_to_T(self.target_pose.pose)

        v_desired = self._compute_desired_twist(T_ee, T_target)

        # Live diagnostics: computed EE world pose + position error.
        ee_ps = PoseStamped()
        ee_ps.header.stamp = rospy.Time.now()
        ee_ps.header.frame_id = "world"
        ee_ps.pose.position.x = T_ee[0, 3]
        ee_ps.pose.position.y = T_ee[1, 3]
        ee_ps.pose.position.z = T_ee[2, 3]
        self.ee_pose_pub.publish(ee_ps)
        self.ee_err_pub.publish(
            Float64(float(np.linalg.norm(T_target[:3, 3] - T_ee[:3, 3])))
        )

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

        return self._build_msg(qdot_arm, q)


if __name__ == "__main__":
    try:
        node = ControllerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
