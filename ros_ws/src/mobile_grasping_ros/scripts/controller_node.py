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
  /arm_velocity_cmd (std_msgs/Float64MultiArray) -- joint velocity command

Parameters (from controller_params.yaml)
----------------------------------------
  control_rate_hz  : float, target control loop rate (default 200)
  n_arm            : int, arm DOF (4 for OMX-X, 7 for Panda)
  n_base           : int, base virtual joints (2 for diff-drive)
  v_max_arm        : float, arm joint velocity bound, rad/s
  v_max_base       : float, base linear velocity bound, m/s
  slack_penalty    : float, slack penalty in the QP cost
"""

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mobile_grasping_ros.qp_solver import HolisticQPSolver, HolisticQPConfig


class ControllerNode:
    def __init__(self):
        rospy.init_node("mobile_grasping_controller", anonymous=False)

        # Parameters
        self.control_rate = rospy.get_param("~control_rate_hz", 200.0)
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
        self.target_pose = None         # geometry_msgs.PoseStamped
        self.base_pose = None           # geometry_msgs.PoseStamped
        self.joint_state = None         # sensor_msgs.JointState

        # Subscribers
        rospy.Subscriber("/grasp_pose", PoseStamped, self._on_grasp_pose)
        rospy.Subscriber("/base_pose", PoseStamped, self._on_base_pose)
        rospy.Subscriber("/joint_states", JointState, self._on_joint_state)

        # Publisher
        self.cmd_pub = rospy.Publisher(
            "/arm_velocity_cmd", Float64MultiArray, queue_size=10
        )

        rospy.loginfo(
            "Controller node up: %d-DOF arm, %d-DOF base, %.0f Hz target rate.",
            n_arm, n_base, self.control_rate,
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

    def _compute_command(self):
        # PLACEHOLDER: this is where the full pipeline plugs in.
        # The structural code is:
        #   1) compute current end-effector pose from joint state and base pose
        #   2) compute desired end-effector twist via Position-Based Servoing:
        #        v_desired = beta * log_se3( T_ee^-1 @ T_target )
        #   3) build J_arm at current q via a kinematics module (to be added)
        #   4) build J_base from the diff-drive virtual-joint mapping
        #   5) call self.solver.solve(...) and publish qdot_arm
        # For now we publish zeros so the topic exists and downstream nodes
        # can be wired up.
        msg = Float64MultiArray()
        msg.data = [0.0] * self.cfg.n_arm
        return msg


if __name__ == "__main__":
    try:
        node = ControllerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
