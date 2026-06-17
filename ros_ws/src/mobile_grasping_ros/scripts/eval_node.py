#!/usr/bin/env python3
"""
eval_node.py
============

Tracking-error logger for M1/M2 trials. Independently recomputes the
end-effector pose from joint state + base pose (does NOT trust the
controller's own number) and logs world-frame tracking error to a CSV.

Produces the raw data for the Exp 1 figure: EE tracking error vs time.
Pair with experiments/plot_tracking_error.py to render the PNG.

Topics
------
Subscribed:
  /grasp_pose      (geometry_msgs/PoseStamped)  -- target grasp pose
  /base_pose       (geometry_msgs/PoseStamped)  -- fused base pose
  /joint_states    (sensor_msgs/JointState)     -- current arm joint state

Output
------
CSV at ~out_csv, one row per sample:
  t, ee_x, ee_y, ee_z, tgt_x, tgt_y, tgt_z, pos_err, theta_err_deg,
  base_x, base_y, base_yaw_deg

Parameters
----------
  out_csv     : str, output path (default /tmp/tracking_error.csv)
  log_rate_hz : float, sampling rate (default 50)
"""

import csv
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix

import mobile_grasping_ros.omx_kinematics as omx

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


def _pose_to_T(pose):
    q = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    T = quaternion_matrix(q)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


def _yaw(T):
    return np.arctan2(T[1, 0], T[0, 0])


class EvalNode:
    def __init__(self):
        rospy.init_node("mobile_grasping_eval", anonymous=False)

        self.out_csv = rospy.get_param("~out_csv", "/tmp/tracking_error.csv")
        self.log_rate = rospy.get_param("~log_rate_hz", 50.0)

        self.target_pose = None
        self.base_pose = None
        self.joint_state = None
        self.t0 = None

        rospy.Subscriber("/grasp_pose", PoseStamped, self._on_grasp)
        rospy.Subscriber("/base_pose", PoseStamped, self._on_base)
        rospy.Subscriber("/joint_states", JointState, self._on_joints)

        self._f = open(self.out_csv, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow([
            "t", "ee_x", "ee_y", "ee_z",
            "tgt_x", "tgt_y", "tgt_z",
            "pos_err", "theta_err_deg",
            "base_x", "base_y", "base_yaw_deg",
        ])
        rospy.on_shutdown(self._close)

        rospy.loginfo("Eval node up: logging to %s at %.0f Hz",
                      self.out_csv, self.log_rate)

    def _on_grasp(self, msg):
        self.target_pose = msg

    def _on_base(self, msg):
        self.base_pose = msg

    def _on_joints(self, msg):
        self.joint_state = msg

    def _arm_q(self):
        name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
        try:
            return np.array([name_to_pos[n] for n in ARM_JOINT_NAMES])
        except KeyError:
            return None

    def spin(self):
        rate = rospy.Rate(self.log_rate)
        while not rospy.is_shutdown():
            if all(x is not None for x in
                   [self.target_pose, self.base_pose, self.joint_state]):
                self._log_row()
            rate.sleep()

    def _log_row(self):
        q = self._arm_q()
        if q is None:
            return

        now = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        T_base = _pose_to_T(self.base_pose.pose)
        T_ee = omx.fk_world(q, T_base)
        T_tgt = _pose_to_T(self.target_pose.pose)

        ee = T_ee[:3, 3]
        tgt = T_tgt[:3, 3]
        pos_err = float(np.linalg.norm(tgt - ee))

        R_err = T_ee[:3, :3].T @ T_tgt[:3, :3]
        cos_a = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        theta_err_deg = float(np.degrees(np.arccos(cos_a)))

        base = T_base[:3, 3]
        base_yaw_deg = float(np.degrees(_yaw(T_base)))

        self._w.writerow([
            f"{t:.4f}",
            f"{ee[0]:.5f}", f"{ee[1]:.5f}", f"{ee[2]:.5f}",
            f"{tgt[0]:.5f}", f"{tgt[1]:.5f}", f"{tgt[2]:.5f}",
            f"{pos_err:.5f}", f"{theta_err_deg:.3f}",
            f"{base[0]:.5f}", f"{base[1]:.5f}", f"{base_yaw_deg:.3f}",
        ])

    def _close(self):
        try:
            self._f.close()
            rospy.loginfo("Eval CSV written: %s", self.out_csv)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        node = EvalNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
