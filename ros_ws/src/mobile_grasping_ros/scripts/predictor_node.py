#!/usr/bin/env python3
"""
predictor_node.py
=================

Placeholder grasp predictor node. Publishes a fixed grasp pose at a
configurable rate so the controller node has a target to track during
bring-up. This will later be replaced with an actual predictor (FCN
reimplementation, GG-CNN, or OpenVLA-OFT) that consumes a heightmap
and outputs the predicted grasp pose.

Topics
------
Published:
  /grasp_pose      (geometry_msgs/PoseStamped) -- target grasp pose

Parameters
----------
  publish_rate_hz : float, how often to republish the fixed pose
  x, y, z         : float, grasp position in world frame
  qx, qy, qz, qw  : float, grasp orientation as quaternion (default identity)
  frame_id        : str, frame to publish in (default "world")
"""

import rospy
from geometry_msgs.msg import PoseStamped


class PredictorNode:
    def __init__(self):
        rospy.init_node("mobile_grasping_predictor", anonymous=False)

        self.publish_rate = rospy.get_param("~publish_rate_hz", 30.0)

        self.x0 = rospy.get_param("~x", 0.5)
        self.y0 = rospy.get_param("~y", 0.0)
        self.z0 = rospy.get_param("~z", 0.1)
        qx = rospy.get_param("~qx", 0.0)
        qy = rospy.get_param("~qy", 0.0)
        qz = rospy.get_param("~qz", 0.0)
        qw = rospy.get_param("~qw", 1.0)
        self.frame_id = rospy.get_param("~frame_id", "world")

        # Optional moving target: ramps position at (vx, vy, vz) m/s for
        # move_dur seconds after move_delay. Used to isolate the arm's
        # velocity-tracking ability (move the target, base stationary).
        # All zero -> fixed pose (default).
        self.vx = float(rospy.get_param("~vx", 0.0))
        self.vy = float(rospy.get_param("~vy", 0.0))
        self.vz = float(rospy.get_param("~vz", 0.0))
        self.move_delay = float(rospy.get_param("~move_delay", 6.0))
        self.move_dur = float(rospy.get_param("~move_dur", 5.0))

        self.pose = PoseStamped()
        self.pose.header.frame_id = self.frame_id
        self.pose.pose.orientation.x = qx
        self.pose.pose.orientation.y = qy
        self.pose.pose.orientation.z = qz
        self.pose.pose.orientation.w = qw

        self.pub = rospy.Publisher("/grasp_pose", PoseStamped, queue_size=1)
        rospy.loginfo(
            "Predictor node up: start (%.3f, %.3f, %.3f) frame %s at %.0f Hz; "
            "vel (%.3f, %.3f, %.3f) after %.1fs for %.1fs",
            self.x0, self.y0, self.z0, self.frame_id, self.publish_rate,
            self.vx, self.vy, self.vz, self.move_delay, self.move_dur,
        )

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        t_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t_start
            ramp = max(0.0, min(t - self.move_delay, self.move_dur))
            self.pose.pose.position.x = self.x0 + self.vx * ramp
            self.pose.pose.position.y = self.y0 + self.vy * ramp
            self.pose.pose.position.z = self.z0 + self.vz * ramp
            self.pose.header.stamp = rospy.Time.now()
            self.pub.publish(self.pose)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = PredictorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
