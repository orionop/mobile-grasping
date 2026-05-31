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

        x = rospy.get_param("~x", 0.5)
        y = rospy.get_param("~y", 0.0)
        z = rospy.get_param("~z", 0.1)
        qx = rospy.get_param("~qx", 0.0)
        qy = rospy.get_param("~qy", 0.0)
        qz = rospy.get_param("~qz", 0.0)
        qw = rospy.get_param("~qw", 1.0)
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.pose = PoseStamped()
        self.pose.header.frame_id = self.frame_id
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        self.pose.pose.orientation.x = qx
        self.pose.pose.orientation.y = qy
        self.pose.pose.orientation.z = qz
        self.pose.pose.orientation.w = qw

        self.pub = rospy.Publisher("/grasp_pose", PoseStamped, queue_size=1)
        rospy.loginfo(
            "Predictor node up: fixed pose (%.3f, %.3f, %.3f) in frame %s at %.0f Hz",
            x, y, z, self.frame_id, self.publish_rate,
        )

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.pose.header.stamp = rospy.Time.now()
            self.pub.publish(self.pose)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = PredictorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
