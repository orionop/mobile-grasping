#!/usr/bin/env python3
"""
estimator_node.py
=================

Live base pose estimator. Subscribes to wheel odometry (and later IMU
and visual odometry) and publishes the fused base pose at the
controller's expected rate. For initial bring-up the node simply
republishes the /odom message as /base_pose; the EKF fusion with IMU
and RGB-D VO is layered in later.

Topics
------
Subscribed:
  /odom            (nav_msgs/Odometry) -- wheel odometry from TB3

Published:
  /base_pose       (geometry_msgs/PoseStamped) -- fused base pose

Parameters
----------
  publish_rate_hz : float, output rate (default 50)
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class EstimatorNode:
    def __init__(self):
        rospy.init_node("mobile_grasping_estimator", anonymous=False)

        self.publish_rate = rospy.get_param("~publish_rate_hz", 50.0)
        self.last_odom = None

        rospy.Subscriber("/odom", Odometry, self._on_odom)
        self.pub = rospy.Publisher("/base_pose", PoseStamped, queue_size=1)

        rospy.loginfo(
            "Estimator node up: republishing /odom as /base_pose at %.0f Hz "
            "(EKF fusion with IMU + VO to be added)", self.publish_rate
        )

    def _on_odom(self, msg):
        self.last_odom = msg

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.last_odom is not None:
                ps = PoseStamped()
                ps.header = self.last_odom.header
                ps.pose = self.last_odom.pose.pose
                self.pub.publish(ps)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = EstimatorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
