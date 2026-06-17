#!/usr/bin/env python3
"""
run_m2_trial.py
===============

Deterministic M2 trial driver. Removes all manual cmd_vel timing.

Sequence:
  1. wait `settle` seconds for the arm to converge on the fixed target
  2. drive the base forward at `v` m/s for `drive_time` seconds
     (kept inside the OMX-X reach window so the target stays graspable)
  3. send a zero twist so the base stops cleanly
  4. report max / mean / final EE position error DURING the drive window

Run the sim first (sim.launch ... use_base_jacobian:=true|false), then:
  rosrun mobile_grasping_ros run_m2_trial.py _v:=0.05 _drive_time:=5.0

Compare the printed max error between a closed-loop run
(use_base_jacobian:=true) and an open-loop run (false).

Params:
  v          : base forward speed [m/s]   (default 0.05)
  drive_time : seconds to drive           (default 5.0  -> 0.25 m at 0.05)
  settle     : seconds to settle first    (default 6.0)
  pub_rate   : cmd_vel publish rate [Hz]  (default 20)
"""

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class M2Trial:
    def __init__(self):
        rospy.init_node("m2_trial", anonymous=True)
        self.v = float(rospy.get_param("~v", 0.05))
        self.drive_time = float(rospy.get_param("~drive_time", 5.0))
        self.settle = float(rospy.get_param("~settle", 6.0))
        self.pub_rate = float(rospy.get_param("~pub_rate", 20.0))

        self.err = None
        rospy.Subscriber("/ee_pos_error", Float64, self._on_err)
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.sleep(1.0)  # let connections establish

    def _on_err(self, msg):
        self.err = msg.data

    def _drive(self, vx):
        t = Twist()
        t.linear.x = vx
        self.cmd.publish(t)

    def run(self):
        rospy.loginfo("M2 trial: settle %.1fs, then drive %.2f m/s for %.1fs "
                      "(%.2f m).", self.settle, self.v, self.drive_time,
                      self.v * self.drive_time)

        # --- settle ---
        rospy.sleep(self.settle)
        settle_err = self.err
        rospy.loginfo("Settled. EE error before motion: %s m",
                      None if settle_err is None else round(settle_err, 4))

        # --- drive, collecting error during the window ---
        samples = []
        rate = rospy.Rate(self.pub_rate)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - t0) < self.drive_time:
            self._drive(self.v)
            if self.err is not None:
                samples.append(self.err)
            rate.sleep()

        # --- stop the base cleanly ---
        for _ in range(10):
            self._drive(0.0)
            rate.sleep()

        if not samples:
            rospy.logerr("No /ee_pos_error samples. Is the controller running?")
            return

        s = np.array(samples)
        rospy.loginfo("=" * 50)
        rospy.loginfo("M2 RESULT over %.1fs drive window (%d samples):",
                      self.drive_time, len(s))
        rospy.loginfo("  max  EE error : %.4f m  (%.2f cm)", s.max(), s.max() * 100)
        rospy.loginfo("  mean EE error : %.4f m  (%.2f cm)", s.mean(), s.mean() * 100)
        rospy.loginfo("  final EE error: %.4f m  (%.2f cm)", s[-1], s[-1] * 100)
        rospy.loginfo("=" * 50)


if __name__ == "__main__":
    try:
        M2Trial().run()
    except rospy.ROSInterruptException:
        pass
