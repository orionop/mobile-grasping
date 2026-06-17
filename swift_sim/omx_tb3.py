"""
omx_tb3.py
==========

OpenManipulator-X (4-DOF) on a TurtleBot3 Waffle differential-drive base,
built as a single Robotics-Toolbox ERobot so that jacobe(q) returns the
R^6x6 *holistic* Jacobian (base + arm) used by the Haviland-Corke QP.

Follows the Frankie pattern (rtbdata .../frankie_arm.xacro): the mobile base
is two virtual joints prepended to the arm -- a revolute about z (heading)
and a prismatic along x (forward) -- which is the non-holonomic diff-drive
parameterisation in configuration space. q = [yaw, forward, j1, j2, j3, j4].

Arm geometry is from the real TB3+OMX URDF
(turtlebot3_manipulation_description/urdf/open_manipulator_x.urdf.xacro),
validated this session against /gazebo/link_states to ~1 mm. fkine() with the
base at zero matches src/mobile_grasping/controller/omx_kinematics.fk exactly.

This is a kinematic model for Swift (no actuator dynamics) -- the standard
Haviland/Burgess-Limerick validation setting. Hardware (Phase 2) uses the same
QP with the Dynamixel velocity interface.
"""

import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import ET

# TB3 Waffle (from its URDF): differential drive, wheel sep 0.287 m,
# wheel radius 0.033 m, max linear 0.26 m/s.
TB3_V_MAX = 0.26          # m/s   base forward
TB3_W_MAX = 1.82          # rad/s base yaw (0.26 / (0.287/2))

# OMX-X joint velocity limit. The real Dynamixel XM430-W350 joints do ~4.8 rad/s
# (URDF velocity=4.8). An earlier conservative 1.5 saturated the arm at higher
# base speeds (it could not compensate >0.10 m/s); use the real spec.
OMX_QD_MAX = 4.8          # rad/s per arm joint

N_BASE = 2
N_ARM = 4


def make_omx_tb3(name: str = "OMX_TB3") -> rtb.Robot:
    """Build the OMX-X + TB3 mobile manipulator as an ERobot."""
    ets = (
        ET.Rz()                                   # q0: base heading (yaw)
        * ET.tx()                                 # q1: base forward (prismatic)
        * ET.tx(-0.092) * ET.tz(0.101)            # base_footprint -> link1 mount
        * ET.tx(0.012) * ET.tz(0.017) * ET.Rz()   # joint1 (axis z)
        * ET.tz(0.058) * ET.Ry()                  # joint2 (axis y)
        * ET.tx(0.024) * ET.tz(0.128) * ET.Ry()   # joint3 (axis y)
        * ET.tx(0.124) * ET.Ry()                  # joint4 (axis y)
        * ET.tx(0.126)                            # end-effector
    )
    robot = rtb.Robot(ets, name=name)

    # Per-joint velocity limits: [yaw, forward, j1..j4].
    robot.qdlim = np.array(
        [TB3_W_MAX, TB3_V_MAX, OMX_QD_MAX, OMX_QD_MAX, OMX_QD_MAX, OMX_QD_MAX]
    )

    # Position limits: base virtual joints are effectively unbounded (mobile);
    # arm limits are the real OMX-X URDF values. Needed by joint_velocity_damper.
    robot.qlim = np.array([
        [-100.0, -100.0, -np.pi * 0.9, -np.pi * 0.57, -np.pi * 0.3, -np.pi * 0.57],
        [100.0,  100.0,  np.pi * 0.9,  np.pi * 0.5,   np.pi * 0.44,  np.pi * 0.65],
    ])

    # Useful configs. qr: arm slightly raised so it is out of singularity and
    # the EE is in front of the base, base at origin.
    robot.addconfiguration_attr("qz", np.zeros(robot.n))
    robot.addconfiguration_attr(
        "qr", np.array([0.0, 0.0, 0.0, -0.4, 0.6, 0.4])
    )
    return robot


if __name__ == "__main__":
    r = make_omx_tb3()
    print(r)
    print("n DOF:", r.n)
    for q, tag in [(r.qz, "qz"), (r.qr, "qr")]:
        print(f"  {tag}: EE = {np.round(r.fkine(q).t, 4)}")
