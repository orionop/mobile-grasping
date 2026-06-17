"""
omx_tb3_urdf.py — OMX-X + TB3 mobile manipulator as a MESH URDF for Swift.

Builds a self-contained URDF (real ROBOTIS meshes, absolute paths) so the robot
renders with proper geometry in Swift -- the same way rtb's Frankie does. The
kinematics are identical to omx_tb3.make_omx_tb3 (ETS): two diff-drive virtual
base joints (revolute yaw + prismatic forward) prepended to the OMX-X arm.

Mesh files (downloaded to swift_sim/meshes/):
  chain_link1..5.stl  (OMX-X arm), waffle_pi_base.stl (TB3 base).
Joint origins validated this session vs /gazebo/link_states (~1 mm). Visual
origins from the ROBOTIS URDF (link2 mesh at z=0.018, rest 0; scale 0.001).

make_omx_tb3_mesh() returns an rtb ERobot with visual geometry; its fkine
matches the ETS model exactly.
"""

import os
import numpy as np
import roboticstoolbox as rtb

_HERE = os.path.dirname(os.path.abspath(__file__))
_MESH = os.path.join(_HERE, "meshes")


def _mesh(name):
    return os.path.join(_MESH, name)


def _urdf_string():
    return f"""<?xml version="1.0"?>
<robot name="omx_tb3">

  <link name="base_footprint"/>

  <!-- Diff-drive virtual base joints: yaw (revolute z) + forward (prismatic x) -->
  <joint name="base_yaw" type="revolute">
    <parent link="base_footprint"/>
    <child link="base_yaw_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" velocity="1.82" lower="-100" upper="100"/>
  </joint>
  <link name="base_yaw_link"/>

  <joint name="base_fwd" type="prismatic">
    <parent link="base_yaw_link"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="100" velocity="0.26" lower="-100" upper="100"/>
  </joint>

  <!-- TB3 Waffle base -->
  <link name="base_link">
    <visual>
      <origin xyz="-0.064 0 0.010" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{_mesh('waffle_pi_base.stl')}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark"><color rgba="0.2 0.2 0.2 1"/></material>
    </visual>
  </link>

  <!-- Mount: base_link -> link1 (OMX-X rear mount). z = 0.101 matches the
       validated base_footprint->link1 origin (base_link is coincident with
       base_footprint in this kinematic chain). -->
  <joint name="mount" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="-0.092 0 0.101" rpy="0 0 0"/>
  </joint>

  <link name="link1">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{_mesh('chain_link1.stl')}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="omx"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.012 0 0.017" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" velocity="4.8" lower="-2.83" upper="2.83"/>
  </joint>
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.018" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{_mesh('chain_link2.stl')}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="omx"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.058" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1" velocity="4.8" lower="-1.79" upper="1.57"/>
  </joint>
  <link name="link3">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{_mesh('chain_link3.stl')}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="omx"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <joint name="joint3" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0.024 0 0.128" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1" velocity="4.8" lower="-0.94" upper="1.38"/>
  </joint>
  <link name="link4">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{_mesh('chain_link4.stl')}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="omx"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <joint name="joint4" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0.124 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="1" velocity="4.8" lower="-1.79" upper="2.04"/>
  </joint>
  <link name="link5">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{_mesh('chain_link5.stl')}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="omx"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
  </link>

  <!-- End-effector frame (gripper centre) -->
  <joint name="ee" type="fixed">
    <parent link="link5"/><child link="ee_link"/>
    <origin xyz="0.126 0 0" rpy="0 0 0"/>
  </joint>
  <link name="ee_link"/>

</robot>
"""


def make_omx_tb3_mesh():
    """Return an rtb ERobot (OMX-X + TB3) with real-mesh visual geometry."""
    urdf_path = os.path.join(_HERE, "omx_tb3.urdf")
    with open(urdf_path, "w") as f:
        f.write(_urdf_string())

    links, name, _, _ = rtb.Robot.URDF_read(urdf_path)
    robot = rtb.Robot(links, name=name)

    robot.qdlim = np.array([1.82, 0.26, 4.8, 4.8, 4.8, 4.8])
    robot.qr = np.array([0.0, 0.0, 0.0, -0.4, 0.6, 0.4])
    robot.qz = np.zeros(robot.n)
    robot.addconfiguration_attr("qr", robot.qr)
    robot.addconfiguration_attr("qz", robot.qz)
    return robot


if __name__ == "__main__":
    r = make_omx_tb3_mesh()
    print(r)
    print("n DOF:", r.n)
    print("EE at qr:", np.round(r.fkine(r.qr).t, 4))
    # cross-check against the ETS model
    from omx_tb3 import make_omx_tb3
    ets = make_omx_tb3()
    d = np.linalg.norm(r.fkine(r.qr).t - ets.fkine(ets.qr).t)
    print(f"FK match vs ETS model: {d*1000:.3f} mm")
    nvis = sum(len(getattr(l, 'geometry', []) or []) for l in r.links)
    print("links with visual geometry:", nvis)
