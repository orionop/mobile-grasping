cmake_minimum_required(VERSION 3.0.2)
project(mobile_grasping_ros)

# ----------------------------------------------------------------
# This is a Python-only package. catkin_python_setup() exposes the
# importable module under src/mobile_grasping_ros so that nodes can
# do `from mobile_grasping_ros.qp_solver import HolisticQPSolver`.
# ----------------------------------------------------------------

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install the executable Python scripts so `rosrun mobile_grasping_ros
# controller_node.py` works after `source devel/setup.bash`.
catkin_install_python(PROGRAMS
  scripts/controller_node.py
  scripts/predictor_node.py
  scripts/estimator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and config files for the launch system to find them.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
