# Lab PC Setup (ROS 1 Noetic + Gazebo Classic)

This guide assumes the lab PC has Ubuntu 20.04 with ROS 1 Noetic and
Gazebo Classic 11 already installed. Only the project-specific
dependencies are added here.

## 1. Install TurtleBot 3 and OpenManipulator-X packages

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-turtlebot3-bringup \
    ros-noetic-dynamixel-sdk \
    ros-noetic-robotis-manipulator
```

## 2. Build TurtleBot 3 Manipulation packages from source

The combined TB3 + OpenManipulator-X packages (`turtlebot3_manipulation_*`)
are not in the Noetic apt repos; they need to be built from ROBOTIS-GIT
source.

```bash
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Symlink this project's ROS package into the catkin workspace

The `mobile_grasping_ros` package lives in this repo at
`ros_ws/src/mobile_grasping_ros/`. Symlink it into the catkin workspace
so updates pulled via git are picked up without copying.

```bash
ln -s "$(pwd)/ros_ws/src/mobile_grasping_ros" ~/catkin_ws/src/
```

(Adjust the source path to wherever you cloned the repo on the lab PC.)

## 4. Install Python dependencies for the QP solver

The QP solver inside `mobile_grasping_ros` needs `qpsolvers`, `osqp`,
and `numpy`. ROS Noetic uses system Python 3.8.

```bash
pip3 install --user "numpy<2" qpsolvers[osqp]
```

## 5. Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 6. Verify the build

```bash
rospack find mobile_grasping_ros
rosnode info /mobile_grasping_controller  # after launching
```

## 7. Launch the simulation

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch mobile_grasping_ros sim.launch
```

Expected: Gazebo Classic opens with the TB3 Waffle + OpenManipulator-X
in an empty world. Three nodes are running (`controller`, `predictor`,
`estimator`). The controller publishes zeros on `/arm_velocity_cmd` for
now (real velocity computation lands in the next iteration once the
full Holistic formulation is in place).

## 8. Useful sanity checks

```bash
# Verify topics are alive
rostopic list
rostopic hz /grasp_pose       # should be ~30 Hz
rostopic hz /base_pose        # should be ~50 Hz
rostopic hz /arm_velocity_cmd # should be ~200 Hz

# Inspect what the controller is publishing
rostopic echo /arm_velocity_cmd

# Inspect the predicted grasp pose
rostopic echo /grasp_pose
```
