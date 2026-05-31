# ros_ws/

ROS 1 Noetic catkin packages for the mobile-grasping project.

## Layout

```
ros_ws/
├── INSTALL.md                          # Lab PC setup instructions
├── README.md                           # This file
└── src/
    └── mobile_grasping_ros/            # Single ROS 1 package
        ├── package.xml                 # ROS dependencies
        ├── CMakeLists.txt              # Build (catkin)
        ├── setup.py                    # Python module install
        ├── launch/
        │   ├── sim.launch              # Full sim: Gazebo + TB3+OMX + pipeline
        │   └── pipeline.launch         # Just our 3 nodes
        ├── config/
        │   └── controller_params.yaml  # QP tuning
        ├── scripts/                    # Executable ROS nodes (Python 3)
        │   ├── controller_node.py
        │   ├── predictor_node.py
        │   └── estimator_node.py
        └── src/mobile_grasping_ros/    # Importable Python module
            ├── __init__.py
            └── qp_solver.py            # Holistic QP solver
```

## How this package relates to the parent repo

The repo root (`mobile-grasping/`) contains the controller-side Python
code (`src/mobile_grasping/`) and the standalone experiment scripts
(`experiments/`) that run on the macOS dev machine without any ROS
dependency. The `ros_ws/src/mobile_grasping_ros/` package wraps the
same QP solver in ROS 1 nodes so it can run on the lab Linux PC against
Gazebo and real hardware.

The `qp_solver.py` inside this ROS package is a self-contained copy of
the controller-side module. The two copies are kept in sync manually
for now; a future refactor may install the parent repo as a pip
dependency in the ROS environment to remove the duplication.

## Lab PC workflow

1. Pull the latest from the GitHub repo.
2. If `mobile_grasping_ros` is not yet symlinked into `~/catkin_ws/src/`,
   follow `INSTALL.md` step 3.
3. `cd ~/catkin_ws && catkin_make`
4. `source devel/setup.bash`
5. `export TURTLEBOT3_MODEL=waffle`
6. `roslaunch mobile_grasping_ros sim.launch`

See `INSTALL.md` for first-time setup of TurtleBot 3 + OpenManipulator-X
packages from source.
