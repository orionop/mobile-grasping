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

## Lab PC workflow (shared machine, isolated workspace)

The lab PC is shared with other users. This workspace is built and
sourced in isolation from any other catkin workspace on the machine,
and Python dependencies are installed at `--user` level so they do not
affect other lab users.

Project location on the lab PC: `~/Desktop/anurag_ws/mobile-grasping/`
Catkin workspace: `~/Desktop/anurag_ws/mobile-grasping/ros_ws/`

1. `git pull` from this repo location.
2. First-time only: follow `INSTALL.md` (apt deps with check-first,
   source clones of `turtlebot3_manipulation_*` into our own `src/`,
   `pip3 install --user qpsolvers osqp`).
3. `cd ~/Desktop/anurag_ws/mobile-grasping/ros_ws && catkin_make`
4. New terminal:

   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/Desktop/anurag_ws/mobile-grasping/ros_ws/devel/setup.bash
   export TURTLEBOT3_MODEL=waffle_pi

   # Verify the platform install first (official ROBOTIS launch, no extras)
   roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

   # Once that works, launch the full pipeline (Gazebo + our 3 nodes)
   roslaunch mobile_grasping_ros sim.launch
   ```

The platform setup follows the official ROBOTIS guide at
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/.

`~/.bashrc` is intentionally not modified; each terminal opts in by
sourcing the workspace `setup.bash` once.

See `INSTALL.md` for the full first-time setup, including the
pre-flight check of what's already installed and the isolation
principles followed throughout.
