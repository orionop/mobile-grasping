# Progress Log

Running log of implementation milestones for the on-the-move grasping project.

---

## 2026-05-31 — Day 1: Toolchain validated on Apple Silicon

**Goal:** Get the Haviland-Corke Holistic QP controller's underlying toolchain
working on a MacBook Pro M3 Pro, before moving to ROS 2 / Linux / hardware.

**Done:**
- Repo structure: `src/`, `Docs/`, `tests/`, `notebooks/`, `experiments/`
- Python 3.9 venv in `.venv`
- Dependencies installed and pinned in `pyproject.toml`:
  - `numpy<2` (roboticstoolbox 1.1 C extensions built against numpy 1.x)
  - `roboticstoolbox-python` 1.1.1
  - `spatialmath-python`, `swift-sim`
  - `qpsolvers` 4.8.2 with `osqp` 1.1.1 backend
- `tests/test_imports.py` validates all imports work, loads Panda model
- `experiments/exp01_holistic_reference.py` implements one QP step of the
  Holistic controller:
  - Decision variable x = [qdot_base (2), qdot_arm (7), slack (6)] = 15-D
  - Cost: `0.5 x^T Q x + C^T x` with heavy slack penalty (1e6)
  - Equality: augmented Jacobian `J = [J_base | J_arm | I_6]` constraint
  - Bounds: joint velocity limits (1.5 rad/s arm, 0.3 m/s base)
- Verified: for a desired EE twist of (0.1, 0, 0, 0, 0, 0) m/s in world frame,
  the QP returns arm joint velocities that realise the twist to <2e-6 error

**What this validates:**
- Apple Silicon supports the full toolchain
- QP solver runs in <10 ms for a 15-D problem (fast enough for 100+ Hz control)
- Reference math from Haviland-Corke RA-L 2022 implements correctly

**What this does NOT include yet:**
- Manipulability cost term (C vector with arm-only Yoshikawa gradient)
- Adaptive weights `lambda_q`, `lambda_delta` as functions of pose error
- Velocity-damper inequality constraints for joint position limits
- Base orientation cost `theta_epsilon`
- Position-Based Servoing wrapper to convert grasp pose target to desired twist
- Adaptation to 4-DOF OMX-X (current uses 7-DOF Panda)
- Non-holonomic differential-drive base kinematics for TB3 Waffle

**Next:**
- Decide ROS distribution + simulator combo for the lab PC
  (candidates: ROS 2 Humble + Gazebo Classic, ROS 2 Jazzy + Gazebo
  Sim, ROS 1 Noetic + Gazebo Classic). Lab PC currently has a missing
  pointer issue with gz-sim on Jazzy.
- Day 2 (at IITB, once stack decided): load combined TB3 + OMX-X URDF,
  bring up in chosen simulator
- Day 3-4: extend `exp01` with manipulability cost + velocity dampers
  + PBS wrapper, still on Panda for validation
- Day 5-6: adapt to OMX-X (4-DOF Jacobian, smaller workspace, modified
  cost weights for reduced redundancy)

**Documents:**
- `Docs/writeup.tex`, `Docs/writeup.pdf` — Kiyokawa-facing progress
  writeup for this milestone.

---

## 2026-05-31 — Day 1 (afternoon): ROS 1 Noetic workspace skeleton

**Goal:** Author the ROS 1 catkin package on macOS so the lab PC can
pull and build it tomorrow without further authoring.

**Stack decision:** ROS 1 Noetic + Gazebo Classic. The lab PC had a
missing-pointer error with gz-sim on ROS 2 Jazzy; Noetic + Gazebo
Classic is the path of least resistance and the standard combo for
TurtleBot 3 + OpenManipulator-X.

**Done:**
- Extracted the QP from `experiments/exp01_holistic_reference.py` into
  a reusable module at `src/mobile_grasping/controller/qp_solver.py`
  with a clean `HolisticQPSolver` / `HolisticQPConfig` API.
- Created the ROS 1 catkin package
  `ros_ws/src/mobile_grasping_ros/`:
  - `package.xml`, `CMakeLists.txt`, `setup.py` (Python-only catkin)
  - `scripts/controller_node.py`: subscribes to `/grasp_pose`,
    `/base_pose`, `/joint_states`; publishes `/arm_velocity_cmd`
  - `scripts/predictor_node.py`: placeholder, publishes a fixed
    `PoseStamped` at 30 Hz on `/grasp_pose`
  - `scripts/estimator_node.py`: subscribes `/odom`, republishes as
    `PoseStamped` on `/base_pose` at 50 Hz
  - `config/controller_params.yaml`: control rate, joint bounds,
    slack penalty, dimensions
  - `launch/pipeline.launch`: brings up the three nodes
  - `launch/sim.launch`: composes with
    `turtlebot3_manipulation_gazebo/empty_world.launch` for a full
    Gazebo bring-up
  - Self-contained copy of `qp_solver.py` inside the ROS package so
    the catkin build is hermetic
- Wrote `ros_ws/INSTALL.md` with explicit apt + source install steps
  for the lab PC (TB3, OpenManipulator-X, `turtlebot3_manipulation` from
  ROBOTIS-GIT, Python `qpsolvers` + `osqp`).
- Wrote `ros_ws/README.md` documenting the workspace layout and the
  parent-repo / ROS-package relationship.
- Updated top-level `README.md` to add the lab PC quick start and the
  expanded layout.

**What this lets the lab PC do tomorrow:**
- `git pull`
- Follow `ros_ws/INSTALL.md` once for first-time deps
- Symlink `ros_ws/src/mobile_grasping_ros` into `~/catkin_ws/src/`
- `catkin_make` then `source devel/setup.bash`
- `export TURTLEBOT3_MODEL=waffle`
- `roslaunch mobile_grasping_ros sim.launch`
- Expected: Gazebo opens with TB3 Waffle + OpenManipulator-X; three
  nodes up; `/grasp_pose` and `/base_pose` flowing; controller
  publishing zeros on `/arm_velocity_cmd` (placeholder until the full
  Holistic formulation lands)

**What this does NOT include yet:**
- Actual QP-based velocity computation inside
  `controller_node._compute_command()` (currently publishes zeros)
- 4-DOF arm Jacobian for OMX-X (kinematics module to be added)
- Manipulability cost term, adaptive weights, velocity dampers
- Real predictor (current node publishes a fixed pose)
- Real base pose estimator (current node passes /odom through)

**Next (Day 2 at IITB lab PC):**
- Build the workspace, fix anything that breaks
- Verify Gazebo spawns TB3 + OMX-X correctly
- Verify all three nodes start and publish on expected topics
- Capture rostopic hz / rostopic echo output for the writeup
