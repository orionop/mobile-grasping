# Progress Log

Running log of implementation milestones for the on-the-move grasping project.

---

## 2026-05-31 — Day 1: Toolchain validated on Apple Silicon

**Goal:** Get the Haviland-Corke Holistic QP controller's underlying toolchain
working on a MacBook Pro M3 Pro, before moving to ROS 2 / Linux / hardware.

**Done:**
- Repo structure: `src/`, `docs/`, `tests/`, `notebooks/`, `experiments/`
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
- Day 2 (at IITB): set up ROS 2 Jazzy workspace, load TB3 Waffle URDF, load
  OMX-X URDF, combine into single mobile manipulator URDF, launch in Gazebo
- Day 3-4: extend `exp01` with manipulability cost + velocity dampers + PBS
  wrapper, still on Panda for validation
- Day 5-6: adapt to OMX-X (4-DOF Jacobian, smaller workspace, modified cost
  weights for reduced redundancy)
