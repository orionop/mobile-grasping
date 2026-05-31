# mobile-grasping

Closed-loop reactive control for on-the-move grasping on TurtleBot 3 Waffle + OpenManipulator-X.

Collaboration with Prof. Takuya Kiyokawa (Osaka University) and Dr. Arpita Sinha (IIT Bombay).
Extending Kiyokawa et al., "Self-Supervised Learning of Grasping Arbitrary Objects On-the-Move," IEEE/SICE SII 2025.

## Status

| Component | Status |
|---|---|
| Reference Holistic QP (Panda, Mac) | Validated (exp01) |
| Reusable Python QP module (`src/mobile_grasping/controller/qp_solver.py`) | Drafted |
| ROS 1 Noetic catkin package (`ros_ws/src/mobile_grasping_ros/`) | Skeleton (pending lab PC build) |
| Full Holistic formulation (manipulability + dampers + PBS) | Not started |
| 4-DOF OMX-X adaptation | Not started |
| TB3 + OMX-X bring-up in Gazebo Classic | Awaiting lab PC test |
| Predictor (FCN / GG-CNN / OpenVLA) | Placeholder node only |
| Base pose estimation (wheel + IMU + VO) | Placeholder node only (republishes /odom) |
| Real-hardware experiments | Not started |

## Workflow

- Code is authored on macOS (M3 Pro) and committed from there.
- The lab Linux PC pulls from this repo to run simulation and hardware.
- The Apple Silicon environment validates the controller-side toolchain
  (Robotics Toolbox, qpsolvers/OSQP) in isolation, without any
  simulator or ROS runtime dependency.

## Repo layout

```
mobile-grasping/
├── Docs/                                # Writeup for collaborator + internal log
│   ├── writeup.tex
│   ├── writeup.pdf
│   └── progress_log.md
├── src/mobile_grasping/                 # Controller-side Python module
│   └── controller/
│       └── qp_solver.py                 # Holistic QP, importable
├── experiments/                         # Standalone experiment scripts
│   └── exp01_holistic_reference.py
├── tests/                               # Toolchain sanity tests
├── notebooks/                           # Math derivations, numerical sanity checks
├── ros_ws/                              # ROS 1 Noetic catkin workspace
│   ├── INSTALL.md                       # Lab PC dependency install steps
│   ├── README.md
│   └── src/mobile_grasping_ros/         # ROS package wrapping the QP solver
│       ├── package.xml, CMakeLists.txt, setup.py
│       ├── launch/                      # sim.launch + pipeline.launch
│       ├── config/                      # controller_params.yaml
│       ├── scripts/                     # controller_node, predictor_node, estimator_node
│       └── src/mobile_grasping_ros/     # qp_solver (copy for hermetic catkin build)
├── pyproject.toml
└── .gitignore
```

## Quick start (macOS, controller-side validation)

```bash
# Create Python environment
python3 -m venv .venv
source .venv/bin/activate
pip install -e .

# Verify toolchain
python tests/test_imports.py

# Run the Holistic QP reference experiment (Panda, single QP step)
python experiments/exp01_holistic_reference.py
```

## Quick start (lab PC, ROS 1 Noetic + Gazebo Classic)

See `ros_ws/INSTALL.md` for first-time setup of TurtleBot 3 +
OpenManipulator-X dependencies. Once those are in place:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch mobile_grasping_ros sim.launch
```

## Progress writeup

The collaborator-facing progress writeup lives at `Docs/writeup.pdf`
and is the document shared with Prof. Kiyokawa between sync meetings.
It is rebuilt from `Docs/writeup.tex` via `pdflatex` and committed
alongside source code changes. The internal day-by-day log lives at
`Docs/progress_log.md`.

## References

1. T. Kiyokawa et al., "Self-Supervised Learning of Grasping Arbitrary Objects On-the-Move," IEEE/SICE SII, 2025.
2. J. Haviland, N. Sünderhauf, P. Corke, "A Holistic Approach to Reactive Mobile Manipulation," IEEE RA-L, 7(2):3122-3129, 2022.
3. J. Haviland, P. Corke, "NEO: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators," IEEE RA-L, 6(2):1043-1050, 2021.
4. B. Burgess-Limerick, C. Lehnert, J. Leitner, P. Corke, "An Architecture for Reactive Mobile Manipulation On-The-Move," IEEE ICRA, 2023.
