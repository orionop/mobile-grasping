"""mobile_grasping_ros: ROS 1 Noetic nodes for closed-loop on-the-move grasping.

The QP solver itself lives in `qp_solver.py` and is a near-verbatim copy
of the controller-side module in the parent repo
(`../../../src/mobile_grasping/controller/qp_solver.py`). Keeping the
ROS-package copy self-contained avoids cross-tree imports and makes the
catkin build hermetic. The two copies are intended to stay in sync; a
future refactor may install the parent repo as a pip dependency in the
catkin environment to remove the duplication.
"""

__version__ = "0.1.0"
