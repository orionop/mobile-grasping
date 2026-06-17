"""
qp_solver.py
=============

QP-based reactive controller for a mobile manipulator, following the
Holistic formulation of:

    J. Haviland, N. Sünderhauf, P. Corke,
    "A Holistic Approach to Reactive Mobile Manipulation,"
    IEEE Robotics and Automation Letters, 7(2):3122-3129, 2022.

The decision variable stacks the base virtual joint velocities, the arm
joint velocities, and a 6-D end-effector velocity slack:

    x = [ qdot_base ;  qdot_arm ;  delta ]   in R^(n_base + n_arm + 6)

The QP solved each control step is:

    min   0.5 x^T Q x + C^T x
     x
    s.t.  J x = v_desired      (equality on end-effector twist with slack)
          A x <= B             (inequality: joint position limit avoidance)
          X_minus <= x <= X_plus

with the augmented Jacobian J = [ J_base | J_arm(q) | I_6 ].

This first version implements the structural pieces (equality with
slack-augmented Jacobian, joint velocity bounds, slack penalty) and
leaves the manipulability gradient, adaptive weights, and velocity
dampers as TODOs that follow the same paper.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
import qpsolvers


@dataclass
class HolisticQPConfig:
    """Tunable parameters for the Holistic QP.

    Field defaults match the reference test on the Panda arm. They are
    intended to be overridden per platform (TB3 + OMX-X will use
    different bounds and slack weights).
    """

    # Decision-variable dimensions
    n_arm: int = 7
    n_base: int = 2  # diff-drive virtual joints (theta_dot, x_dot)
    n_slack: int = 6

    # Velocity bounds
    v_max_arm: float = 1.5     # rad/s
    v_max_base: float = 0.3    # m/s

    # Slack penalty (higher = tighter tracking). Scalar applied to all 6
    # end-effector twist components unless slack_weights overrides per-axis.
    slack_penalty: float = 1.0e6

    # Per-component slack weights (vx, vy, vz, wx, wy, wz). Enables
    # task-space relaxation: on a redundancy-free 4-DOF arm the full 6-D
    # twist is infeasible, so orientation axes get a low weight (soft) while
    # position stays hard. None -> use scalar slack_penalty on all axes.
    slack_weights: Optional[tuple] = None

    # QP solver selection
    solver: str = "osqp"

    @property
    def n_total(self) -> int:
        """Number of joint-velocity decision variables (no slack)."""
        return self.n_base + self.n_arm

    @property
    def n_x(self) -> int:
        """Total decision variable dimension."""
        return self.n_total + self.n_slack


class HolisticQPSolver:
    """One-step QP solver for the Holistic mobile-manipulation controller.

    Usage
    -----
    >>> cfg = HolisticQPConfig(n_arm=7, n_base=2)
    >>> solver = HolisticQPSolver(cfg)
    >>> qdot_base, qdot_arm, slack = solver.solve(
    ...     J_arm=panda.jacob0(q),
    ...     J_base=np.zeros((6, 2)),
    ...     v_desired=np.array([0.1, 0, 0, 0, 0, 0]),
    ... )
    """

    def __init__(self, config: HolisticQPConfig):
        self.cfg = config

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(
        self,
        J_arm: np.ndarray,
        J_base: np.ndarray,
        v_desired: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Solve one QP step.

        Parameters
        ----------
        J_arm : (6, n_arm) array
            Body or world-frame Jacobian of the arm at the current joint
            configuration.
        J_base : (6, n_base) array
            Jacobian contribution of the mobile base virtual joints to the
            end-effector twist. Zero matrix is acceptable for an isolated
            arm-only test.
        v_desired : (6,) array
            Desired end-effector spatial velocity (vx, vy, vz, wx, wy, wz)
            in the same frame as J_arm and J_base.

        Returns
        -------
        qdot_base : (n_base,) array
            Base virtual joint velocities.
        qdot_arm : (n_arm,) array
            Arm joint velocities.
        slack : (n_slack,) array
            End-effector velocity slack absorbed by the QP.
        """
        self._validate_shapes(J_arm, J_base, v_desired)
        qp = self._build_matrices(J_arm, J_base, v_desired)
        x = qpsolvers.solve_qp(
            qp["P"], qp["q"],
            A=qp["A"], b=qp["b"],
            lb=qp["lb"], ub=qp["ub"],
            solver=self.cfg.solver,
        )
        if x is None:
            raise RuntimeError(
                "QP infeasible. Check Jacobian conditioning and bounds."
            )
        return self._unpack(x)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _validate_shapes(
        self, J_arm: np.ndarray, J_base: np.ndarray, v_desired: np.ndarray
    ) -> None:
        if J_arm.shape != (6, self.cfg.n_arm):
            raise ValueError(
                f"J_arm has shape {J_arm.shape}, expected (6, {self.cfg.n_arm})."
            )
        if J_base.shape != (6, self.cfg.n_base):
            raise ValueError(
                f"J_base has shape {J_base.shape}, expected (6, {self.cfg.n_base})."
            )
        if v_desired.shape != (6,):
            raise ValueError(
                f"v_desired has shape {v_desired.shape}, expected (6,)."
            )

    def _build_matrices(
        self,
        J_arm: np.ndarray,
        J_base: np.ndarray,
        v_desired: np.ndarray,
    ) -> dict:
        n_total = self.cfg.n_total
        n_slack = self.cfg.n_slack
        n_x = self.cfg.n_x

        # Augmented Jacobian: J = [ J_base | J_arm | I_6 ]
        J_aug = np.hstack([J_base, J_arm, np.eye(n_slack)])

        # Quadratic cost: identity on joint vels, penalty on slack.
        # Per-axis weights enable task-space relaxation (soft orientation).
        Q = np.eye(n_x)
        if self.cfg.slack_weights is not None:
            Q[n_total:, n_total:] = np.diag(np.asarray(self.cfg.slack_weights, float))
        else:
            Q[n_total:, n_total:] *= self.cfg.slack_penalty
        C = np.zeros(n_x)  # manipulability term will land here later

        # Bounds
        X_plus = np.concatenate([
            np.full(self.cfg.n_base, self.cfg.v_max_base),
            np.full(self.cfg.n_arm, self.cfg.v_max_arm),
            np.full(n_slack, np.inf),
        ])
        X_minus = -X_plus

        return {
            "P": Q,
            "q": C,
            "A": J_aug,
            "b": v_desired,
            "lb": X_minus,
            "ub": X_plus,
        }

    def _unpack(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        qdot_base = x[: self.cfg.n_base]
        qdot_arm = x[self.cfg.n_base : self.cfg.n_total]
        slack = x[self.cfg.n_total :]
        return qdot_base, qdot_arm, slack
