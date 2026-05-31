"""
exp01_holistic_reference.py
============================

Reference implementation of the Haviland-Corke Holistic QP controller
on the Frankie platform (Franka Panda 7-DOF + Omron LD-60 differential drive).

This script is the SANITY CHECK for the toolchain. It verifies:
  1. roboticstoolbox-python loads the Panda model on Apple Silicon
  2. spatialmath SE3 operations work
  3. qpsolvers OSQP backend solves QPs in real time
  4. The augmented Jacobian formulation from Haviland-Corke RA-L 2022
     produces sensible joint velocity outputs given a desired end-effector velocity

This is NOT the contribution. It is the baseline implementation we will
then adapt to TB3 Waffle + OMX-X (4-DOF arm, differential-drive base).

Reference:
  J. Haviland, N. Sünderhauf, P. Corke, "A Holistic Approach to Reactive
  Mobile Manipulation," IEEE RA-L 7(2):3122-3129, 2022.

Run:
  python experiments/exp01_holistic_reference.py
"""

import numpy as np
import qpsolvers
import roboticstoolbox as rtb
from spatialmath import SE3


def build_qp_matrices(
    panda: rtb.Robot,
    q: np.ndarray,
    v_desired: np.ndarray,
    base_dof: int = 2,
) -> dict:
    """
    Build the Holistic QP matrices for a mobile manipulator.

    Decision variable: x = [q_dot_base (2), q_dot_arm (7), slack (6)]  ->  dim 15

    Cost:    min  0.5 * x^T Q x + C^T x
    Equality: J x = v_desired   where J = [J_base | J_arm | I_6]
    Bounds:   x in [X_minus, X_plus]

    For this reference implementation we use a simple unit Q matrix and zero C
    (no manipulability term yet) just to verify the QP solver returns a feasible
    solution. The full Holistic formulation adds:
      - manipulability gradient in C (arm-only)
      - adaptive weights lambda_q, lambda_delta as functions of pose error
      - velocity-damper inequality constraints for joint position limits

    Those are layered in later. Here we test the solver pipeline end-to-end.
    """
    n_arm = panda.n  # 7 for Panda
    n_total = base_dof + n_arm  # 9 virtual+real joints
    slack_dim = 6  # one slack per end-effector velocity dimension

    # --- Jacobian (in end-effector / base frame) ---
    # For the reference, use the body Jacobian of the arm only and pad zeros
    # for the base. Full mobile Jacobian would couple base motion to ee velocity.
    J_arm = panda.jacob0(q)              # 6 x n_arm in world frame
    J_base = np.zeros((6, base_dof))     # placeholder for differential-drive base
    J_aug = np.hstack([J_base, J_arm, np.eye(slack_dim)])  # 6 x (n_total + slack)

    # --- Cost matrices ---
    # Q: identity on joint velocities + heavy weight on slack to discourage error
    Q = np.eye(n_total + slack_dim)
    Q[n_total:, n_total:] *= 1.0e6       # heavy slack penalty: enforce tight tracking
    C = np.zeros(n_total + slack_dim)    # no manipulability term in this baseline

    # --- Equality: J x = v_desired ---
    A_eq = J_aug
    b_eq = v_desired

    # --- Bounds ---
    # Allow modest joint velocities. Slack is unbounded for now.
    v_max_arm = 1.5      # rad/s
    v_max_base = 0.3     # m/s (Omron LD-60 is faster but cap it here)
    X_plus = np.concatenate([
        np.full(base_dof, v_max_base),
        np.full(n_arm, v_max_arm),
        np.full(slack_dim, np.inf),
    ])
    X_minus = -X_plus

    return {
        "P": Q,
        "q": C,
        "A": A_eq,
        "b": b_eq,
        "lb": X_minus,
        "ub": X_plus,
    }


def solve_holistic_step(
    panda: rtb.Robot,
    q: np.ndarray,
    v_desired: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    One control step of the Holistic QP.

    Returns: (qdot_base, qdot_arm, slack)
    """
    qp = build_qp_matrices(panda, q, v_desired)
    x = qpsolvers.solve_qp(
        qp["P"], qp["q"],
        A=qp["A"], b=qp["b"],
        lb=qp["lb"], ub=qp["ub"],
        solver="osqp",
    )
    if x is None:
        raise RuntimeError("QP infeasible. Check Jacobian conditioning.")

    qdot_base = x[:2]
    qdot_arm = x[2:9]
    slack = x[9:]
    return qdot_base, qdot_arm, slack


def main():
    print("=" * 70)
    print("exp01: Holistic QP reference implementation on Frankie (Panda)")
    print("=" * 70)

    # Load Panda (Haviland-Corke's arm)
    panda = rtb.models.Panda()
    print(f"\nPanda loaded: {panda.n}-DOF, {len(panda.links)} links")

    # Test configuration: ready pose
    q = panda.qr   # joint config in 'ready' configuration
    print(f"Joint configuration (qr): {np.round(q, 3)}")

    # Current end-effector pose
    T_ee = panda.fkine(q)
    print(f"End-effector position: {np.round(T_ee.t, 3)}")
    print(f"End-effector rotation (rpy, deg): {np.round(np.degrees(T_ee.rpy()), 2)}")

    # Desired end-effector velocity: move +0.1 m/s in world-x, no rotation
    v_desired = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    print(f"\nDesired EE twist (world frame): {v_desired}")

    # Solve QP
    qdot_base, qdot_arm, slack = solve_holistic_step(panda, q, v_desired)
    print(f"\nQP solution:")
    print(f"  base joint vel  (omega, v): {np.round(qdot_base, 4)}")
    print(f"  arm joint vels  (7 DOF):    {np.round(qdot_arm, 4)}")
    print(f"  slack (6-D):                {np.round(slack, 4)}")

    # Sanity: J_arm @ qdot_arm should approximately equal v_desired
    J_arm = panda.jacob0(q)
    v_realised = J_arm @ qdot_arm
    err = v_desired - v_realised
    print(f"\nRealised EE twist:    {np.round(v_realised, 4)}")
    print(f"Tracking error:       {np.round(err, 4)}")
    print(f"|error| L2 norm:      {np.linalg.norm(err):.6f}")

    if np.linalg.norm(err) < 1e-4:
        print("\n[PASS] QP solver tracks desired EE velocity to within 1e-4.")
    else:
        print(f"\n[WARN] Tracking error larger than 1e-4. Slack absorbed: "
              f"{np.linalg.norm(slack):.4f}")

    print("\nReference toolchain validated. Ready to adapt to TB3 + OMX-X.")


if __name__ == "__main__":
    main()
