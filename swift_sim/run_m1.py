"""
run_m1.py — M1: arm tracks a fixed world-frame EE target, base stationary.

Holistic QP (Haviland-Corke) on the OMX-X + TB3 mobile manipulator in Swift.
For M1 the base is locked (its two virtual joints are held at zero velocity by
zero bounds), so only the 4 arm joints move. Validates the kinematics + QP +
servo loop end-to-end, kinematically, with no actuator dynamics.

Run:
    swift_venv/bin/python swift_sim/run_m1.py            # with Swift viewer
    swift_venv/bin/python swift_sim/run_m1.py --headless # log only, no browser

Done when EE position error converges < 1 cm.
"""

import argparse
import sys
import time

import numpy as np
import qpsolvers as qp
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm

from omx_tb3 import make_omx_tb3, N_BASE


def run(headless: bool = False, target=(0.20, 0.10, 0.15), max_t: float = 8.0):
    robot = make_omx_tb3()
    robot.q = robot.qr
    n = robot.n                       # 6 (2 base + 4 arm)

    env = None
    if not headless:
        import swift
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(robot)

    # Fixed world-frame EE target.
    Tep = sm.SE3(*target) * sm.SE3.OA([0, 1, 0], [0, 0, -1])  # top-down-ish
    if env is not None:
        env.add(sg.Sphere(radius=0.02, pose=Tep, color=[0.2, 0.8, 0.2, 0.8]))

    dt = 0.02
    log = []
    t = 0.0
    arrived = False

    while t < max_t and not arrived:
        Te = robot.fkine(robot.q)
        v, _ = rtb.p_servo(Te, Tep, gain=1.0, threshold=0.005)
        # Position-based convergence (orientation is intentionally relaxed for
        # the 4-DOF arm, so don't gate on the full 6-D p_servo "arrived").
        arrived = bool(np.linalg.norm(Tep.t - Te.t) < 0.005)

        # --- Holistic QP (Haviland-Corke), arm + base decision vars + slack ---
        Y = 0.01                                  # joint-velocity regularisation
        Q = np.eye(n + 6)
        Q[:n, :n] *= Y
        e = float(np.sum(np.abs(np.r_[(Te.inv() * Tep).t,
                                      (Te.inv() * Tep).rpy()])))
        # Task-space relaxation for the redundancy-free 4-DOF arm: HARD position
        # slack (tracked tightly), SOFT orientation slack (the full 6-D twist is
        # infeasible, so orientation about the approach axis is relaxed). Weights
        # mirror the validated qp_solver (position penalty must dominate the
        # manipulability gradient, else the redundancy-free arm trades tracking
        # for manipulability and drifts to a joint limit).
        slack_w = np.r_[np.ones(3) * 1.0e4,    # position: hard
                        np.ones(3) * 1.0e-1]   # orientation: soft
        Q[n:, n:] = np.diag(slack_w)

        Aeq = np.c_[robot.jacobe(robot.q), np.eye(6)]
        beq = v.reshape((6,))

        # Joint-limit velocity dampers (arm); base has none.
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)
        Ain[:n, :n], bin[:n] = robot.joint_velocity_damper(0.05, 0.9, n)

        # Manipulability maximisation (linear term). On a redundancy-free 4-DOF
        # arm this term is load-bearing (no redundancy to resolve singularities)
        # but must stay small relative to the position-slack penalty, else it
        # overpowers tracking. Low gain keeps it a secondary objective.
        c = np.zeros(n + 6)
        c[:n] = -0.1 * robot.jacobm(robot.q).reshape((n,))

        # Bounds: M1 locks the base (zero velocity on the 2 base joints).
        qd_lim = robot.qdlim[:n].copy()
        lb = -np.r_[qd_lim, 10 * np.ones(6)]
        ub = np.r_[qd_lim, 10 * np.ones(6)]
        lb[:N_BASE] = 0.0
        ub[:N_BASE] = 0.0

        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="quadprog")
        if qd is None:
            qd = np.zeros(n + 6)

        robot.qd = qd[:n]
        if env is not None:
            env.step(dt)
        else:
            robot.q = robot.q + robot.qd * dt

        pos_err = float(np.linalg.norm(Tep.t - Te.t))
        log.append((t, pos_err))
        t += dt

    err = log[-1][1] if log else float("nan")
    print(f"M1 done: t={t:.2f}s  final EE pos error = {err*100:.2f} cm  "
          f"arrived={arrived}")
    # write CSV
    import csv
    with open("/tmp/m1_swift.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "pos_err"])
        w.writerows(log)
    print("logged /tmp/m1_swift.csv")
    if env is not None:
        env.hold()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()
    run(headless=args.headless)
