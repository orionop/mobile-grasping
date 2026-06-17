"""
run_m2.py — M2: base moves at constant v, arm holds a fixed world EE target.

The base forward virtual joint is advanced externally at a constant speed (this
is the "base driven at v_hat" of the on-the-move task). The holistic QP solves
for the ARM joints to keep the end-effector on a fixed world-frame target while
the base rolls underneath it.

Two conditions (the Exp-1 comparison):
  closed-loop  (feedback=True) : QP uses the LIVE base configuration in J(q),
                                 so the arm reactively compensates base motion.
  open-loop    (feedback=False): QP uses the base configuration LATCHED at t=0
                                 (Kiyokawa-style blind execution) -> EE drifts
                                 as the base moves, with no mechanism to correct.

Run:
    swift_venv/bin/python swift_sim/run_m2.py --feedback   # closed-loop
    swift_venv/bin/python swift_sim/run_m2.py --open       # open-loop
    swift_venv/bin/python swift_sim/run_m2.py --open --headless
"""

import argparse
import csv

import numpy as np
import qpsolvers as qp
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm

from omx_tb3 import make_omx_tb3, N_BASE


def solve_arm_qd(robot, q_ctrl, Tep, base_qd=None):
    """Holistic QP at configuration q_ctrl; returns arm joint velocities.

    The base joints are locked in the QP (the base is driven externally), so the
    solver only moves the arm to track the EE target from the given config.

    base_qd (closed-loop only): the KNOWN base joint velocity [yaw, forward].
    The arm feed-forwards it -- the desired arm-induced EE twist is the servo
    correction MINUS the EE motion the base induces (J_base @ base_qd), so the
    arm cancels base motion instead of only reacting to the resulting drift.
    """
    n = robot.n
    Te_ctrl = robot.fkine(q_ctrl)
    v, _ = rtb.p_servo(Te_ctrl, Tep, gain=1.5, threshold=0.005)

    if base_qd is not None:
        J = robot.jacobe(q_ctrl)
        v = v - J[:, :N_BASE] @ np.asarray(base_qd)

    Q = np.eye(n + 6)
    Q[:n, :n] *= 0.01
    slack_w = np.r_[np.ones(3) * 1.0e4, np.ones(3) * 1.0e-1]   # hard pos, soft ori
    Q[n:, n:] = np.diag(slack_w)

    Aeq = np.c_[robot.jacobe(q_ctrl), np.eye(6)]
    beq = v.reshape((6,))

    Ain = np.zeros((n + 6, n + 6))
    bin = np.zeros(n + 6)
    q_save = robot.q
    robot.q = q_ctrl
    Ain[:n, :n], bin[:n] = robot.joint_velocity_damper(0.05, 0.9, n)
    c = np.zeros(n + 6)
    c[:n] = -0.1 * robot.jacobm(q_ctrl).reshape((n,))
    robot.q = q_save

    qd_lim = robot.qdlim[:n].copy()
    lb = -np.r_[qd_lim, 10 * np.ones(6)]
    ub = np.r_[qd_lim, 10 * np.ones(6)]
    lb[:N_BASE] = 0.0     # base locked in the QP (driven externally)
    ub[:N_BASE] = 0.0

    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="quadprog")
    return np.zeros(n) if qd is None else qd[:n]


def run(feedback: bool, headless: bool, v_base: float = 0.10,
        drive_time: float = 5.0, settle: float = 2.5):
    robot = make_omx_tb3()
    robot.q = robot.qr
    n = robot.n

    env = None
    if not headless:
        import swift
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(robot)

    # Fixed world target, placed in front of the start pose, in reach.
    Tep = sm.SE3(0.20, 0.10, 0.18) * sm.SE3.OA([0, 1, 0], [0, 0, -1])
    if env is not None:
        env.add(sg.Sphere(0.02, pose=Tep, color=[0.2, 0.8, 0.2, 0.8]))

    dt = 0.02
    log = []
    q_base_latched = None
    t = 0.0

    # Phase 1: settle the arm on the target (base still). Phase 2: drive base.
    while t < settle + drive_time:
        driving = t >= settle
        if driving:
            # Advance the base forward externally at constant v (the "v_hat").
            robot.q[1] += v_base * dt

        if q_base_latched is None:
            q_base_latched = robot.q.copy()

        # Control configuration: live (closed) or base-frozen-at-t0 (open).
        # Closed-loop also feed-forwards the known base velocity; open-loop runs
        # blind on the stale config with no base-motion knowledge.
        if feedback:
            q_ctrl = robot.q.copy()
            base_qd = np.array([0.0, v_base]) if driving else None
        else:
            q_ctrl = robot.q.copy()
            q_ctrl[:N_BASE] = q_base_latched[:N_BASE]   # stale base pose
            base_qd = None

        qd_arm = solve_arm_qd(robot, q_ctrl, Tep, base_qd=base_qd)
        robot.q[N_BASE:] = robot.q[N_BASE:] + qd_arm[N_BASE:] * dt

        if env is not None:
            env.step(dt)

        # Always log the TRUE EE position error (live config).
        pos_err = float(np.linalg.norm(Tep.t - robot.fkine(robot.q).t))
        log.append((t, pos_err, "drive" if driving else "settle"))
        t += dt

    drive_errs = [e for (tt, e, ph) in log if ph == "drive"]
    cond = "closed" if feedback else "open"
    print(f"M2 [{cond}] v={v_base} m/s: drive-window EE error "
          f"max={max(drive_errs)*100:.2f}cm mean={np.mean(drive_errs)*100:.2f}cm "
          f"final={drive_errs[-1]*100:.2f}cm")

    out = f"/tmp/m2_{cond}.csv"
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "pos_err", "phase"])
        w.writerows(log)
    print(f"logged {out}")
    if env is not None:
        env.hold()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--feedback", action="store_true", help="closed-loop")
    g.add_argument("--open", action="store_true", help="open-loop baseline")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--v", type=float, default=0.10)
    args = ap.parse_args()
    run(feedback=not args.open, headless=args.headless, v_base=args.v)
