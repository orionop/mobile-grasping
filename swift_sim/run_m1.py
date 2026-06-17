"""
run_m1.py — M1: arm converges to a fixed world-frame grasp POSE, base stationary.

Holistic QP (Haviland-Corke) on the OMX-X + TB3 mobile manipulator. The base is
locked (handled inside solve_arm_qd by zero base-velocity bounds); only the
4 arm joints move. The target is a FEASIBLE grasp pose (position + the
4-DOF-achievable orientation), and convergence is judged on the FULL pose:
position < 1 cm AND orientation < 5 deg.

Run:
    swift_venv/bin/python swift_sim/run_m1.py            # Swift viewer
    swift_venv/bin/python swift_sim/run_m1.py --headless # log only

Done when the end-effector holds the grasp pose within 1 cm and 5 deg.
"""

import argparse
import csv

import numpy as np
import spatialgeometry as sg

from omx_tb3 import make_omx_tb3, N_BASE
from run_m2 import solve_arm_qd, feasible_grasp_pose, pose_error


def run(headless: bool = False, max_t: float = 6.0):
    robot = make_omx_tb3()
    robot.q = robot.qr

    env = None
    if not headless:
        import swift
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(robot)

    Tep = feasible_grasp_pose([0.22, 0.0, 0.06])
    if env is not None:
        env.add(sg.Sphere(0.02, pose=Tep, color=[0.2, 0.8, 0.2, 0.8]))

    dt = 0.02
    log = []
    t = 0.0
    t_arrived = None
    while t < max_t:
        qd = solve_arm_qd(robot, robot.q.copy(), Tep)   # base locked inside
        robot.q[N_BASE:] = robot.q[N_BASE:] + qd[N_BASE:] * dt
        if env is not None:
            env.step(dt)

        pos_err, ori_err = pose_error(robot.fkine(robot.q), Tep)
        log.append((t, pos_err, ori_err))
        if t_arrived is None and pos_err < 0.01 and ori_err < 5.0:
            t_arrived = t
        t += dt

    pe, oe = log[-1][1], log[-1][2]
    print(f"M1 done: reached 1cm&5deg at t={t_arrived:.2f}s  |  "
          f"settled pos={pe*100:.2f}cm ori={oe:.2f}deg")
    with open("/tmp/m1_swift.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "pos_err", "ori_err"])
        w.writerows(log)
    print("logged /tmp/m1_swift.csv")
    if env is not None:
        env.hold()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()
    run(headless=args.headless)
