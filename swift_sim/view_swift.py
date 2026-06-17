"""
view_swift.py — watch M1 / M2 in the Swift browser viewer (real OMX-X + TB3 meshes).

Visualisation only. It imports the SAME validated control used by the M1/M2
milestone runs (solve_arm_qd, feasible_grasp_pose, pose_error from run_m2) and
drives a MESH robot (omx_tb3_urdf.make_omx_tb3_mesh) whose kinematics match the
M1/M2 ETS model exactly (FK identical to 0.000 mm). The robot renders with the
real ROBOTIS meshes in Swift, like the Haviland/Corke Frankie demos.

The M1/M2 files are NOT modified; this only consumes their control functions.

Run (Swift opens in your browser):
    swift_venv/bin/python swift_sim/view_swift.py --m1          # base still
    swift_venv/bin/python swift_sim/view_swift.py --m2-closed   # base moving, ours
    swift_venv/bin/python swift_sim/view_swift.py --m2-open     # base moving, baseline

If Swift errors with "no running event loop", pin websockets first:
    swift_venv/bin/pip install "websockets<11"
"""

import argparse
import numpy as np
import spatialgeometry as sg
import swift

from omx_tb3_urdf import make_omx_tb3_mesh
from run_m2 import solve_arm_qd, feasible_grasp_pose, pose_error

N_BASE = 2


def view(mode, v_base=0.10, drive_dist=0.15, settle=3.0, dt=0.02):
    robot = make_omx_tb3_mesh()          # real-mesh robot, ETS-identical kinematics
    robot.q = robot.qr

    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot)                       # renders the OMX-X + TB3 meshes

    Tep = feasible_grasp_pose([0.22, 0.0, 0.06])
    env.add(sg.Sphere(0.02, pose=Tep, color=[0.2, 0.8, 0.2, 0.8]))

    feedback = (mode != "m2-open")
    moving = (mode != "m1")
    q_latched = None
    t = 0.0
    dist = 0.0
    while t < settle or (moving and dist < drive_dist):
        driving = moving and t >= settle
        if driving:
            robot.q[1] += v_base * dt
            dist += v_base * dt
        if q_latched is None:
            q_latched = robot.q.copy()

        if feedback:
            q_ctrl = robot.q.copy()
            base_qd = np.array([0.0, v_base]) if driving else None
        else:
            q_ctrl = robot.q.copy()
            q_ctrl[:N_BASE] = q_latched[:N_BASE]
            base_qd = None

        qd = solve_arm_qd(robot, q_ctrl, Tep, base_qd=base_qd)
        robot.q[N_BASE:] = robot.q[N_BASE:] + qd[N_BASE:] * dt
        env.step(dt)

        if mode == "m1" and t >= settle:
            break
        t += dt

    pe, oe = pose_error(robot.fkine(robot.q), Tep)
    print(f"[{mode}] final pose error: {pe*100:.2f} cm, {oe:.2f} deg")
    print("Swift viewer holding. Ctrl-C to exit.")
    env.hold()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--m1", action="store_true")
    g.add_argument("--m2-closed", action="store_true")
    g.add_argument("--m2-open", action="store_true")
    args = ap.parse_args()
    mode = "m1" if args.m1 else ("m2-closed" if args.m2_closed else "m2-open")
    view(mode)
