"""
record.py — render M1 / M2 simulation videos (mp4) without the browser.

roboticstoolbox's built-in movie export is broken on matplotlib 3.9
(Axes3D.w_xaxis removed), and Swift renders only in a browser tab, so this
module re-runs the control loop, collects the joint trajectory, and renders a
matplotlib 3D skeleton animation (kinematic chain + base footprint + EE trace +
target) to mp4 via ffmpeg.

Outputs:
  figures/video_m1.mp4         static target, base stationary
  figures/video_m2_closed.mp4  base moving, closed-loop reactive QP
  figures/video_m2_open.mp4    base moving, open-loop

Run:
  swift_venv/bin/python swift_sim/record.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import spatialmath as sm

from omx_tb3 import make_omx_tb3, N_BASE
from run_m2 import solve_arm_qd


def simulate(feedback, v_base, drive_time=5.0, settle=2.5, dt=0.02):
    """Run the M1/M2 loop; return (qtraj, phases, robot, Tep, errs)."""
    robot = make_omx_tb3()
    robot.q = robot.qr
    Tep = sm.SE3(0.20, 0.10, 0.18) * sm.SE3.OA([0, 1, 0], [0, 0, -1])

    qtraj, phases, errs = [], [], []
    q_latched = None
    t = 0.0
    while t < settle + drive_time:
        driving = t >= settle
        if driving and v_base > 0:
            robot.q[1] += v_base * dt
        if q_latched is None:
            q_latched = robot.q.copy()

        if feedback:
            q_ctrl = robot.q.copy()
            base_qd = np.array([0.0, v_base]) if (driving and v_base > 0) else None
        else:
            q_ctrl = robot.q.copy()
            q_ctrl[:N_BASE] = q_latched[:N_BASE]
            base_qd = None

        qd_arm = solve_arm_qd(robot, q_ctrl, Tep, base_qd=base_qd)
        robot.q[N_BASE:] = robot.q[N_BASE:] + qd_arm[N_BASE:] * dt

        qtraj.append(robot.q.copy())
        phases.append("drive" if driving else "settle")
        errs.append(float(np.linalg.norm(Tep.t - robot.fkine(robot.q).t)))
        t += dt
    return np.array(qtraj), phases, robot, Tep, errs


def render(qtraj, phases, robot, Tep, errs, out, title, stride=3):
    """Render a 3D skeleton animation to mp4."""
    idx = list(range(0, len(qtraj), stride))
    target = Tep.t
    ee_trace = []

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    def frame(k):
        ax.cla()
        q = qtraj[k]
        pts = np.array([X.t for X in robot.fkine_all(q)])
        ee = robot.fkine(q).t
        ee_trace.append(ee)
        base_xy = pts[2]   # base mount origin (after the 2 virtual base joints)

        # kinematic chain
        ax.plot(pts[2:, 0], pts[2:, 1], pts[2:, 2], "-o", color="#2b6cb0",
                lw=2.5, ms=4, label="OMX-X arm")
        # base footprint (square on the ground)
        s = 0.14
        bx, by = base_xy[0], base_xy[1]
        ax.plot([bx - s, bx + s, bx + s, bx - s, bx - s],
                [by - s, by - s, by + s, by + s, by - s],
                [0, 0, 0, 0, 0], "-", color="#555", lw=1.5, label="TB3 base")
        # target
        ax.scatter(*target, color="#2ca02c", s=90, marker="*", label="target")
        # EE trace
        if len(ee_trace) > 1:
            tr = np.array(ee_trace)
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], "-", color="#c23b3b",
                    lw=1.0, alpha=0.7)

        ax.set_xlim(-0.2, 0.9); ax.set_ylim(-0.5, 0.5); ax.set_zlim(0, 0.5)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
        err_cm = errs[k] * 100
        ph = phases[k]
        ax.set_title(f"{title}\nt={k*0.02:.1f}s   phase={ph}   "
                     f"EE error={err_cm:.2f} cm")
        ax.legend(loc="upper right", fontsize=8)
        ax.view_init(elev=22, azim=-60)

    anim = FuncAnimation(fig, frame, frames=idx, interval=60)
    os.makedirs("figures", exist_ok=True)
    writer = FFMpegWriter(fps=20, bitrate=2400)
    anim.save(out, writer=writer)
    plt.close(fig)
    print(f"saved {out}  ({len(idx)} frames)")


def main():
    cases = [
        (True,  0.0,  "M1: static target, base stationary", "figures/video_m1.mp4"),
        (True,  0.10, "M2: base moving 0.10 m/s, closed-loop (reactive QP)",
         "figures/video_m2_closed.mp4"),
        (False, 0.10, "M2: base moving 0.10 m/s, open-loop",
         "figures/video_m2_open.mp4"),
    ]
    for fb, v, title, out in cases:
        qtraj, phases, robot, Tep, errs = simulate(fb, v)
        render(qtraj, phases, robot, Tep, errs, out, title)


if __name__ == "__main__":
    main()
