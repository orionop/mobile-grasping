"""
exp2_sweep.py — M2 robustness: base-velocity sweep + QP solve-rate measurement.

Sweeps the base velocity over {0.05, 0.10, 0.15, 0.20} m/s, closed-loop vs
open-loop, and reports per-velocity tracking error during the drive window plus
the fraction of the drive window within the 1 cm grasp tolerance. Also measures
the QP solve rate (Hz) to substantiate the ~200 Hz claim.

Outputs:
  figures/exp2_success_vs_velocity.png   error / in-tolerance vs base velocity
  prints a table + mean QP solve rate

Run:
  swift_venv/bin/python swift_sim/exp2_sweep.py
"""

import time
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import spatialmath as sm

from omx_tb3 import make_omx_tb3, N_BASE
from run_m2 import solve_arm_qd

VELOCITIES = [0.05, 0.10, 0.15, 0.20]
TOL = 0.01           # 1 cm grasp tolerance
DT = 0.02
SETTLE = 2.5
# Drive a FIXED DISTANCE at each velocity (not a fixed time): the world-fixed
# target must stay inside the arm's ~0.38 m reach for the comparison to be fair.
# A fixed time would push the base past the target at high speed (a workspace-
# reach limit, not a control failure). 0.30 m keeps the target reachable
# throughout the pass at every velocity.
DRIVE_DIST = 0.30


def run_case(feedback, v_base, solve_times):
    robot = make_omx_tb3()
    robot.q = robot.qr
    Tep = sm.SE3(0.20, 0.10, 0.18) * sm.SE3.OA([0, 1, 0], [0, 0, -1])
    q_latched = None
    t = 0.0
    drive_dist = 0.0
    drive_errs = []
    while t < SETTLE or drive_dist < DRIVE_DIST:
        driving = t >= SETTLE
        if driving:
            robot.q[1] += v_base * DT
            drive_dist += v_base * DT
        if q_latched is None:
            q_latched = robot.q.copy()

        if feedback:
            q_ctrl = robot.q.copy()
            base_qd = np.array([0.0, v_base]) if driving else None
        else:
            q_ctrl = robot.q.copy()
            q_ctrl[:N_BASE] = q_latched[:N_BASE]
            base_qd = None

        t0 = time.perf_counter()
        qd_arm = solve_arm_qd(robot, q_ctrl, Tep, base_qd=base_qd)
        solve_times.append(time.perf_counter() - t0)

        robot.q[N_BASE:] = robot.q[N_BASE:] + qd_arm[N_BASE:] * DT
        if driving:
            drive_errs.append(float(np.linalg.norm(Tep.t - robot.fkine(robot.q).t)))
        t += DT

    e = np.array(drive_errs)
    return {
        "max_cm": e.max() * 100,
        "mean_cm": e.mean() * 100,
        "in_tol_pct": float(np.mean(e < TOL) * 100),
    }


def main():
    solve_times = []
    rows = []
    for v in VELOCITIES:
        c = run_case(True, v, solve_times)
        o = run_case(False, v, solve_times)
        rows.append((v, c, o))

    # --- table ---
    print(f"{'v[m/s]':>7} | {'closed max':>10} {'closed mean':>11} "
          f"{'closed<1cm':>10} | {'open max':>9} {'open mean':>9}")
    for v, c, o in rows:
        print(f"{v:>7.2f} | {c['max_cm']:>9.2f}c {c['mean_cm']:>10.2f}c "
              f"{c['in_tol_pct']:>9.0f}% | {o['max_cm']:>8.2f}c {o['mean_cm']:>8.2f}c")

    st = np.array(solve_times)
    print(f"\nQP solve: mean {st.mean()*1e3:.3f} ms  -> {1.0/st.mean():.0f} Hz  "
          f"(p95 {np.percentile(st,95)*1e3:.3f} ms -> {1.0/np.percentile(st,95):.0f} Hz, "
          f"{len(st)} solves)")

    # --- figure ---
    vs = [r[0] for r in rows]
    cmean = [r[1]["mean_cm"] for r in rows]
    cmax = [r[1]["max_cm"] for r in rows]
    omean = [r[2]["mean_cm"] for r in rows]
    omax = [r[2]["max_cm"] for r in rows]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(vs, cmean, "-o", color="#2b6cb0", label="Closed-loop (mean)")
    ax.fill_between(vs, cmean, cmax, color="#2b6cb0", alpha=0.15,
                    label="Closed-loop (mean–max)")
    ax.plot(vs, omean, "-s", color="#c23b3b", label="Open-loop (mean)")
    ax.fill_between(vs, omean, omax, color="#c23b3b", alpha=0.12,
                    label="Open-loop (mean–max)")
    ax.axhline(1.0, color="green", ls="--", lw=0.8, label="1 cm grasp tolerance")
    ax.set_xlabel("Base velocity (m/s)")
    ax.set_ylabel("End-effector position error (cm)")
    ax.set_title("Tracking error vs. base velocity: open-loop vs. closed-loop")
    ax.set_xticks(vs)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    os.makedirs("figures", exist_ok=True)
    out = "figures/exp2_success_vs_velocity.png"
    fig.savefig(out, dpi=150)
    print(f"\nsaved {out}")


if __name__ == "__main__":
    main()
