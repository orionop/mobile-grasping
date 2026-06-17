"""
plot_m1.py — M1 figure: EE position error vs time, base stationary.

Reads /tmp/m1_swift.csv (from run_m1.py) and renders the convergence curve.
"""

import csv
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    t, e = [], []
    with open("/tmp/m1_swift.csv") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            e.append(float(row["pos_err"]) * 100.0)   # cm

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, e, color="#2b6cb0", lw=1.8, label="Closed-loop (reactive QP)")
    ax.axhline(1.0, color="green", ls="--", lw=0.8, label="1 cm grasp tolerance")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("End-effector position error (cm)")
    ax.set_title("End-effector convergence to a fixed target (stationary base)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    os.makedirs("figures", exist_ok=True)
    out = "figures/m1_convergence_swift.png"
    fig.savefig(out, dpi=150)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
