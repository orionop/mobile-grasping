"""
plot_m2.py — render the Exp-1 figure: EE tracking error vs time, open vs closed.

Reads /tmp/m2_closed.csv and /tmp/m2_open.csv (from run_m2.py) and overlays
position error over time, marking the settle->drive transition.
"""

import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    t, e, ph = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            e.append(float(row["pos_err"]) * 100.0)   # cm
            ph.append(row["phase"])
    return t, e, ph


def main():
    tc, ec, phc = load("/tmp/m2_closed.csv")
    to, eo, _ = load("/tmp/m2_open.csv")
    drive_start = next(t for t, p in zip(tc, phc) if p == "drive")

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(to, eo, label="Open-loop (Kiyokawa-style, blind)", color="#c23b3b", lw=1.8)
    ax.plot(tc, ec, label="Closed-loop (ours, reactive)", color="#2b6cb0", lw=1.8)
    ax.axvline(drive_start, color="gray", ls=":", lw=1,
               label="base starts moving (0.10 m/s)")
    ax.axhline(1.0, color="green", ls="--", lw=0.8, label="1 cm grasp threshold")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("EE position error (cm)")
    ax.set_title("M2 / Exp-1: EE tracking while base moves — open vs closed loop")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = "figures/exp1_m2_swift.png"
    import os
    os.makedirs("figures", exist_ok=True)
    fig.savefig(out, dpi=150)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
