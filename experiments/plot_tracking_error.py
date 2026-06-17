"""
plot_tracking_error.py
======================

Render the Exp 1 figure (EE tracking error vs time) from one or more
CSVs produced by eval_node.py.

Single CSV (one condition):
  python experiments/plot_tracking_error.py /tmp/tracking_error.csv

Overlay open-loop vs closed-loop (Exp 1 primary figure):
  python experiments/plot_tracking_error.py \
      closed:/tmp/closed_loop.csv open:/tmp/open_loop.csv \
      --out figures/exp1_tracking.png --title "M2: base @ 0.05 m/s"

Each arg is either PATH or LABEL:PATH. Output PNG goes to --out
(default figures/tracking_error.png).
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _load(path):
    t, pos_err, theta_err = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            pos_err.append(float(row["pos_err"]) * 100.0)   # m -> cm
            theta_err.append(float(row["theta_err_deg"]))
    return t, pos_err, theta_err


def _parse_series(arg):
    if ":" in arg and not os.path.exists(arg):
        label, path = arg.split(":", 1)
        return label, path
    return os.path.splitext(os.path.basename(arg))[0], arg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("series", nargs="+", help="PATH or LABEL:PATH")
    ap.add_argument("--out", default="figures/tracking_error.png")
    ap.add_argument("--title", default="End-effector tracking error")
    args = ap.parse_args()

    fig, (ax_pos, ax_th) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    for s in args.series:
        label, path = _parse_series(s)
        t, pos_err, theta_err = _load(path)
        ax_pos.plot(t, pos_err, label=label, linewidth=1.5)
        ax_th.plot(t, theta_err, label=label, linewidth=1.5)

    ax_pos.axhline(1.0, color="r", ls="--", lw=0.8,
                   label="1 cm success threshold")
    ax_pos.set_ylabel("position error (cm)")
    ax_pos.set_title(args.title)
    ax_pos.legend()
    ax_pos.grid(True, alpha=0.3)

    ax_th.axhline(5.0, color="r", ls="--", lw=0.8,
                  label="5 deg success threshold")
    ax_th.set_ylabel("orientation error (deg)")
    ax_th.set_xlabel("time (s)")
    ax_th.legend()
    ax_th.grid(True, alpha=0.3)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
