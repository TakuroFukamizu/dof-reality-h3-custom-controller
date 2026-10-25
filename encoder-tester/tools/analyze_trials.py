#!/usr/bin/env python3
"""Plot one capture on a shared time axis.

The figure is a drive response. It is not a PPR measurement.
"""

import argparse
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(description="Plot encoder-tester CSV. This does not compute PPR.")
    parser.add_argument("csv_path")
    parser.add_argument("--out", help="png path. Refuses to overwrite.")
    return parser.parse_args()


def as_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def load_rows(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("csv has no header")
        required = ["t_us", "applied_duty_pct", "pwm_raw", "velocity_count_s", "count_total_x4", "stop_reason"]
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError("csv missing columns: %s" % ", ".join(missing))
        rows = []
        for row in reader:
            t_us = as_float(row.get("t_us"))
            if t_us is None:
                continue
            rows.append(row)
        return rows


def column(rows, name):
    values = []
    for row in rows:
        number = as_float(row.get(name))
        values.append(0.0 if number is None else number)
    return values


def main():
    args = parse_args()
    if not os.path.isfile(args.csv_path):
        print("csv not found: %s" % args.csv_path, file=sys.stderr)
        return 2
    try:
        rows = load_rows(args.csv_path)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if not rows:
        print("csv has no data rows", file=sys.stderr)
        return 2

    if args.out:
        out = args.out
    else:
        base, _ext = os.path.splitext(args.csv_path)
        out = base + "_plot.png"
    if os.path.exists(out):
        print("refusing to overwrite %s" % out, file=sys.stderr)
        return 2

    t0 = as_float(rows[0]["t_us"])
    t_s = [(as_float(row["t_us"]) - t0) / 1.0e6 for row in rows]
    duty = column(rows, "applied_duty_pct")
    pwm = column(rows, "pwm_raw")
    speed = column(rows, "velocity_count_s")
    count = column(rows, "count_total_x4")

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle("This is not a PPR measurement")
    axes[0].plot(t_s, duty, color="tab:orange", label="applied duty %")
    twin = axes[0].twinx()
    twin.plot(t_s, speed, color="tab:blue", label="count/s")
    axes[0].set_ylabel("duty %")
    twin.set_ylabel("count/s")
    axes[0].legend(loc="upper left")
    twin.legend(loc="upper right")

    axes[1].plot(t_s, pwm, color="tab:orange", label="pwm raw")
    axes[1].set_ylabel("pwm raw")
    axes[1].legend(loc="upper left")

    axes[2].plot(t_s, count, color="black", label="count total x4")
    axes[2].set_ylabel("count")
    axes[2].set_xlabel("time s")
    axes[2].legend(loc="upper left")

    previous = ""
    for t, row in zip(t_s, rows):
        reason = (row.get("stop_reason") or "").strip()
        if reason and reason != previous:
            for axis in axes:
                axis.axvline(t, color="tab:red", linewidth=0.8, alpha=0.8)
            axes[0].annotate(reason, (t, max(duty) if duty else 0), fontsize=8, rotation=90, va="bottom")
        previous = reason

    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
