#!/usr/bin/env python3
"""Plot observer behavior from record_curves data1.csv.

This script reads the semantic CSV written by record_y_curves.cpp and compares:
- vision measurement vs remote observer position estimate
- motion-capture relative position/velocity vs smart sensor and remote estimates
- disturbance estimates and gate status

Example:
    python plot_observer_effect.py data1.csv
    python plot_observer_effect.py data1.csv --axis y --output observer_y.png
"""

import argparse
import csv
import math
import os
from typing import Dict, List


AXES = ("x", "y", "z")


def read_csv(path: str) -> Dict[str, List[float]]:
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise RuntimeError("CSV has no header row.")

        data = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in reader.fieldnames:
                text = row.get(name, "")
                try:
                    data[name].append(float(text))
                except (TypeError, ValueError):
                    data[name].append(float("nan"))
    return data


def require_columns(data: Dict[str, List[float]], names: List[str]) -> None:
    missing = [name for name in names if name not in data]
    if missing:
        raise RuntimeError("CSV missing columns: " + ", ".join(missing))


def column(data: Dict[str, List[float]], name: str) -> List[float]:
    return data[name]


def relative(data: Dict[str, List[float]], axis: str, kind: str) -> List[float]:
    target = column(data, f"{axis}_target_gt_{kind}")
    own = column(data, f"{axis}_gt_{kind}")
    return [a - b for a, b in zip(target, own)]


def smooth(values: List[float], window: int) -> List[float]:
    if window <= 1:
        return values
    out = []
    acc = 0.0
    buf: List[float] = []
    for value in values:
        v = 0.0 if math.isnan(value) else value
        buf.append(v)
        acc += v
        if len(buf) > window:
            acc -= buf.pop(0)
        out.append(acc / len(buf))
    return out


def time_axis(data: Dict[str, List[float]]) -> List[float]:
    if "step" in data and "dt" in data and data["dt"]:
        dt = data["dt"][0] if data["dt"][0] > 0.0 else 0.01
        return [step * dt for step in data["step"]]
    if "elapsed" in data:
        return list(range(len(data["elapsed"])))
    first_key = next(iter(data))
    return list(range(len(data[first_key])))


def plot_one_axis(data: Dict[str, List[float]], axis: str, output: str, smooth_window: int) -> None:
    import matplotlib.pyplot as plt

    require_columns(
        data,
        [
            f"{axis}_remote_e",
            f"{axis}_remote_v",
            f"{axis}_remote_d",
            f"{axis}_smart_e",
            f"{axis}_smart_v",
            f"{axis}_smart_d",
            f"{axis}_vision_error",
            f"{axis}_gt_pos",
            f"{axis}_gt_vel",
            f"{axis}_target_gt_pos",
            f"{axis}_target_gt_vel",
            f"{axis}_gate_metric",
            f"{axis}_gate_pass",
            f"{axis}_used_hold",
            f"{axis}_u",
        ],
    )

    t = time_axis(data)
    gt_e = relative(data, axis, "pos")
    gt_v = relative(data, axis, "vel")

    remote_e = smooth(column(data, f"{axis}_remote_e"), smooth_window)
    smart_e = smooth(column(data, f"{axis}_smart_e"), smooth_window)
    remote_v = smooth(column(data, f"{axis}_remote_v"), smooth_window)
    smart_v = smooth(column(data, f"{axis}_smart_v"), smooth_window)

    fig, axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"{axis.upper()} axis observer check", fontsize=14)

    axs[0].plot(t, gt_e, label="mocap relative pos", linewidth=1.5)
    axs[0].plot(t, column(data, f"{axis}_vision_error"), label="vision error", linewidth=1.0, alpha=0.75)
    axs[0].plot(t, remote_e, label="remote_e", linewidth=1.3)
    axs[0].plot(t, smart_e, label="smart_e", linewidth=1.3)
    axs[0].set_ylabel("position error")
    axs[0].legend(loc="best")
    axs[0].grid(True)

    axs[1].plot(t, gt_v, label="mocap relative vel", linewidth=1.5)
    axs[1].plot(t, remote_v, label="remote_v", linewidth=1.3)
    axs[1].plot(t, smart_v, label="smart_v", linewidth=1.3)
    axs[1].set_ylabel("velocity error")
    axs[1].legend(loc="best")
    axs[1].grid(True)

    axs[2].plot(t, column(data, f"{axis}_remote_d"), label="remote_d", linewidth=1.3)
    axs[2].plot(t, column(data, f"{axis}_smart_d"), label="smart_d", linewidth=1.3)
    axs[2].set_ylabel("disturbance")
    axs[2].legend(loc="best")
    axs[2].grid(True)

    axs[3].plot(t, column(data, f"{axis}_gate_metric"), label="gate_metric", linewidth=1.3)
    if "gate_delta" in data:
        axs[3].plot(t, column(data, "gate_delta"), label="gate_delta", linestyle="--", linewidth=1.0)
    axs[3].set_ylabel("gate")
    axs[3].legend(loc="best")
    axs[3].grid(True)

    axs[4].step(t, column(data, f"{axis}_gate_pass"), where="post", label="gate_pass")
    axs[4].step(t, column(data, f"{axis}_used_hold"), where="post", label="used_hold")
    axs[4].plot(t, column(data, f"{axis}_u"), label="u", linewidth=1.0, alpha=0.75)
    axs[4].set_xlabel("time [s]")
    axs[4].set_ylabel("flags / control")
    axs[4].legend(loc="best")
    axs[4].grid(True)

    fig.tight_layout()
    fig.savefig(output, dpi=160)
    print(f"saved {output}")


def plot_overview(data: Dict[str, List[float]], output: str) -> None:
    import matplotlib.pyplot as plt

    t = time_axis(data)
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle("Observer overview: remote/smart position estimates", fontsize=14)

    for axis in AXES:
        if f"{axis}_remote_e" not in data:
            continue
        gt_e = relative(data, axis, "pos")
        axs[0].plot(t, gt_e, label=f"{axis} mocap rel")
        axs[1].plot(t, column(data, f"{axis}_remote_e"), label=f"{axis} remote_e")
        axs[2].plot(t, column(data, f"{axis}_smart_e"), label=f"{axis} smart_e")

    axs[0].set_ylabel("mocap relative pos")
    axs[1].set_ylabel("remote_e")
    axs[2].set_ylabel("smart_e")
    axs[2].set_xlabel("time [s]")
    for ax in axs:
        ax.legend(loc="best")
        ax.grid(True)

    fig.tight_layout()
    fig.savefig(output, dpi=160)
    print(f"saved {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot observer effect from data1.csv.")
    parser.add_argument("csv", nargs="?", default="data1.csv", help="Path to data1.csv")
    parser.add_argument("--axis", choices=("x", "y", "z", "all"), default="all")
    parser.add_argument("--output", default="", help="Output PNG path for one-axis mode")
    parser.add_argument("--smooth", type=int, default=1, help="Moving-average window for e/v curves")
    args = parser.parse_args()

    data = read_csv(args.csv)
    if not data:
        raise RuntimeError("CSV is empty.")

    out_dir = os.path.dirname(os.path.abspath(args.csv)) or "."
    if args.axis == "all":
        plot_overview(data, os.path.join(out_dir, "observer_overview.png"))
        for axis in AXES:
            plot_one_axis(data, axis, os.path.join(out_dir, f"observer_{axis}.png"), args.smooth)
    else:
        output = args.output or os.path.join(out_dir, f"observer_{args.axis}.png")
        plot_one_axis(data, args.axis, output, args.smooth)


if __name__ == "__main__":
    main()
