#!/usr/bin/env python3
"""Export recorded navigation data as a standalone scientific figure (no live ROS)."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from navigation_metrics import read_events, transform_truth, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists")
    events = read_events(args.events)
    validate(events)
    report = json.loads(args.report.read_text())
    goals = [e for e in events if e["kind"] == "goal"]
    begin = min((e["t"] for e in goals), default=min(e["t"] for e in events))
    events = sorted([e for e in events if e["t"] >= begin], key=lambda e: e["t"])
    frame = goals[0]["frame_id"] if goals else "map"
    poses = [e for e in events if e["kind"] == "pose" and e["frame_id"] == frame]
    alignment = report["configuration"]["truth_alignment"]
    truths = [e for e in events if e["kind"] == "truth" and
              (e["frame_id"] == frame or alignment is not None)]
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    trajectory, linear, angular, timing = axes.flat
    if poses:
        trajectory.plot([p["x"] for p in poses], [p["y"] for p in poses],
                        label="estimated pose", linewidth=1.5)
    if truths:
        values = [transform_truth(e, alignment) for e in truths]
        trajectory.plot([p[0] for p in values], [p[1] for p in values],
                        "--", label="independent world pose", linewidth=1.3)
    else:
        trajectory.text(0.02, 0.98, "No independent truth in matching frame",
                        transform=trajectory.transAxes, va="top", fontsize=9)
    paths = [e for e in events if e["kind"] == "path" and e["frame_id"] == frame]
    if paths:
        points = paths[0].get("points", [])
        trajectory.plot([p[0] for p in points], [p[1] for p in points],
                        ":", label="first recorded plan", color="grey")
    predictions = [e for e in events if e["kind"] == "prediction"
                   and e["frame_id"] == frame and len(e.get("points", [])) > 1]
    if predictions:
        predicted = predictions[len(predictions)//2]
        points = predicted["points"]
        trajectory.plot([p[0] for p in points], [p[1] for p in points], color="teal",
                        label=f"selected MPC prediction at {predicted['t']-begin:.2f}s")
    for index, goal in enumerate(goals):
        if goal["frame_id"] == frame:
            trajectory.plot(goal["x"], goal["y"], "x", color="black")
            trajectory.annotate(f"G{index+1}", (goal["x"], goal["y"]))
    trajectory.set(title=f"Recorded navigation in {frame}", xlabel="x [m]", ylabel="y [m]")
    trajectory.axis("equal")
    topics = ("cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel_cbf", "cmd_vel")
    for topic in topics:
        values = [e for e in events if e["kind"] == "command" and e.get("topic") == topic]
        if values:
            linear.plot([e["t"] - begin for e in values], [e["v"] for e in values],
                        label=topic, linewidth=1)
            angular.plot([e["t"] - begin for e in values], [e["w"] for e in values],
                         label=topic, linewidth=1)
    linear.set(title="Linear command chain", xlabel="simulation time since first goal [s]",
               ylabel="v [m/s]")
    angular.set(title="Angular command chain", xlabel="simulation time since first goal [s]",
                ylabel="w [rad/s]")
    diagnostics = [e for e in events if e["kind"] == "diagnostic"]
    for controller in sorted({e.get("controller", "unknown") for e in diagnostics}):
        values = [e for e in diagnostics if e.get("controller") == controller]
        timing.plot([e["t"] - begin for e in values], [e["compute_ms"] for e in values],
                    label=controller, linewidth=1)
    timing.axhline(50, linestyle="--", color="red", label="20 Hz period (not server latency)")
    timing.set(title="Plugin compute time (including TF/adapter)",
               xlabel="simulation time since first goal [s]", ylabel="compute [ms]")
    for axis in axes.flat:
        axis.grid(alpha=0.25)
        handles, labels = axis.get_legend_handles_labels()
        if handles:
            axis.legend(fontsize=8)
    metadata = report.get("metadata", {})
    figure.suptitle(f"{metadata.get('scenario', 'recorded session')} / "
                    f"{metadata.get('controller_requested', 'mixed controllers')}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    plt.close(figure)
    print(args.output)


if __name__ == "__main__":
    main()
