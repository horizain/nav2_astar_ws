#!/usr/bin/env python3
"""Offline navigation experiment metrics. Standard library only; never starts ROS.

Input JSONL events: goal, result, pose, truth, path, command, diagnostic.
All t fields use one simulation clock, positions/metres and angles/radians.
A truth event must name its independent source. Frame mismatch is not aligned
automatically; an optional fixed SE(2) transform must be supplied explicitly.
"""
import argparse
import bisect
import csv
import gzip
import json
import math
from pathlib import Path


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def percentile(values, fraction):
    if not values:
        return None
    data = sorted(values)
    index = (len(data) - 1) * fraction
    lo = int(index)
    hi = min(lo + 1, len(data) - 1)
    return data[lo] + (data[hi] - data[lo]) * (index - lo)


def statistics(values):
    if not values:
        return {"count": 0, "mean": None, "rmse": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def distance_to_path(x, y, points):
    if not points:
        return None
    if len(points) == 1:
        return math.hypot(x - points[0][0], y - points[0][1])
    best = math.inf
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        u = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2)) if length2 else 0.0
        best = min(best, math.hypot(x - a[0] - u * dx, y - a[1] - u * dy))
    return best


def validate(events):
    """Reject corrupt numeric values rather than silently turning them into success."""
    numeric = {
        "goal": ("x", "y", "yaw"), "pose": ("x", "y", "yaw"),
        "truth": ("x", "y", "yaw"), "command": ("v", "w"),
        "diagnostic": ("compute_ms",),
    }
    allowed = set(numeric) | {"result", "path", "prediction"}
    for event in events:
        kind = event.get("kind")
        if kind not in allowed:
            raise ValueError(f"Unknown event kind: {kind}")
        for key in ("t",) + numeric.get(kind, ()):
            value = event.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{kind}.{key} must be a finite number")
        if kind in ("goal", "result") and not event.get("goal_id"):
            raise ValueError("Goal/result events require a goal_id")
        if kind in ("goal", "pose", "truth", "path", "prediction") and not event.get("frame_id"):
            raise ValueError(f"{kind} requires an explicit frame_id")
        if kind == "truth" and not event.get("source"):
            raise ValueError("Truth requires an independent source name")
        if kind in ("path", "prediction"):
            for point in event.get("points", []):
                if len(point) != 2 or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in point):
                    raise ValueError("Invalid path point")


def transform_truth(truth, alignment):
    x, y, yaw = truth["x"], truth["y"], truth["yaw"]
    if alignment is not None:
        tx, ty, theta = alignment
        x, y = tx + math.cos(theta) * x - math.sin(theta) * y, ty + math.sin(theta) * x + math.cos(theta) * y
        yaw += theta
    return x, y, yaw


def matched_truth(poses, truths, max_gap, alignment):
    times = [v["t"] for v in truths]
    positional, angular, pairs, offsets = [], [], [], []
    unmatched = 0
    for pose in poses:
        i = bisect.bisect_left(times, pose["t"])
        candidates = [truths[k] for k in (i - 1, i) if 0 <= k < len(truths)]
        candidates = [v for v in candidates if
                      abs(v["t"] - pose["t"]) <= max_gap and
                      (v["frame_id"] == pose["frame_id"] or alignment is not None)]
        if not candidates:
            unmatched += 1
            continue
        truth = min(candidates, key=lambda v: abs(v["t"] - pose["t"]))
        x, y, yaw = transform_truth(truth, alignment)
        error = math.hypot(pose["x"] - x, pose["y"] - y)
        theta_error = abs(angle_difference(pose["yaw"], yaw))
        offsets.append(abs(truth["t"] - pose["t"]))
        positional.append(error)
        angular.append(theta_error)
        pairs.append((pose, error, theta_error))
    return {
        "position_m": statistics(positional),
        "yaw_rad": statistics(angular),
        "unmatched_pose_samples": unmatched,
        "matched_samples": len(pairs),
        "time_offset_s": statistics(offsets),
    }


def summarize(events, max_truth_gap=0.1, alignment=None, endpoint_max_gap=0.5):
    validate(events)
    if not all(math.isfinite(v) and v >= 0 for v in (max_truth_gap, endpoint_max_gap)):
        raise ValueError("Time tolerances must be finite and nonnegative")
    if alignment is not None:
        if len(alignment) != 3 or not all(math.isfinite(v) for v in alignment):
            raise ValueError("Alignment must be a finite SE(2) transform")
        if len({e["frame_id"] for e in events if e["kind"] == "truth"}) > 1:
            raise ValueError("One fixed alignment cannot silently mix truth frames")
    events = sorted(events, key=lambda e: e["t"])
    goals = [e for e in events if e["kind"] == "goal"]
    ids = [g["goal_id"] for g in goals]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate accepted goal id")
    results = {}
    for event in events:
        if event["kind"] == "result":
            if event["goal_id"] not in ids or event["goal_id"] in results:
                raise ValueError("Unknown or duplicate result goal id")
            results[event["goal_id"]] = event
    poses = [e for e in events if e["kind"] == "pose"]
    truths = [e for e in events if e["kind"] == "truth"]
    paths = [e for e in events if e["kind"] == "path"]
    commands = [e for e in events if e["kind"] == "command"]
    diagnostics = [e for e in events if e["kind"] == "diagnostic"]
    runs = []
    for goal in goals:
        result = results.get(goal["goal_id"])
        if result is not None and result["t"] < goal["t"]:
            raise ValueError("Result precedes its accepted goal")
        end = result["t"] if result else max((e["t"] for e in events), default=goal["t"])
        samples = [p for p in poses if goal["t"] <= p["t"] <= end]
        # Deduplicate pose timestamps within a frame before deriving distance.
        samples = list({(p["t"], p["frame_id"]): p for p in samples}.values())
        samples.sort(key=lambda p: p["t"])
        same_frame = [p for p in samples if p["frame_id"] == goal["frame_id"]]
        endpoint = same_frame[-1] if result and same_frame and end - same_frame[-1]["t"] <= endpoint_max_gap else None
        route = [p for p in paths if goal["t"] <= p["t"] <= end]
        route_times = [p["t"] for p in route]
        cross_track = []
        for pose in samples:
            index = bisect.bisect_right(route_times, pose["t"]) - 1
            if index >= 0 and route[index]["frame_id"] == pose["frame_id"]:
                value = distance_to_path(pose["x"], pose["y"], route[index].get("points", []))
                if value is not None:
                    cross_track.append(value)
        final_plan = route[-1] if route and route[-1].get("points") else None
        goal_offset = math.hypot(final_plan["points"][-1][0]-goal["x"],
                                 final_plan["points"][-1][1]-goal["y"]) if final_plan else None
        travelled = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                        for a, b in zip(samples, samples[1:])
                        if a["frame_id"] == b["frame_id"])
        velocity = [e for e in commands if goal["t"] <= e["t"] <= end and e.get("topic") == "cmd_vel_nav"]
        dv, dw, acceleration, angular_acceleration = [], [], [], []
        for a, b in zip(velocity, velocity[1:]):
            dt = b["t"] - a["t"]
            if dt <= 0:
                continue
            dv.append(abs(b["v"] - a["v"]))
            dw.append(abs(b["w"] - a["w"]))
            acceleration.append(dv[-1] / dt)
            angular_acceleration.append(dw[-1] / dt)
        actual = [e for e in diagnostics if goal["t"] <= e["t"] <= end]
        status = result.get("status") if result else "unknown"
        terminal_truth = [e for e in truths if result and abs(e["t"] - end) <= max_truth_gap
                          and (e["frame_id"] == goal["frame_id"] or alignment is not None)]
        terminal_truth = min(terminal_truth, key=lambda e: abs(e["t"] - end)) if terminal_truth else None
        true_pose = transform_truth(terminal_truth, alignment) if terminal_truth else None
        truth_stats = matched_truth(samples, truths, max_truth_gap, alignment)
        runs.append({
            "goal_id": goal["goal_id"], "controller_requested": goal.get("controller"),
            "controllers_observed": sorted({e["controller"] for e in actual if e.get("controller")}),
            "purpose": goal.get("purpose", "reach_goal"),
            "status": status, "success": status == "succeeded",
            "endpoint_true_position_m": math.hypot(true_pose[0] - goal["x"], true_pose[1] - goal["y"]) if true_pose else None,
            "endpoint_true_yaw_rad": abs(angle_difference(true_pose[2], goal["yaw"])) if true_pose else None,
            "elapsed_sim_s": end - goal["t"] if result else None,
            "pose_samples": len(samples),
            "estimated_travel_m": travelled if len(samples) > 1 else None,
            "endpoint_estimated_position_m": math.hypot(endpoint["x"] - goal["x"], endpoint["y"] - goal["y"]) if endpoint else None,
            "endpoint_estimated_yaw_rad": abs(angle_difference(endpoint["yaw"], goal["yaw"])) if endpoint else None,
            "endpoint_sample_age_s": end - endpoint["t"] if endpoint else None,
            "last_plan_goal_offset_m": goal_offset,
            "cross_track_to_latest_plan_m": statistics(cross_track),
            "command_total_variation_v": sum(dv) if dv else None,
            "command_total_variation_w": sum(dw) if dw else None,
            "command_acceleration_m_s2": statistics(acceleration),
            "command_angular_acceleration_rad_s2": statistics(angular_acceleration),
            "controller_compute_ms": statistics([e["compute_ms"] for e in actual]),
            "control_error_samples": sum(e.get("stage") == "control_error" for e in actual),
            "localization_vs_truth": truth_stats,
        })
    reach = [r for r in runs if r["purpose"] == "reach_goal"]
    outcome_counts = {name: sum(r["status"] == name for r in runs)
                      for name in ("succeeded", "canceled", "aborted", "rejected", "unknown")}
    return {
        "schema_version": 1,
        "definitions": {
            "time": "one simulation clock; command timestamps are receive times",
            "endpoint": "last pose at/before terminal result within endpoint_max_gap",
            "cross_track": "nearest segment of latest plan observed during the goal",
            "compute_ms": "TF/adapter/solver computation; excludes diagnostic publication and server waits",
            "truth_matching": "nearest timestamp within max_truth_gap; no interpolation",
            "precision_warning": "estimated errors and success tolerances are not ground-truth accuracy",
        },
        "configuration": {"max_truth_gap_s": max_truth_gap,
                          "endpoint_max_gap_s": endpoint_max_gap,
                          "truth_alignment": list(alignment) if alignment is not None else None,
                          "truth_sources": sorted({e["source"] for e in truths})},
        "outcome_counts": outcome_counts,
        "reach_goal_attempts": len(reach),
        "reach_goal_success_rate": sum(r["success"] for r in reach) / len(reach) if reach else None,
        "expected_cancels_completed": sum(r["purpose"] == "cancel_test" and r["status"] == "canceled" for r in runs),
        "attempted_goals": len(runs),
        "succeeded_goals": sum(r["success"] for r in runs),
        "success_rate": sum(r["success"] for r in runs) / len(runs) if runs else None,
        "unknown_results": sum(r["status"] == "unknown" for r in runs),
        "localization_vs_truth": matched_truth(poses, truths, max_truth_gap, alignment),
        "runs": runs,
    }


def write_report(events, directory, metadata=None, **kwargs):
    result = summarize(events, **kwargs)
    directory = Path(directory)
    # Never overwrite an existing report directory.
    directory.mkdir(parents=True, exist_ok=False)
    result["metadata"] = metadata or {}
    (directory / "report.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    with (directory / "runs.csv").open("w", newline="") as stream:
        keys = ["goal_id", "controller_requested", "status", "success", "elapsed_sim_s",
                "endpoint_estimated_position_m", "endpoint_estimated_yaw_rad",
                "endpoint_true_position_m", "endpoint_true_yaw_rad", "purpose",
                "estimated_travel_m", "control_error_samples"]
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(result["runs"])
    return result


def read_events(path):
    path = Path(path)
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-truth-gap", type=float)
    parser.add_argument("--configuration", type=Path,
                        help="reuse matching/alignment and metadata from a recorded report")
    parser.add_argument("--truth-alignment", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    args = parser.parse_args()
    events = read_events(args.events)
    previous = json.loads(args.configuration.read_text()) if args.configuration else {}
    configuration = previous.get("configuration", {})
    gap = args.max_truth_gap if args.max_truth_gap is not None else configuration.get("max_truth_gap_s", 0.1)
    alignment = args.truth_alignment if args.truth_alignment is not None else configuration.get("truth_alignment")
    report = write_report(events, args.output, max_truth_gap=gap,
                          endpoint_max_gap=configuration.get("endpoint_max_gap_s", 0.5),
                          alignment=alignment, metadata=previous.get("metadata", {}))
    print(json.dumps({k: report[k] for k in
                      ("attempted_goals", "succeeded_goals", "success_rate", "unknown_results")}))
    print(f"Report: {args.output / 'report.json'}")


if __name__ == "__main__":
    main()
