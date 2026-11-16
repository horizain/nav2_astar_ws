#!/usr/bin/env python3
"""Known-answer tests for educational experiment metrics (no ROS)."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_metrics import (angle_difference, distance_to_path, read_events, summarize, write_report)


def pose(t, x, y=0.0, yaw=0.0, kind="pose", **extra):
    return dict(kind=kind, t=t, x=x, y=y, yaw=yaw, frame_id="map", **extra)


def goal(t=0, goal_id="a", x=1.0):
    return dict(kind="goal", t=t, goal_id=goal_id, x=x, y=0.0, yaw=0.0,
                frame_id="map", controller="MPC")


class MetricsTest(unittest.TestCase):
    def test_wrapped_angles_and_segment_projection(self):
        self.assertAlmostEqual(abs(angle_difference(math.radians(179), math.radians(-179))),
                               math.radians(2))
        self.assertAlmostEqual(distance_to_path(0.5, 0.2, [[0, 0], [1, 0]]), 0.2)
        self.assertAlmostEqual(distance_to_path(2, 0, [[0, 0], [1, 0]]), 1.0)

    def test_known_endpoint_and_truth_rmse(self):
        events = [goal(), pose(0, 0), pose(1, 0.5), pose(2, 0.8),
                  dict(kind="result", t=2, goal_id="a", status="succeeded")]
        events += [pose(t, x, kind="truth", source="independent_fixture")
                   for t, x in [(0, 0), (1, 0.6), (2, 1.0)]]
        result = summarize(events)
        self.assertAlmostEqual(result["runs"][0]["endpoint_estimated_position_m"], 0.2)
        self.assertAlmostEqual(result["localization_vs_truth"]["position_m"]["rmse"],
                               math.sqrt(0.05 / 3))
        self.assertEqual(result["success_rate"], 1.0)

    def test_missing_truth_is_unknown_not_zero(self):
        result = summarize([goal(), pose(1, 0.5)])
        self.assertIsNone(result["localization_vs_truth"]["position_m"]["rmse"])
        self.assertIsNone(result["runs"][0]["endpoint_estimated_position_m"])
        self.assertEqual(result["unknown_results"], 1)
        self.assertEqual(result["success_rate"], 0)

    def test_stale_or_different_frame_truth_is_unmatched(self):
        truth = pose(10, 1.0, kind="truth", source="fixture")
        result = summarize([pose(0, 1), truth])
        self.assertEqual(result["localization_vs_truth"]["matched_samples"], 0)
        truth["t"] = 0
        truth["frame_id"] = "world"
        result = summarize([pose(0, 1), truth])
        self.assertEqual(result["localization_vs_truth"]["matched_samples"], 0)
        result = summarize([pose(0, 2), truth], alignment=(1, 0, 0))
        self.assertEqual(result["localization_vs_truth"]["position_m"]["rmse"], 0)

    def test_each_goal_uses_its_own_terminal_sample(self):
        events = [
            goal(), pose(1, 0.8),
            dict(kind="result", t=1, goal_id="a", status="succeeded"),
            goal(2, "b", 3.0), pose(3, 2.5),
            dict(kind="result", t=3, goal_id="b", status="canceled"),
        ]
        result = summarize(events)
        self.assertEqual(result["success_rate"], 0.5)
        self.assertAlmostEqual(result["runs"][0]["endpoint_estimated_position_m"], 0.2)
        self.assertAlmostEqual(result["runs"][1]["endpoint_estimated_position_m"], 0.5)

    def test_command_variation_uses_time_and_correct_topic(self):
        events = [goal(), pose(0.5, 0.2),
                  dict(kind="result", t=1, goal_id="a", status="aborted")]
        for t, v in [(0, 0.0), (0.5, 0.2), (1, 0.3)]:
            events.append(dict(kind="command", t=t, v=v, w=0.0, topic="cmd_vel_nav"))
        events.append(dict(kind="command", t=0.6, v=30.0, w=0.0, topic="cmd_vel"))
        result = summarize(events)["runs"][0]
        self.assertAlmostEqual(result["command_total_variation_v"], 0.3)
        self.assertAlmostEqual(result["command_acceleration_m_s2"]["max"], 0.4)

    def test_missing_endpoint_sample_is_not_reported_as_accurate(self):
        result = summarize([goal(), pose(0, 0.99),
                            dict(kind="result", t=2, goal_id="a", status="succeeded")])
        self.assertIsNone(result["runs"][0]["endpoint_estimated_position_m"])

    def test_corrupt_data_and_duplicate_results_are_rejected(self):
        with self.assertRaises(ValueError):
            summarize([pose(0, math.nan)])
        result = dict(kind="result", t=1, goal_id="a", status="succeeded")
        with self.assertRaises(ValueError):
            summarize([goal(), result, result])

    def test_expected_cancel_is_separate_from_reach_goal_rate(self):
        cancellation = goal(2, "cancel", 2.0)
        cancellation["purpose"] = "cancel_test"
        events = [goal(), pose(1, 1.0),
                  dict(kind="result", t=1, goal_id="a", status="succeeded"),
                  cancellation, pose(3, 1.5),
                  dict(kind="result", t=3, goal_id="cancel", status="canceled")]
        report = summarize(events)
        self.assertEqual(report["success_rate"], 0.5)
        self.assertEqual(report["reach_goal_success_rate"], 1.0)
        self.assertEqual(report["expected_cancels_completed"], 1)

    def test_true_endpoint_can_differ_from_estimated_endpoint(self):
        truth = pose(1, 1.2, kind="truth", source="independent_fixture")
        truth["frame_id"] = "world"
        report = summarize([goal(), pose(1, 1.0), truth,
                            dict(kind="result", t=1, goal_id="a", status="succeeded")],
                           alignment=(0, 0, 0))
        self.assertEqual(report["runs"][0]["endpoint_estimated_position_m"], 0)
        self.assertAlmostEqual(report["runs"][0]["endpoint_true_position_m"], 0.2)

    def test_archived_real_trace_reproduces_recorded_numeric_results(self):
        fixture = Path(__file__).resolve().parents[1] / "examples/fixed_straight_before_goal_anchor"
        import json
        expected = json.loads((fixture / "report.json").read_text())
        actual = summarize(read_events(fixture / "events.jsonl.gz"),
                           alignment=expected["configuration"]["truth_alignment"])
        self.assertEqual(actual["outcome_counts"], expected["outcome_counts"])
        self.assertEqual(actual["localization_vs_truth"]["matched_samples"],
                         expected["localization_vs_truth"]["matched_samples"])
        for measured, original in zip(actual["runs"], expected["runs"]):
            self.assertAlmostEqual(measured["endpoint_true_position_m"],
                                   original["endpoint_true_position_m"], places=12)
            self.assertAlmostEqual(measured["controller_compute_ms"]["p95"],
                                   original["controller_compute_ms"]["p95"], places=12)

    def test_quantized_plan_goal_offset_is_visible(self):
        events = [goal(), pose(1, 0.8),
                  dict(kind="path", t=0, frame_id="map", points=[[0, 0], [0.975, 0]]),
                  dict(kind="result", t=1, goal_id="a", status="succeeded")]
        self.assertAlmostEqual(summarize(events)["runs"][0]["last_plan_goal_offset_m"], 0.025)

    def test_csv_report_does_not_overwrite_existing_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp) / "report"
            write_report([goal()], target)
            self.assertTrue((target / "runs.csv").is_file())
            with self.assertRaises(FileExistsError):
                write_report([goal()], target)


if __name__ == "__main__":
    unittest.main()
