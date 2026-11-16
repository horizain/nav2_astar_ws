#!/usr/bin/env python3
"""Safety regressions for uncertain and explicitly rejected map loads."""
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from mapping_console import Console, ServiceOutcomeUnknown  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402


class FakeConsole:
    operation = Console.operation
    localize = Console.localize

    def __init__(self, error):
        self.manual = True
        self.state_uncertain = False
        self.saved = Path("/maps/original")
        self.error = error
        self.calls = []

    def stop(self):
        self.calls.append(("stop",))

    def prompt_prefix(self, _):
        return Path("/maps/new")

    def load_static_map(self, prefix):
        self.calls.append(("map", str(prefix)))
        return object()

    def set_parameter(self, name, value):
        self.calls.append(("parameter", name, value))

    def graph(self, action):
        self.calls.append(("graph", action))
        raise self.error

    def mapping(self):
        self.calls.append(("mapping",))

    def cancel_navigation(self):
        self.calls.append(("cancel",))


class ConsoleFailureTests(unittest.TestCase):
    @patch.object(Path, "is_file", return_value=True)
    def test_unknown_graph_result_keeps_manual_disabled(self, _):
        console = FakeConsole(ServiceOutcomeUnknown("load outcome unknown"))
        with contextlib.redirect_stdout(io.StringIO()):
            console.operation("4")
            before = list(console.calls)
            console.operation("1")
        self.assertFalse(console.manual)
        self.assertTrue(console.state_uncertain)
        self.assertNotIn(("parameter", "mode", "mapping"), console.calls)
        self.assertEqual(console.calls, before)

    @patch.object(Path, "is_file", return_value=True)
    def test_explicit_graph_failure_restores_previous_state(self, _):
        console = FakeConsole(RuntimeError("invalid graph"))
        with contextlib.redirect_stdout(io.StringIO()):
            console.operation("4")
        self.assertTrue(console.manual)
        self.assertFalse(console.state_uncertain)
        self.assertIn(("map", "/maps/original"), console.calls)
        self.assertIn(("parameter", "graph_path", "/maps/original.mmpg"),
                      console.calls)
        self.assertIn(("parameter", "mode", "mapping"), console.calls)

    @patch.object(Path, "is_file", return_value=True)
    def test_unknown_map_rollback_keeps_manual_disabled(self, _):
        console = FakeConsole(RuntimeError("invalid graph"))
        def load_map(prefix):
            if prefix == Path("/maps/original"):
                raise ServiceOutcomeUnknown("rollback timeout")
            return object()
        console.load_static_map = load_map
        with contextlib.redirect_stdout(io.StringIO()):
            console.operation("4")
        self.assertFalse(console.manual)
        self.assertTrue(console.state_uncertain)
        self.assertNotIn(("parameter", "mode", "mapping"), console.calls)

    @patch.object(Path, "is_file", return_value=True)
    def test_unknown_mode_rollback_keeps_manual_disabled(self, _):
        console = FakeConsole(RuntimeError("invalid graph"))
        def set_parameter(name, value):
            console.calls.append(("parameter", name, value))
            if name == "mode" and value == "mapping":
                raise ServiceOutcomeUnknown("rollback timeout")
        console.set_parameter = set_parameter
        with contextlib.redirect_stdout(io.StringIO()):
            console.operation("4")
        self.assertFalse(console.manual)
        self.assertTrue(console.state_uncertain)

    def test_repeated_odom_stamp_cannot_confirm_stop(self):
        # 持续收到速度为零但时间戳不前进的消息，不能误认为设备已稳定停止。
        class FakeCancel:
            cancel_navigation = Console.cancel_navigation

            def __init__(self):
                self.pub = Mock()
                self.action_status = {"navigate_to_pose": {}, "follow_path": {}}
                self.odom_velocity = None

            def call(self, *_args):
                response = CancelGoal.Response()
                response.return_code = CancelGoal.Response.ERROR_NONE
                return response

        class Clock:
            value = 0.0
            def tick(self):
                self.value += 0.05
                return self.value

        console, clock = FakeCancel(), Clock()
        def spin_once(*_args, **_kwargs):
            console.odom_velocity = (clock.value, 12345, 0.0, 0.0, 0.0)

        with patch("mapping_console.time.monotonic", side_effect=clock.tick), \
             patch("mapping_console.rclpy.spin_once", side_effect=spin_once):
            with self.assertRaises(ServiceOutcomeUnknown):
                console.cancel_navigation()

    @patch.object(Path, "is_file", return_value=True)
    def test_unknown_static_map_result_keeps_manual_disabled(self, _):
        console = FakeConsole(RuntimeError("unused"))
        def unknown(_prefix):
            raise ServiceOutcomeUnknown("map service timeout")
        console.load_static_map = unknown
        with contextlib.redirect_stdout(io.StringIO()):
            console.operation("4")
        self.assertFalse(console.manual)
        self.assertTrue(console.state_uncertain)
        self.assertNotIn(("parameter", "mode", "mapping"), console.calls)


if __name__ == "__main__":
    unittest.main()
