#!/usr/bin/env python3
"""Safety regressions for uncertain and explicitly rejected map loads."""
import contextlib
import os
import pty
import select
import signal
import subprocess
import termios
import time
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


class ConsoleTerminalTest(unittest.TestCase):
    def test_ready_banner_means_single_keys_are_already_enabled(self):
        master, slave = pty.openpty()
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID="94", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
        script = Path(__file__).resolve().parents[2] / "scripts/mapping_console.py"
        process = subprocess.Popen(
            [sys.executable, str(script)], stdin=slave, stdout=slave, stderr=slave,
            env=environment, start_new_session=True)
        os.close(slave)
        output = ""
        def wait_for(text):
            nonlocal output
            end = time.monotonic() + 15
            while time.monotonic() < end:
                if select.select([master], [], [], 0.1)[0]:
                    output += os.read(master, 8192).decode(errors="replace")
                    if text in output:
                        return
                if process.poll() is not None:
                    break
            self.fail(f"Missing {text}: {output}")
        try:
            wait_for("Console ready")
            self.assertFalse(termios.tcgetattr(master)[3] & termios.ICANON)
            os.write(master, b"3")  # No newline: must work immediately after readiness.
            wait_for("Map name")
            os.write(master, b"\n")  # Empty name rejects before any map service or write.
            wait_for("No map name entered")
            os.write(master, b"q")
            self.assertEqual(process.wait(timeout=10), 0)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            os.close(master)


if __name__ == "__main__":
    unittest.main()
