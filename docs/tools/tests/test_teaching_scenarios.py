import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teaching_scenarios import SCENARIOS, truth_robot, world_sdf

class ScenarioTest(unittest.TestCase):
    def test_fixed_worlds_have_physical_bounds_and_unique_names(self):
        for name in SCENARIOS:
            root = ET.fromstring(world_sdf(name))
            models = root.findall("world/model")
            names = [m.attrib["name"] for m in models]
            self.assertEqual(len(names), len(set(names)))
            self.assertTrue({"floor", "north", "south", "east", "west"} <= set(names))
            for model in models:
                self.assertIsNotNone(model.find("link/collision/geometry/box/size"))

    def test_world_generation_is_repeatable(self):
        self.assertEqual(world_sdf("narrow_passage"), world_sdf("narrow_passage"))
        self.assertNotEqual(world_sdf("straight"), world_sdf("blocked"))

    def test_truth_is_separate_from_control_tf(self):
        model = truth_robot("<sdf><model name='robot'></model></sdf>")
        root = ET.fromstring(model)
        plugin = root.find("model/plugin")
        self.assertEqual(plugin.findtext("odom_topic"), "/teaching/ground_truth")
        self.assertNotEqual(plugin.findtext("tf_topic"), "/tf")

    def test_experiment_entry_points_parse_before_launch(self):
        import ast
        root = Path(__file__).resolve().parents[3]
        for name in ("docs/tools/run_navigation_lesson.py", "docs/tools/run_lesson_suite.py",
                     "docs/tools/plot_navigation_report.py",
                     "src/mini_slam/scripts/automated_session_test.py"):
            path = root / name
            ast.parse(path.read_text(), filename=str(path))

    def test_gui_close_never_targets_a_different_partition(self):
        from unittest.mock import patch
        from owned_gui import close_gazebo_gui
        with self.assertRaises(ValueError):
            close_gazebo_gui("")
        tree = '0x600012 "Gazebo Sim": ("gz-sim-gui" "Gazebo GUI") 1000x800+0+0'
        with patch.dict("os.environ", {"DISPLAY": ":0"}), \
             patch("owned_gui.subprocess.check_output", side_effect=[tree, "_NET_WM_PID = 12345"]), \
             patch("owned_gui.Path.read_bytes", return_value=b"GZ_PARTITION=someone_else"), \
             patch("owned_gui.ctypes.CDLL") as library:
            self.assertEqual(close_gazebo_gui("teaching_123456789a"), 0)
            library.assert_not_called()

    def test_malformed_robot_is_rejected(self):
        with self.assertRaises(ValueError):
            truth_robot("<sdf/>")

if __name__ == "__main__":
    unittest.main()
