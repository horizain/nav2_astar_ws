"""Deterministic geometry and requests for the teaching navigation experiments."""
import math
import xml.etree.ElementTree as ET

SCENARIOS = {
    "straight": {"goals": [[1.6, 0.0, 0.0]], "boxes": [], "expected": "succeeded"},
    "arc": {"goals": [[1.2, 1.2, math.pi / 2]], "boxes": [], "expected": "succeeded"},
    "right_angle": {"goals": [[1.4, 0.0, math.pi / 2], [1.4, 1.4, math.pi / 2]],
                    "boxes": [], "expected": "succeeded"},
    "narrow_passage": {"goals": [[2.0, 0.0, 0.0]],
                       "boxes": [[1.4, 0.5, 2.0, 0.1], [1.4, -0.5, 2.0, 0.1]],
                       "expected": "succeeded"},
    "blocked": {"goals": [[2.0, 0.0, 0.0]], "boxes": [[1.0, 0.0, 0.15, 6.0]],
                "expected": "aborted_or_rejected"},
    "unreachable": {"goals": [[3.5, 0.0, 0.0]], "boxes": [],
                    "expected": "aborted_or_rejected"},
    "cancel_replan": {"goals": [[1.8, 0.0, 0.0], [0.5, 1.2, math.pi / 2]],
                      "boxes": [], "expected": "canceled_then_succeeded"},
}


def world_sdf(name):
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {name}")
    root = ET.Element("sdf", version="1.6")
    world = ET.SubElement(root, "world", name="default")
    for filename, system in [
        ("physics", "Physics"), ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors"), ("imu", "Imu"),
    ]:
        plugin = ET.SubElement(world, "plugin", filename=f"gz-sim-{filename}-system",
                               name=f"gz::sim::systems::{system}")
        if system == "Sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    physics = ET.SubElement(world, "physics", name="teaching_3ms", type="ode")
    ET.SubElement(physics, "max_step_size").text = "0.003"
    ET.SubElement(physics, "real_time_factor").text = "1"
    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 8 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    def box(identifier, x, y, sx, sy, height=1.0, z=None):
        model = ET.SubElement(world, "model", name=identifier)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{x} {y} {height / 2 if z is None else z} 0 0 0"
        link = ET.SubElement(model, "link", name="link")
        for kind in ("collision", "visual"):
            element = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(element, "geometry")
            shape = ET.SubElement(geometry, "box")
            ET.SubElement(shape, "size").text = f"{sx} {sy} {height}"
            if kind == "visual":
                material = ET.SubElement(element, "material")
                ET.SubElement(material, "ambient").text = "0.65 0.7 0.75 1"

    box("floor", 0, 0, 12, 12, height=0.1, z=-0.05)
    for args in [("east", 3, 0, 0.1, 6.1), ("west", -3, 0, 0.1, 6.1),
                 ("north", 0, 3, 6.1, 0.1), ("south", 0, -3, 6.1, 0.1)]:
        box(*args)
    for i, geometry in enumerate(SCENARIOS[name]["boxes"]):
        box(f"lesson_obstacle_{i}", *geometry)
    return ET.tostring(root, encoding="unicode")


def truth_robot(source):
    if source.count("</model>") != 1:
        raise ValueError("Expected one robot model")
    plugin = """
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame><robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>/teaching/ground_truth</odom_topic><odom_publish_frequency>20</odom_publish_frequency>
      <dimensions>2</dimensions><gaussian_noise>0</gaussian_noise>
      <tf_topic>/teaching/ground_truth_tf_unused</tf_topic>
    </plugin>
    """
    return source.replace("</model>", plugin + "</model>")
