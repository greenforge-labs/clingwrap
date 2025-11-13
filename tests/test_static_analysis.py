"""Tests for clingwrap static analysis functionality."""

import importlib.util
from pathlib import Path

from clingwrap.static_info import StaticInformation


def get_launch_file_path(launch_file_name: str) -> Path:
    """
    Get the path to a launch file in the fixtures directory.

    Args:
        launch_file_name: Name of the launch file in the fixtures directory

    Returns:
        Path to the launch file
    """
    fixtures_dir = Path(__file__).parent / "fixtures"
    return fixtures_dir / launch_file_name


def get_static_info(launch_file_path: Path) -> StaticInformation:
    """
    Load a launch file and return its static information.

    Args:
        launch_file_path: Path to the launch file

    Returns:
        StaticInformation from the launch file
    """
    # Load the launch file module
    spec = importlib.util.spec_from_file_location("test_launch", launch_file_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {launch_file_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # Get the launch description (which is a LaunchBuilder instance)
    launch_builder = module.generate_launch_description()

    # Get static information
    return launch_builder.get_static_information()


def test_basic_node():
    """Test static analysis of a basic node with parameters and remappings."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("basic_node.launch.py"))

    # Should have exactly one node
    assert len(static_info.nodes) == 1

    node = static_info.nodes[0]

    # Check basic node properties
    assert node.package == "test_package"
    assert node.executable == "test_executable"
    assert node.name == "test_node"
    assert node.namespace is None

    # Check parameters
    assert node.parameters is not None
    assert node.parameters["param1"] == "value1"
    assert node.parameters["param2"] == 42
    assert node.parameters_file is None

    # Check remappings
    assert node.remappings is not None
    assert len(node.remappings) == 2
    assert node.remappings["/input"] == "/remapped_input"
    assert node.remappings["/output"] == "/remapped_output"

    # Should have no composable nodes, containers, relays, or includes
    assert len(static_info.composable_node_containers) == 0
    assert len(static_info.topic_relays) == 0
    assert len(static_info.included_launch_files) == 0


def test_namespace_nodes():
    """Test static analysis of nodes in different namespaces."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("namespace_nodes.launch.py"))

    # Should have exactly 3 nodes
    assert len(static_info.nodes) == 3

    # Root node (no namespace)
    root_node = static_info.nodes[0]
    assert root_node.package == "root_package"
    assert root_node.executable == "root_node"
    assert root_node.namespace is None

    # Node in ns1
    ns1_node = static_info.nodes[1]
    assert ns1_node.package == "ns1_package"
    assert ns1_node.executable == "ns1_node"
    assert ns1_node.name == "node_in_ns1"
    assert ns1_node.namespace == "ns1"

    # Node in ns2
    ns2_node = static_info.nodes[2]
    assert ns2_node.package == "ns2_package"
    assert ns2_node.executable == "ns2_node"
    assert ns2_node.name == "node_in_ns2"
    assert ns2_node.namespace == "ns2"


def test_composable_nodes():
    """Test static analysis of composable nodes in a container."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("composable_nodes.launch.py"))

    # Should have exactly 1 container
    assert len(static_info.composable_node_containers) == 1

    container = static_info.composable_node_containers[0]

    # Check container properties
    assert container.name == "test_container"
    assert container.namespace == "container_ns"
    assert container.package == "rclcpp_components"
    # Default container type is SINGLE_THREAD_EACH which uses component_container_isolated
    assert container.executable == "component_container_isolated"

    # Should have 2 composable nodes in the container
    assert len(container.nodes) == 2

    # Check first composable node (camera)
    camera_node = container.nodes[0]
    assert camera_node.package == "camera_package"
    assert camera_node.plugin == "camera_package::CameraDriver"
    assert camera_node.name == "camera_node"
    # Composable nodes don't automatically inherit container namespace
    # They use the namespace from the namespace stack at creation time
    assert camera_node.namespace is None

    # Check camera parameters
    assert camera_node.parameters is not None
    assert camera_node.parameters["fps"] == 30
    assert camera_node.parameters["resolution"] == "1920x1080"
    assert camera_node.parameters_file is None

    # Check camera remappings
    assert camera_node.remappings is not None
    assert len(camera_node.remappings) == 1
    assert camera_node.remappings["/image_raw"] == "/camera/image_raw"

    # Check second composable node (processor)
    processor_node = container.nodes[1]
    assert processor_node.package == "processing_package"
    assert processor_node.plugin == "processing_package::ImageProcessor"
    assert processor_node.name == "processor_node"
    assert processor_node.namespace is None

    # Check processor parameters
    assert processor_node.parameters is not None
    assert processor_node.parameters["algorithm"] == "edge_detection"
    assert processor_node.parameters_file is None

    # Check get_all_composable_nodes helper
    all_composable = static_info.get_all_composable_nodes()
    assert len(all_composable) == 2
    assert all_composable[0] == camera_node
    assert all_composable[1] == processor_node

    # Should have no regular nodes, relays, or includes
    assert len(static_info.nodes) == 0
    assert len(static_info.topic_relays) == 0
    assert len(static_info.included_launch_files) == 0


def test_topic_relays():
    """Test static analysis of topic relays and throttles."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("topic_relays.launch.py"))

    # Should have 3 topic relays (2 relays + 1 throttle)
    assert len(static_info.topic_relays) == 3

    # Regular relay
    relay1 = static_info.topic_relays[0]
    assert relay1.from_topic == "/input/camera"
    assert relay1.to_topic == "/output/camera"
    assert relay1.relay_type == "relay"
    assert relay1.lazy is True
    assert relay1.namespace is None

    # Throttle
    throttle = static_info.topic_relays[1]
    assert throttle.from_topic == "/sensors/lidar"
    assert throttle.to_topic == "/sensors/lidar/throttled"
    assert throttle.relay_type == "throttle"
    assert throttle.lazy is False
    assert throttle.rate == 10.0
    assert throttle.namespace is None

    # Relay in namespace
    relay2 = static_info.topic_relays[2]
    assert relay2.from_topic == "/cmd_vel"
    assert relay2.to_topic == "/cmd_vel_safe"
    assert relay2.relay_type == "relay"
    assert relay2.lazy is False
    assert relay2.namespace == "robot1"

    # Topic relays create composable nodes, so we should have containers
    assert len(static_info.composable_node_containers) == 3


def test_nested_namespaces():
    """Test static analysis of nodes in nested namespaces."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("nested_namespaces.launch.py"))

    # Should have exactly 6 nodes
    assert len(static_info.nodes) == 6

    # Root level node (before any namespace)
    assert static_info.nodes[0].name == "root_node"
    assert static_info.nodes[0].namespace is None

    # robot1 namespace
    assert static_info.nodes[1].name == "robot_node"
    assert static_info.nodes[1].namespace == "robot1"

    # robot1/sensors namespace
    assert static_info.nodes[2].name == "camera"
    assert static_info.nodes[2].namespace == "robot1/sensors"

    # robot1/sensors/stereo namespace (triple nested)
    assert static_info.nodes[3].name == "left_camera"
    assert static_info.nodes[3].namespace == "robot1/sensors/stereo"

    # Back to robot1 namespace (after sensors context exited)
    assert static_info.nodes[4].name == "controller"
    assert static_info.nodes[4].namespace == "robot1"

    # Back to root level (after robot1 context exited)
    assert static_info.nodes[5].name == "monitor"
    assert static_info.nodes[5].namespace is None


def test_included_launch_files():
    """Test static analysis of included launch files."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("include_test.launch.py"))

    # Should have 2 regular nodes
    assert len(static_info.nodes) == 2

    main_node = static_info.nodes[0]
    assert main_node.name == "main_node"
    assert main_node.namespace is None

    final_node = static_info.nodes[1]
    assert final_node.name == "final_node"
    assert final_node.namespace is None

    # Should have 2 included launch files
    assert len(static_info.included_launch_files) == 2

    # First include (with arguments)
    include1 = static_info.included_launch_files[0]
    assert include1.package == "some_package"
    assert include1.launch_file == "some_launch.py"
    assert include1.directory == "launch"
    assert include1.launch_arguments["arg1"] == "value1"
    assert include1.launch_arguments["arg2"] == 42
    assert include1.namespace is None

    # Second include (in namespace, no user arguments but use_sim_time added automatically)
    include2 = static_info.included_launch_files[1]
    assert include2.package == "another_package"
    assert include2.launch_file == "another_launch.py"
    assert include2.directory == "bringup"
    # use_sim_time is automatically added to launch arguments
    assert "use_sim_time" in include2.launch_arguments
    assert include2.namespace == "included_ns"


def test_global_namespaces():
    """Test static analysis of nodes with global (absolute) namespaces."""
    static_info: StaticInformation = get_static_info(get_launch_file_path("global_namespaces.launch.py"))

    # Should have exactly 6 nodes
    assert len(static_info.nodes) == 6

    # Node in global namespace /robot1
    robot1_node = static_info.nodes[0]
    assert robot1_node.name == "robot1_node"
    assert robot1_node.namespace == "/robot1"

    # Node in /robot2/sensors (global + relative)
    robot2_sensor = static_info.nodes[1]
    assert robot2_sensor.name == "robot2_sensor"
    assert robot2_sensor.namespace == "/robot2/sensors"

    # Node in relative_ns (before global namespace override)
    relative_node = static_info.nodes[2]
    assert relative_node.name == "relative_node"
    assert relative_node.namespace == "relative_ns"

    # Node in /robot3 (global namespace replaces previous relative_ns)
    robot3_node = static_info.nodes[3]
    assert robot3_node.name == "robot3_node"
    assert robot3_node.namespace == "/robot3"

    # Node in /robot3/camera (global + relative, ignoring previous relative_ns)
    robot3_camera = static_info.nodes[4]
    assert robot3_camera.name == "robot3_camera"
    assert robot3_camera.namespace == "/robot3/camera"

    # Node in /robot5/actuators (last global /robot5 replaces /robot4/sensors)
    robot5_actuator = static_info.nodes[5]
    assert robot5_actuator.name == "robot5_actuator"
    assert robot5_actuator.namespace == "/robot5/actuators"
