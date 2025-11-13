"""Nodes with global (absolute) namespaces for testing."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Node in a global namespace
    with l.namespace("/robot1"):
        l.node("test_package", "test_node", name="robot1_node")

    # Global namespace followed by relative namespace
    with l.namespace("/robot2"):
        with l.namespace("sensors"):
            l.node("test_package", "test_node", name="robot2_sensor")

    # Relative namespace, then global namespace (global should replace)
    with l.namespace("relative_ns"):
        l.node("test_package", "test_node", name="relative_node")
        with l.namespace("/robot3"):
            l.node("test_package", "test_node", name="robot3_node")
            with l.namespace("camera"):
                l.node("test_package", "test_node", name="robot3_camera")

    # Multiple global namespaces (last one should win)
    with l.namespace("/robot4"):
        with l.namespace("sensors"):
            with l.namespace("/robot5"):
                with l.namespace("actuators"):
                    l.node("test_package", "test_node", name="robot5_actuator")

    return l
