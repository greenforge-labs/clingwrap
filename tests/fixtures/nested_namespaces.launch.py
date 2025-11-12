"""Nested namespaces for testing namespace stack tracking."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Root level node
    l.node("root_package", "root_executable", name="root_node")

    # Single level namespace
    with l.namespace("robot1"):
        l.node("robot_package", "robot_executable", name="robot_node")

        # Nested namespace
        with l.namespace("sensors"):
            l.node("sensor_package", "camera_executable", name="camera")

            # Triple nested namespace
            with l.namespace("stereo"):
                l.node("stereo_package", "stereo_executable", name="left_camera")

        # Back to robot1 namespace (sensors context exited)
        l.node("robot_package", "controller_executable", name="controller")

    # Back to root level
    l.node("root_package", "monitor_executable", name="monitor")

    return l
