"""Target launch file to be included by include_test.launch.py."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Simple nodes in the included launch file
    l.node("included_package", "included_executable", name="included_node")

    return l
