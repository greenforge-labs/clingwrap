"""Launch file that includes another launch file for testing."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Node before include
    l.node("main_package", "main_executable", name="main_node")

    # Include a launch file with arguments
    l.include_launch_py(
        package="some_package",
        launch_file="some_launch.py",
        launch_arguments={"arg1": "value1", "arg2": 42},
        directory="launch",
    )

    # Include in a namespace
    with l.namespace("included_ns"):
        l.include_launch_py(
            package="another_package",
            launch_file="another_launch.py",
            directory="bringup",
        )

    # Node after include
    l.node("final_package", "final_executable", name="final_node")

    return l
