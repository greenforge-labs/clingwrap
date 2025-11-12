"""Basic node launch file for testing."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        "test_package",
        "test_executable",
        name="test_node",
        parameters={"param1": "value1", "param2": 42},
        remappings={"/input": "/remapped_input", "/output": "/remapped_output"},
    )

    return l
