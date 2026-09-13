"""Check the ROS descriptions used to load topic throttles."""

from unittest.mock import patch

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.utilities import evaluate_parameters
import pytest

from clingwrap import LaunchBuilder


@pytest.mark.parametrize("existing_container", [False, True])
@pytest.mark.parametrize("include_rate", [False, True])
@pytest.mark.parametrize("rate, suffix", [(2.0, "2_0"), (0.5, "0_5"), (2, "2")])
def test_throttle_remappings(existing_container, include_rate, rate, suffix):
    builder = LaunchBuilder()
    topic = "/perception/lidar/points/combined"
    with patch("clingwrap.launch_builder.ros_act.ComposableNodeContainer", wraps=ComposableNodeContainer) as container:
        if existing_container:
            with builder.composable_node_container("perception"):
                builder.topic_throttle_hz(topic, rate, include_hz_in_output_topic=include_rate)
        else:
            builder.topic_throttle_hz(topic, rate, include_hz_in_output_topic=include_rate)

    container.assert_called_once()
    descriptions = container.call_args.kwargs["composable_node_descriptions"]
    assert len(descriptions) == 1
    node = descriptions[0]
    context = LaunchContext()
    context.launch_configurations["use_sim_time"] = "false"
    resolve = lambda value: perform_substitutions(context, value)
    assert resolve(node.node_plugin) == "topic_tools::ThrottleNode"
    remappings = {resolve(source): resolve(target) for source, target in node.remappings}
    expected_output = topic + "/throttled" + (f"/hz_{suffix}" if include_rate else "")
    assert remappings == {"in": topic, "out": expected_output}
    parameters = evaluate_parameters(context, node.parameters)[0]
    assert parameters["msgs_per_sec"] == float(rate)
    assert parameters["throttle_type"] == "messages"
    assert parameters["lazy"] is True
