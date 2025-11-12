"""Topic relays and throttles for testing."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Regular relay
    l.topic_relay(from_="/input/camera", to="/output/camera", lazy=True)

    # Throttle
    l.topic_throttle_hz(topic="/sensors/lidar", rate=10.0, lazy=False)

    # Relay in namespace
    with l.namespace("robot1"):
        l.topic_relay(from_="/cmd_vel", to="/cmd_vel_safe", lazy=False)

    return l
