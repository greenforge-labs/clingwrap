"""Composable nodes in container for testing."""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Container with multiple composable nodes
    with l.composable_node_container(name="test_container", namespace="container_ns"):
        l.composable_node(
            "camera_package",
            "camera_package::CameraDriver",
            name="camera_node",
            parameters={"fps": 30, "resolution": "1920x1080"},
            remappings={"/image_raw": "/camera/image_raw"},
        )

        l.composable_node(
            "processing_package",
            "processing_package::ImageProcessor",
            name="processor_node",
            parameters={"algorithm": "edge_detection"},
        )

    return l
