# clingwrap
_Launch? ... Lunch? ... Who knows._

A Pythonic ROS2 launch wrapper that makes launch files cleaner, more intuitive, and statically analysable.

## Overview

Clingwrap simplifies ROS2 launch file creation by providing:
- Cleaner, more readable syntax
- Context managers for namespaces and composable node containers
- Automatic `use_sim_time` parameter injection
- Static analysis capabilities to extract launch file structure


## Quick Start

Import `clingwrap` as `cw` and use `LaunchBuilder` to generate a launch description.

```python
from launch import substitutions as sub

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    robot_description_path = l.declare_arg("robot_description_path")
    robot_description_urdf = sub.Command(["xacro ", robot_description_path])

    l.node("robot_state_publisher", parameters={"robot_description": cw.as_str_param(robot_description_urdf)})

    return l
```

See [launch_builder.py](./clingwrap/launch_builder.py) and [launch_helpers.py](./clingwrap/launch_helpers.py) and [param_helpers](./clingwrap/param_helpers.py) for the full API.

## Features & Examples

### 1. Launch Arguments

#### Declare Generic Arguments
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Declare a string argument with a default value
    robot_name = l.declare_arg("robot_name", default_value="robot1")

    # Use the argument in node configuration
    l.node(
        "my_package",
        "my_node",
        name=robot_name,
        parameters={"robot_name": robot_name}
    )

    return l
```

#### Declare Boolean Arguments
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Declare a boolean argument (automatically validates choices)
    enable_feature = l.declare_bool_arg("enable_feature", default_value=True)

    l.node(
        "my_package",
        "my_node",
        parameters={"feature_enabled": enable_feature}
    )

    return l
```

#### Automatic `use_sim_time`
`clingwrap` automatically adds `use_sim_time` as a launch argument to all launch descriptions and passes the value into any nodes added to the launch description. The value of the parameter (as a substitution) can be accessed via `l.use_sim_time`.

### 2. Node Management

#### Basic Node
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Simple node with package name as executable
    l.node("camera_driver")

    # Node with explicit executable
    l.node("my_package", "my_executable", name="my_node")

    return l
```

#### Node with Parameters
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Using parameter dict
    l.node(
        "camera_driver",
        parameters={
            "fps": 30,
            "resolution": "1920x1080",
            "auto_exposure": True
        }
    )

    # Using parameter file
    l.node(
        "camera_driver",
        parameters_file=cw.pkg_file("my_package", "config", "camera.yaml")
    )

    # Using both
    l.node(
        "camera_driver",
        parameters={"fps": 60},  # Override file values
        parameters_file=cw.pkg_file("my_package", "config", "camera.yaml")
    )

    return l
```

#### Node with Remappings
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        "image_processor",
        remappings={
            "image_raw": "/camera/image_raw",
            "image_processed": "/output/processed"
        }
    )

    return l
```

#### Node with Log Level
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        "debug_node",
        log_level=cw.LogLevel.DEBUG
    )

    l.node(
        "production_node",
        log_level=cw.LogLevel.WARN
    )

    return l
```

### 3. Namespaces

#### Single Namespace
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Node at root level
    l.node("monitor", name="global_monitor")

    # Nodes in namespace
    with l.namespace("robot1"):
        l.node("driver", name="driver")
        l.node("controller", name="controller")

    # Back to root level
    l.node("analyzer", name="global_analyzer")

    return l
```

#### Nested Namespaces
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.namespace("robot1"):
        l.node("base_controller", name="controller")

        with l.namespace("sensors"):
            l.node("camera_driver", name="camera")
            l.node("lidar_driver", name="lidar")

            with l.namespace("imu"):
                l.node("imu_driver", name="main_imu")

        # Back to robot1 namespace
        l.node("planner", name="path_planner")

    return l
```

#### Global (Absolute) Namespaces
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.namespace("relative_ns"):
        l.node("pkg1", name="relative_node")  # namespace: "relative_ns"

        # Global namespace replaces relative context
        with l.namespace("/robot1"):
            l.node("pkg2", name="robot1_node")  # namespace: "/robot1"

            with l.namespace("sensors"):
                l.node("pkg3", name="sensor")  # namespace: "/robot1/sensors"

    return l
```

### 4. Composable Nodes

#### Basic Container with Composable Nodes
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.composable_node_container(name="my_container"):
        l.composable_node(
            "camera_package",
            "camera_package::CameraDriver",
            name="camera",
            parameters={"fps": 30}
        )

        l.composable_node(
            "processing_package",
            "processing_package::ImageProcessor",
            name="processor",
            parameters={"algorithm": "edge_detection"}
        )

    return l
```

#### Container Types
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Single-threaded executor (all components share one thread)
    with l.composable_node_container(
        name="single_thread_container",
        container_type=cw.ContainerType.SINGLE_THREAD
    ):
        l.composable_node("pkg1", "pkg1::Node1")
        l.composable_node("pkg2", "pkg2::Node2")

    # Isolated single-threaded (each component has its own thread)
    with l.composable_node_container(
        name="isolated_container",
        container_type=cw.ContainerType.SINGLE_THREAD_EACH
    ):
        l.composable_node("pkg3", "pkg3::Node3")
        l.composable_node("pkg4", "pkg4::Node4")

    # Multi-threaded (all components share multi-threaded executor)
    with l.composable_node_container(
        name="mt_container",
        container_type=cw.ContainerType.MULTI_THREAD
    ):
        l.composable_node("pkg5", "pkg5::Node5")
        l.composable_node("pkg6", "pkg6::Node6")

    # Isolated multi-threaded (each has own multi-threaded executor)
    with l.composable_node_container(
        name="mt_isolated_container",
        container_type=cw.ContainerType.MULTI_THREAD_EACH
    ):
        l.composable_node("pkg7", "pkg7::Node7")
        l.composable_node("pkg8", "pkg8::Node8")

    return l
```

### 5. Topic Tools

#### Topic Relay
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Relay from one topic to another
    l.topic_relay("/sensor/raw", "/output/relay")

    # Non-lazy relay (always relays, even with no subscribers)
    l.topic_relay("/important/topic", "/backup/topic", lazy=False)

    # Inside existing composable node container
    with l.composable_node_container(name="my_container"):
        l.composable_node("my_pkg", "my_pkg::MyNode")
        l.topic_relay("/topic_a", "/topic_b")  # Added to same container

    return l
```

#### Topic Throttle
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Throttle to 10 Hz
    l.topic_throttle_hz("/camera/image_raw", rate=10.0)
    # Creates: /camera/image_raw/throttled

    # Include Hz in output topic name
    l.topic_throttle_hz(
        "/lidar/points",
        rate=5.0,
        include_hz_in_output_topic=True
    )
    # Creates: /lidar/points/throttled/hz_5_0

    # Non-lazy throttle
    l.topic_throttle_hz("/critical/data", rate=1.0, lazy=False)

    return l
```

### 6. Launch File Inclusion

#### Include Other Launch Files
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Include launch file from another package
    l.include_launch_py(
        package="navigation_package",
        launch_file="navigation.launch.py"
    )

    # Include with arguments
    l.include_launch_py(
        package="sensor_package",
        launch_file="sensors.launch.py",
        launch_arguments={
            "sensor_name": "main_camera",
            "fps": 30
        }
    )

    # Include from custom directory
    l.include_launch_py(
        package="my_package",
        launch_file="custom.launch.py",
        directory="bringup"  # Instead of default "launch"
    )

    # Include in namespace
    with l.namespace("robot1"):
        l.include_launch_py(
            package="control_package",
            launch_file="controller.launch.py"
        )

    return l
```

### 7. Logging

```python
import clingwrap as cw
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    l = cw.LaunchBuilder()

    robot_name = l.declare_arg("robot_name", default_value="robot1")

    # Simple log message
    l.log("Starting robot system...")

    # Log with substitutions
    l.log(["Launching robot: ", robot_name])

    l.node("my_package", "my_node", name=robot_name)

    return l
```

### 9. Static Analysis

Extract structured information from launch files for documentation, validation, or tooling.

```python
# Load the launch file
spec = importlib.util.spec_from_file_location("launch_file", path/to/launch/file.launch.py)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
launch_description = module.generate_launch_description()

# Get static information
info = launch_description.get_static_information()

# Access regular nodes
for node in info.nodes:
    print(f"Node: {node.package}/{node.executable} in namespace {node.namespace}")

# Access composable node containers
for container in info.composable_node_containers:
    print(f"Container: {container.name}")
    for comp_node in container.nodes:
        print(f"  - Composable: {comp_node.package}::{comp_node.plugin}")

# Access all composable nodes as flat list
all_composable = info.get_all_composable_nodes()

# Access included launch files
for include in info.included_launch_files:
    print(f"Includes: {include.package}/{include.launch_file}")

```

### 10. Helper Functions

#### Package File Paths
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Get path to file in package
    config_file = cw.pkg_file("my_package", "config", "params.yaml")
    urdf_file = cw.pkg_file("robot_description", "urdf", "robot.urdf")

    l.node(
        "robot_state_publisher",
        parameters=[{"robot_description": urdf_file}]
    )

    return l
```

#### String Parameters
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Force parameter to be interpreted as string
    l.node(
        "my_node",
        parameters={
            "numeric_string": cw.as_str_param(123),  # "123" not 123
            "bool_string": cw.as_str_param(True)     # "True" not true
        }
    )

    return l
```

#### Hide Topics
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        "my_node",
        remappings={
            # debug_topic becomes _hidden_debug_topic
            **cw.remap_hidden("debug_topic")
        }
    )

    return l
```

#### Covariance Matrix Helper
```python
import clingwrap as cw

def generate_launch_description():
    l = cw.LaunchBuilder()

    # Create diagonal covariance for x, y, z
    position_cov = cw.cov_xyz_diag(0.1, 0.1, 0.2)
    # Returns: [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.2]

    l.node(
        "localization_node",
        parameters={"initial_pose_covariance": position_cov}
    )

    return l
```

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.
