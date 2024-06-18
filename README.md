# clingwrap
_Launch? ... Lunch? ... Who knows._

A ROS2 lunch<sub>launch</sub> wrapper to simplify writing python launch files.

## Usage
Import `clingwrap` as `cw` and use `LaunchBuilder` to generate a launch description in a descriptive way.

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

See [launch_builder.py](./clingwrap/launch_builder.py) and [launch_helpers.py](./clingwrap/launch_helpers.py) for the full API. A non-exhaustive list of use cases is presented below.

### Nodes
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node("robot_state_publisher", parameters={"robot_description": "..."})

    return l
```

### Namespace
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.namespace("my_namespace"):
        l.node("joint_state_publisher_gui")

    return l
```

### Launch Arguments
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    my_argument = l.declare_arg("my_argument", default_value="hello")
    my_bool_argument = l.declare_bool_arg("my_bool_argument")

    # use my_argument and my_bool_argument as value substitutions
    ...

    return l
```

### `use_sim_time` Launch Argument
`clingwrap` automatically adds `use_sim_time` as a launch argument to all launch descriptions and passes the value into any nodes added to the launch description. The value of the parameter (as a substitution) can be accessed via `l.use_sim_time`.

### Including Other Launch Files
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    l.include_launch_py("some_package", "my_launch_file.launch.py")

    return l
```

### Composable Nodes
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.composable_node_container("my_container"):
        l.composable_node(
            "some_package",
            "some_package::ComposableNode",
        )

        l.composable_node(
            "some_other_package",
            "some_other_package::ComposableNode2",
        )

    return l
```

### Remapping Actions
There is an open github issue for this (https://github.com/ros2/ros2/issues/1312). In the meantime:
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    l.node(
        "some_package",
        "some_executable",
        remappings={
            "a_different_remap": "remap_to_this",
            **cw.remap_action("some_action", "different_action"),
        },
    )
```

### Log Info
```python
def generate_launch_description():
    l = cw.LaunchBuilder()

    l.log("hello world!")

    return l
```
