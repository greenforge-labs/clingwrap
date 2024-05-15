from launch import LaunchDescription, SomeSubstitutionsType
from launch import actions as act
from launch_ros import actions as ros_act

from typing import Optional


class LaunchBuilder(LaunchDescription):
    def __init__(self):
        super().__init__()

    def log(self, msg: SomeSubstitutionsType):
        self.add_action(act.LogInfo(msg=msg))

    def node(self, package: str, executable: Optional[str] = None, **node_kwargs):
        if executable is None:
            executable = package

        if "emulate_tty" not in node_kwargs and "output" not in node_kwargs:
            # https://github.com/ros2/launch/issues/188
            node_kwargs["emulate_tty"] = True
            node_kwargs["output"] = "screen"

        self.add_action(ros_act.Node(package=package, executable=executable, **node_kwargs))
