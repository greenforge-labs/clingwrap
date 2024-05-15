from launch import LaunchDescription, SomeSubstitutionsType
from launch import actions as act
from launch_ros import actions as ros_act
from launch_ros.parameters_type import SomeParameterFile, SomeParameterName, SomeParameterValue

from typing import Optional


class LaunchBuilder(LaunchDescription):
    def __init__(self):
        super().__init__()

    def log(self, msg: SomeSubstitutionsType):
        self.add_action(act.LogInfo(msg=msg))

    def node(
        self,
        package: str,
        executable: Optional[str] = None,
        parameters: Optional[dict[SomeParameterName, SomeParameterValue]] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[list[tuple[SomeSubstitutionsType, SomeSubstitutionsType]]] = None,
        **node_kwargs
    ):
        if executable is None:
            executable = package

        parameters_list = []
        if parameters_file is not None:
            parameters_list.append(parameters_file)
        if parameters is not None:
            parameters_list.append(parameters)

        if "emulate_tty" not in node_kwargs and "output" not in node_kwargs:
            # https://github.com/ros2/launch/issues/188
            node_kwargs["emulate_tty"] = True
            node_kwargs["output"] = "screen"

        self.add_action(
            ros_act.Node(
                package=package,
                executable=executable,
                parameters=parameters_list,
                remappings=remappings,
                **node_kwargs,
            )
        )
