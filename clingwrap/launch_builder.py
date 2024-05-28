from contextlib import contextmanager

from launch import LaunchDescription, SomeSubstitutionsType
from launch import actions as act
from launch import launch_description_sources
from launch import substitutions as sub
from launch_ros import actions as ros_act
from launch_ros.parameters_type import SomeParameterFile, SomeParameterName, SomeParameterValue

from .action_list import ActionList, ActionListImpl
from .launch_helpers import find_file

from typing import Generator, Iterable, Optional, Text


class LaunchBuilder(LaunchDescription):
    _action_list: ActionList

    def __init__(self):
        self._action_list = self
        super().__init__()

    def declare_arg(self, name: Text, **arg_kwargs) -> sub.LaunchConfiguration:
        self._action_list.add_action(act.DeclareLaunchArgument(name, **arg_kwargs))
        return sub.LaunchConfiguration(name)

    def declare_bool_arg(self, name: Text, **arg_kwargs) -> sub.LaunchConfiguration:
        arg_kwargs["choices"] = ["true", "false", "True", "False"]
        return self.declare_arg(name, **arg_kwargs)

    def include_launch_py(
        self,
        package: str,
        launch_file: str,
        directory: str = "launch",
        launch_arguments: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **launch_kwargs,
    ):
        launch_file_path = find_file(package, directory, launch_file)
        launch_arguments_tuple = [(k, v) for k, v in launch_arguments.items()] if launch_arguments is not None else None

        self._action_list.add_action(
            act.IncludeLaunchDescription(
                launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments=launch_arguments_tuple,
                **launch_kwargs,
            )
        )

    @contextmanager
    def namespace(self, namespace: str) -> Generator[None, None, None]:
        previous_action_list = self._action_list
        self._action_list = ActionListImpl()
        yield
        previous_action_list.add_action(
            act.GroupAction([ros_act.PushRosNamespace(namespace=namespace)] + self._action_list.actions)
        )
        self._action_list = previous_action_list

    def log(self, msg: SomeSubstitutionsType):
        self._action_list.add_action(act.LogInfo(msg=msg))

    def node(
        self,
        package: str,
        executable: Optional[str] = None,
        parameters: Optional[dict[SomeParameterName, SomeParameterValue]] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[list[tuple[SomeSubstitutionsType, SomeSubstitutionsType]]] = None,
        **node_kwargs,
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

        self._action_list.add_action(
            ros_act.Node(
                package=package,
                executable=executable,
                parameters=parameters_list,
                remappings=remappings,
                **node_kwargs,
            )
        )
