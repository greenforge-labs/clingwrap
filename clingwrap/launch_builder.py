from contextlib import contextmanager

from launch import Action, LaunchDescription, SomeSubstitutionsType
from launch import actions as act
from launch import launch_description_sources
from launch import substitutions as sub
from launch_ros import actions as ros_act
from launch_ros import descriptions as desc
from launch_ros.parameters_type import SomeParameterFile, SomeParameterName, SomeParameters, SomeParameterValue
from launch_ros.remap_rule_type import SomeRemapRules

from .action_list import ActionList, ActionListImpl
from .launch_helpers import ContainerType, find_file

from typing import Generator, Optional, Text

ComposableNodeList = list[desc.ComposableNode]


def generate_parameter_list(
    parameters: Optional[dict[SomeParameterName, SomeParameterValue]] = None,
    parameters_file: Optional[SomeParameterFile] = None,
) -> SomeParameters:
    parameters_list = []
    if parameters_file is not None:
        parameters_list.append(parameters_file)
    if parameters is not None:
        parameters_list.append(parameters)
    return parameters_list


def generate_remappings_list(
    remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None
) -> SomeRemapRules:
    if remappings is not None:
        return list(remappings.items())
    return []


class LaunchBuilder(LaunchDescription):
    _action_list: ActionList
    _composable_node_list: Optional[ComposableNodeList] = None

    def __init__(self):
        self._action_list = self
        super().__init__()

    @property
    def actions(self) -> list[Action]:
        return [e for e in self.entities if isinstance(e, Action)]

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
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **node_kwargs,
    ):
        if executable is None:
            executable = package

        if "emulate_tty" not in node_kwargs and "output" not in node_kwargs:
            # https://github.com/ros2/launch/issues/188
            node_kwargs["emulate_tty"] = True
            node_kwargs["output"] = "screen"

        self._action_list.add_action(
            ros_act.Node(
                package=package,
                executable=executable,
                parameters=generate_parameter_list(parameters, parameters_file),
                remappings=generate_remappings_list(remappings),
                **node_kwargs,
            )
        )

    @contextmanager
    def composable_node_container(
        self,
        name: str,
        container_type: ContainerType = ContainerType.SINGLE_THREAD_EACH,
        parameters: Optional[dict[SomeParameterName, SomeParameterValue]] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **container_kwargs,
    ) -> Generator[None, None, None]:
        if self._composable_node_list is not None:
            raise ValueError("A composable node container is already in context!")

        self._composable_node_list = []

        yield

        executable, args = container_type.value
        container_kwargs["arguments"] = container_kwargs.get("arguments", []) + args

        self._action_list.add_action(
            ros_act.ComposableNodeContainer(
                package="rclcpp_components",
                executable=executable,
                name=name,
                namespace="",
                composable_node_descriptions=self._composable_node_list,
                parameters=generate_parameter_list(parameters, parameters_file),
                remappings=generate_remappings_list(remappings),
                **container_kwargs,
            )
        )

        self._composable_node_list = None

    def composable_node(
        self,
        package: str,
        plugin: str,
        parameters: Optional[dict[SomeParameterName, SomeParameterValue]] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **node_kwargs,
    ):
        if self._composable_node_list is None:
            raise ValueError("There is no composable node container in context!")

        self._composable_node_list.append(
            desc.ComposableNode(
                package=package,
                plugin=plugin,
                parameters=generate_parameter_list(parameters, parameters_file),
                remappings=generate_remappings_list(remappings),
                **node_kwargs,
            )
        )

    def topic_relay(self, from_: str, to: str, lazy: bool = True):
        friendly_from = from_.replace("/", "_").strip("_")
        friendly_to = to.replace("/", "_").strip("_")

        self.node(
            "topic_tools",
            "relay",
            name=f"relay_{friendly_from}_{friendly_to}",
            parameters={"input_topic": from_, "output_topic": to, "lazy": lazy},
        )
