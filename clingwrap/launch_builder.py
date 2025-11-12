from contextlib import contextmanager

from launch import Action, LaunchContext, LaunchDescription, SomeSubstitutionsType
from launch import actions as act
from launch import launch_description_sources
from launch import substitutions as sub
from launch_ros import actions as ros_act
from launch_ros import descriptions as desc
from launch_ros.parameters_type import SomeParameterFile, SomeParameters, SomeParametersDict
from launch_ros.remap_rule_type import SomeRemapRules

from .action_list import ActionList, ActionListImpl
from .launch_helpers import ContainerType, LogLevel, pkg_file
from .static_analyzer import LaunchStaticAnalyzer
from .static_info import StaticInformation

from typing import Callable, Generator, Optional, Text

ComposableNodeList = list[desc.ComposableNode]


def generate_parameter_list(
    parameters: Optional[SomeParametersDict] = None,
    parameters_file: Optional[SomeParameterFile] = None,
) -> SomeParameters:
    parameters_list = []
    if parameters_file is not None:
        parameters_list.append(parameters_file)
    if parameters is not None:
        parameters_list.append(parameters)
    return parameters_list


def generate_remappings_list(
    remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
) -> SomeRemapRules:
    if remappings is not None:
        return list(remappings.items())
    return []


def add_log_level(kwargs: dict, log_level: Optional[LogLevel]) -> dict:
    if log_level is not None:
        if "ros_arguments" not in kwargs:
            kwargs["ros_arguments"] = []
        kwargs["ros_arguments"] += ["--log-level", log_level.value]
    return kwargs


class LaunchBuilder(LaunchDescription):
    _action_list: ActionList
    _composable_node_list: Optional[ComposableNodeList] = None

    def __init__(self):
        self._action_list = self
        super().__init__()
        self._use_sim_time = self.declare_bool_arg("use_sim_time", default_value=False)
        self._static_analyzer = LaunchStaticAnalyzer()

    def _add_sim_time(self, parameters: Optional[dict]) -> dict:
        if parameters is None:
            parameters = {}

        parameters["use_sim_time"] = self._use_sim_time
        return parameters

    @property
    def use_sim_time(self) -> sub.LaunchConfiguration:
        return self._use_sim_time

    @property
    def actions(self) -> list[Action]:
        return [e for e in self.entities if isinstance(e, Action)]

    @property
    def in_composable_node_context(self) -> bool:
        return self._composable_node_list is not None

    def declare_arg(self, name: Text, **arg_kwargs) -> sub.LaunchConfiguration:
        self._action_list.add_action(act.DeclareLaunchArgument(name, **arg_kwargs))
        return sub.LaunchConfiguration(name)

    def declare_bool_arg(self, name: Text, default_value: bool = False, **arg_kwargs) -> sub.LaunchConfiguration:
        arg_kwargs["choices"] = ["true", "false", "True", "False"]
        return self.declare_arg(name, default_value=str(default_value), **arg_kwargs)

    def include_launch_py(
        self,
        package: str,
        launch_file: str,
        launch_arguments: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        directory: str = "launch",
        **launch_kwargs,
    ):
        launch_file_path = pkg_file(package, directory, launch_file)

        launch_arguments = self._add_sim_time(launch_arguments)

        launch_arguments_tuple = [(k, v) for k, v in launch_arguments.items()] if launch_arguments is not None else None

        # Track the included launch file for static analysis
        self._static_analyzer.track_included_launch_file(
            package=package,
            launch_file=launch_file,
            directory=directory,
            launch_arguments=launch_arguments if launch_arguments else {},  # type: ignore
        )

        self._action_list.add_action(
            act.IncludeLaunchDescription(
                launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
                launch_arguments=launch_arguments_tuple,
                **launch_kwargs,
            )
        )

    def include_actions_from_launch_description(self, ld: LaunchDescription):
        for e in ld.entities:
            if isinstance(e, Action):
                self._action_list.add_action(e)

    @contextmanager
    def namespace(self, namespace: str) -> Generator[None, None, None]:
        previous_action_list = self._action_list
        self._action_list = ActionListImpl()

        # Push namespace for static analysis
        self._static_analyzer.push_namespace(namespace)

        yield

        # Pop namespace when exiting context
        self._static_analyzer.pop_namespace()

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
        parameters: Optional[SomeParametersDict] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        log_level: Optional[LogLevel] = None,
        **node_kwargs,
    ):
        if executable is None:
            executable = package

        if "emulate_tty" not in node_kwargs and "output" not in node_kwargs:
            # https://github.com/ros2/launch/issues/188
            node_kwargs["emulate_tty"] = True
            node_kwargs["output"] = "screen"

        parameters = self._add_sim_time(parameters)
        node_kwargs = add_log_level(node_kwargs, log_level)

        # Track node for static analysis
        self._static_analyzer.track_node(
            package=package,
            executable=executable,
            parameters=list(generate_parameter_list(parameters, parameters_file)),
            remappings=list(generate_remappings_list(remappings)),
            name=node_kwargs.get("name"),
            **{k: v for k, v in node_kwargs.items() if k not in ["name", "namespace"]},
        )

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
        parameters: Optional[SomeParametersDict] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        log_level: Optional[LogLevel] = None,
        **container_kwargs,
    ) -> Generator[None, None, None]:
        if self._composable_node_list is not None:
            raise ValueError("A composable node container is already in context!")

        self._composable_node_list = []

        # Start static analysis container context
        self._static_analyzer.enter_container_context()

        yield

        executable, args = container_type.value
        container_kwargs["arguments"] = container_kwargs.get("arguments", []) + args

        parameters = self._add_sim_time(parameters)
        container_kwargs = add_log_level(container_kwargs, log_level)

        # Extract namespace from container_kwargs
        namespace = container_kwargs.pop("namespace", "")

        # Track container for static analysis
        self._static_analyzer.exit_container_context(
            name=name,
            executable=executable,
            parameters=list(generate_parameter_list(parameters, parameters_file)),
            remappings=list(generate_remappings_list(remappings)),
            namespace=namespace,
            **{k: v for k, v in container_kwargs.items() if k not in ["arguments", "ros_arguments"]},
        )

        self._action_list.add_action(
            ros_act.ComposableNodeContainer(
                package="rclcpp_components",
                executable=executable,
                name=name,
                namespace=namespace,
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
        parameters: Optional[SomeParametersDict] = None,
        parameters_file: Optional[SomeParameterFile] = None,
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        **node_kwargs,
    ):
        if not self.in_composable_node_context:
            raise ValueError("There is no composable node container in context!")

        parameters = self._add_sim_time(parameters)

        # Track composable node for static analysis
        self._static_analyzer.track_composable_node(
            package=package,
            plugin=plugin,
            parameters=list(generate_parameter_list(parameters, parameters_file)),
            remappings=list(generate_remappings_list(remappings)),
            name=node_kwargs.get("name"),
            **{k: v for k, v in node_kwargs.items() if k not in ["name", "namespace"]},
        )

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

        # Track topic relay for static analysis
        self._static_analyzer.track_topic_relay(
            from_topic=from_,
            to_topic=to,
            relay_type="relay",
            lazy=lazy,
        )

        def create_node():
            self.composable_node(
                "topic_tools",
                "topic_tools::RelayNode",
                name=f"relay_{friendly_from}_{friendly_to}",
                parameters={"input_topic": from_, "output_topic": to, "lazy": lazy},
            )

        if self.in_composable_node_context:
            create_node()
        else:
            with self.composable_node_container(f"container_relay_{friendly_from}_{friendly_to}"):
                create_node()

    def topic_throttle_hz(self, topic: str, rate: float, lazy: bool = True, include_hz_in_output_topic: bool = False):
        friendly_topic = topic.replace("/", "_").strip("_")
        friendly_rate = str(rate).replace(".", "_")

        output_topic = topic + f"/throttled" + ("/hz_{friendly_rate}" if include_hz_in_output_topic else "")

        # Track topic throttle for static analysis
        self._static_analyzer.track_topic_relay(
            from_topic=topic,
            to_topic=output_topic,
            relay_type="throttle",
            lazy=lazy,
            rate=rate,
        )

        def create_node():
            self.composable_node(
                "topic_tools",
                "topic_tools::ThrottleNode",
                name=f"throttle_{friendly_topic}_{friendly_rate}_hz",
                parameters={
                    "input_topic": topic,
                    "output_topic": output_topic,
                    "lazy": lazy,
                    "throttle_type": "messages",
                    "msgs_per_sec": float(rate),
                },
            )

        if self.in_composable_node_context:
            create_node()
        else:
            with self.composable_node_container(f"container_throttle_{friendly_topic}_{friendly_rate}_hz"):
                create_node()

    def opaque_function(self, func: Callable[[LaunchContext], list[Action]]):
        self._action_list.add_action(act.OpaqueFunction(function=func))

    def get_static_information(self) -> StaticInformation:
        """
        Get all static information collected from this launch file.

        Returns a StaticInformation object containing all tracked nodes,
        containers (with nested composable nodes), topic relays, and included launch files.
        """
        return self._static_analyzer.get_static_information()
