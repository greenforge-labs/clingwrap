"""Static analysis for ROS2 launch files."""

from launch import SomeSubstitutionsType
from launch_ros.parameters_type import SomeParameterFile, SomeParametersDict

from .static_info import (
    ComposableNodeContainerInfo,
    ComposableNodeInfo,
    LaunchFileInclude,
    NodeInfo,
    StaticInformation,
    TopicRelayInfo,
)

from typing import Optional


class LaunchStaticAnalyzer:
    """
    Tracks static information about nodes, containers, and inclusions in a launch file.

    This class automatically manages namespace context and container context,
    mirroring the behavior of LaunchBuilder.
    """

    def __init__(self):
        """Initialize the static analyzer."""
        # Tracking state
        self._tracked_nodes: list[NodeInfo] = []
        self._tracked_composable_node_containers: list[ComposableNodeContainerInfo] = []
        self._tracked_topic_relays: list[TopicRelayInfo] = []
        self._tracked_included_launch_files: list[LaunchFileInclude] = []

        # Context tracking
        self._namespace_stack: list[str] = []
        self._container_context_nodes: Optional[list[ComposableNodeInfo]] = None

    def get_current_namespace(self) -> Optional[str]:
        """Get the current namespace from the stack, or None if empty."""
        return "/".join(self._namespace_stack) if self._namespace_stack else None

    def push_namespace(self, namespace: str) -> None:
        """Enter a namespace context."""
        self._namespace_stack.append(namespace)

    def pop_namespace(self) -> None:
        """Exit a namespace context."""
        if self._namespace_stack:
            self._namespace_stack.pop()

    def in_container_context(self) -> bool:
        """Check if currently tracking nodes for a container."""
        return self._container_context_nodes is not None

    def enter_container_context(self) -> None:
        """Start tracking composable nodes for a container."""
        if self._container_context_nodes is not None:
            raise ValueError("Already in a container context!")
        self._container_context_nodes = []

    def exit_container_context(
        self,
        name: str,
        executable: str,
        parameters: Optional[SomeParametersDict],
        parameters_file: Optional[SomeParameterFile],
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]],
        **kwargs,
    ) -> None:
        """
        Finish tracking a container and create ComposableNodeContainerInfo.

        Args:
            name: Container name
            executable: Container executable (e.g., "component_container")
            parameters: Parameters dict
            parameters_file: Parameters file path
            remappings: Remappings dict
            **kwargs: Additional container arguments (including optional namespace)
        """
        if self._container_context_nodes is None:
            raise ValueError("Not in a container context!")

        # Extract namespace if explicitly provided, otherwise use current namespace from stack
        namespace = kwargs.pop("namespace", None)
        if namespace is None or namespace == "":
            namespace = self.get_current_namespace()

        # Create ComposableNodeContainerInfo with the accumulated composable nodes
        self._tracked_composable_node_containers.append(
            ComposableNodeContainerInfo(
                name=name,
                namespace=namespace,
                package="rclcpp_components",
                executable=executable,
                parameters=parameters,
                parameters_file=parameters_file,
                remappings=remappings,
                nodes=self._container_context_nodes.copy(),
                additional_kwargs=kwargs,
            )
        )

        # Clear the context
        self._container_context_nodes = None

    def track_node(
        self,
        package: str,
        executable: str,
        parameters: Optional[SomeParametersDict],
        parameters_file: Optional[SomeParameterFile],
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]],
        name: Optional[str] = None,
        **kwargs,
    ) -> None:
        """
        Track a regular ROS2 node.

        Automatically uses the current namespace context.
        """
        self._tracked_nodes.append(
            NodeInfo(
                package=package,
                executable=executable,
                name=name,
                namespace=self.get_current_namespace(),
                parameters=parameters,
                parameters_file=parameters_file,
                remappings=remappings,
                additional_kwargs=kwargs,
            )
        )

    def track_composable_node(
        self,
        package: str,
        plugin: str,
        parameters: Optional[SomeParametersDict],
        parameters_file: Optional[SomeParameterFile],
        remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]],
        name: Optional[str] = None,
        **kwargs,
    ) -> None:
        """
        Track a composable node.

        Automatically adds to the current container context and uses current namespace.
        Must be called within a container context.
        """
        if not self.in_container_context():
            raise ValueError("Cannot track composable node outside container context!")

        composable_node = ComposableNodeInfo(
            package=package,
            plugin=plugin,
            name=name,
            namespace=self.get_current_namespace(),
            parameters=parameters,
            parameters_file=parameters_file,
            remappings=remappings,
            additional_kwargs=kwargs,
        )

        self._container_context_nodes.append(composable_node)  # type: ignore

    def track_topic_relay(
        self,
        from_topic: str,
        to_topic: str,
        relay_type: str = "relay",
        lazy: bool = True,
        rate: Optional[float] = None,
    ) -> None:
        """
        Track a topic relay or throttle.

        Automatically uses the current namespace context.

        Args:
            from_topic: Source topic
            to_topic: Destination topic
            relay_type: "relay" or "throttle"
            lazy: Whether the relay is lazy
            rate: Throttle rate in Hz (for throttle type)
        """
        self._tracked_topic_relays.append(
            TopicRelayInfo(
                from_topic=from_topic,
                to_topic=to_topic,
                relay_type=relay_type,
                lazy=lazy,
                rate=rate,
                namespace=self.get_current_namespace(),
            )
        )

    def track_included_launch_file(
        self,
        package: str,
        launch_file: str,
        directory: str = "launch",
        launch_arguments: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
    ) -> None:
        """
        Track an included launch file.

        Automatically uses the current namespace context.

        Args:
            package: Package containing the launch file
            launch_file: Launch file name
            directory: Directory within package (default: "launch")
            launch_arguments: Arguments passed to the launch file
        """
        self._tracked_included_launch_files.append(
            LaunchFileInclude(
                package=package,
                launch_file=launch_file,
                directory=directory,
                launch_arguments=launch_arguments or {},
                namespace=self.get_current_namespace(),
            )
        )

    def get_static_information(self) -> StaticInformation:
        """
        Get all static information collected from the launch file.

        Returns:
            StaticInformation object with all tracked data
        """
        return StaticInformation(
            nodes=self._tracked_nodes.copy(),
            composable_node_containers=self._tracked_composable_node_containers.copy(),
            topic_relays=self._tracked_topic_relays.copy(),
            included_launch_files=self._tracked_included_launch_files.copy(),
        )
