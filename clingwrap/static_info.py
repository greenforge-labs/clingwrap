"""Data classes for static analysis of ROS2 launch files."""

from dataclasses import dataclass, field

from launch import SomeSubstitutionsType
from launch_ros.parameters_type import SomeParameterFile, SomeParametersDict

from typing import Any, Optional


@dataclass
class NodeInfo:
    """Information about a regular ROS2 node."""

    package: str
    executable: str
    name: Optional[str] = None
    namespace: Optional[str] = None
    parameters: Optional[SomeParametersDict] = None
    parameters_file: Optional[SomeParameterFile] = None
    remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None
    additional_kwargs: dict[str, Any] = field(default_factory=dict)


@dataclass
class ComposableNodeInfo:
    """Information about a composable ROS2 node."""

    package: str
    plugin: str
    name: Optional[str] = None
    namespace: Optional[str] = None
    parameters: Optional[SomeParametersDict] = None
    parameters_file: Optional[SomeParameterFile] = None
    remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None
    additional_kwargs: dict[str, Any] = field(default_factory=dict)


@dataclass
class ComposableNodeContainerInfo:
    """Information about a composable node container."""

    name: str
    namespace: Optional[str] = None
    package: str = "rclcpp_components"
    executable: str = "component_container"  # or component_container_mt, component_container_isolated
    parameters: Optional[SomeParametersDict] = None
    parameters_file: Optional[SomeParameterFile] = None
    remappings: Optional[dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None
    nodes: list[ComposableNodeInfo] = field(default_factory=list)
    additional_kwargs: dict[str, Any] = field(default_factory=dict)


@dataclass
class TopicRelayInfo:
    """Information about a topic relay."""

    from_topic: str
    to_topic: str
    relay_type: str = "relay"  # "relay" or "throttle"
    lazy: bool = False
    # For throttle:
    rate: Optional[float] = None
    namespace: Optional[str] = None


@dataclass
class LaunchFileInclude:
    """Information about an included launch file."""

    package: str
    launch_file: str
    directory: str
    launch_arguments: dict[SomeSubstitutionsType, SomeSubstitutionsType] = field(default_factory=dict)
    namespace: Optional[str] = None


@dataclass
class StaticInformation:
    """Complete static information collected from a launch file."""

    nodes: list[NodeInfo] = field(default_factory=list)
    composable_node_containers: list[ComposableNodeContainerInfo] = field(default_factory=list)
    topic_relays: list[TopicRelayInfo] = field(default_factory=list)
    included_launch_files: list[LaunchFileInclude] = field(default_factory=list)

    def get_all_composable_nodes(self) -> list[ComposableNodeInfo]:
        """
        Get all composable nodes from all containers as a flat list.

        Returns:
            List of all composable nodes across all containers
        """
        result = []
        for container in self.composable_node_containers:
            result.extend(container.nodes)
        return result

    def merge(self, other: "StaticInformation") -> None:
        """
        Merge another StaticInformation into this one.

        Args:
            other: The StaticInformation to merge
            namespace_prefix: Optional namespace to prepend to all entities from other
        """
        self.nodes.extend(other.nodes)
        self.composable_node_containers.extend(other.composable_node_containers)
        self.topic_relays.extend(other.topic_relays)
        self.included_launch_files.extend(other.included_launch_files)
