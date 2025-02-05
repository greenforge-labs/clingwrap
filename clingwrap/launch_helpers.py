from enum import Enum

from launch import SomeSubstitutionsType, Substitution
from launch import substitutions as sub
from launch.utilities.type_utils import SomeValueType
from launch_ros import parameter_descriptions as param
from launch_ros import substitutions as ros_sub

from typing import Text, Union


class ContainerType(Enum):
    """
    Composable node container type.
    """

    # All components using one single-threaded executor
    SINGLE_THREAD = ("component_container", [])
    # Each component gets its own single-threaded executor
    SINGLE_THREAD_EACH = ("component_container_isolated", [])
    # Each component gets its own multi-threaded executor
    MULTI_THREAD_EACH = ("component_container_isolated", ["--use_multi_threaded_executor"])
    # All components using one multi-threaded executor
    MULTI_THREAD = ("component_container_mt", [])


class LogLevel(Enum):
    DEBUG = "debug"
    INFO = "info"
    WARN = "warn"
    ERROR = "error"
    FATAL = "fatal"
    NONE = "none"


def pkg_file(
    package: str,
    file_dir: Union[Text, Substitution],
    file_name: Union[Text, Substitution],
) -> sub.PathJoinSubstitution:
    return sub.PathJoinSubstitution([ros_sub.FindPackageShare(package), file_dir, file_name])


def as_str_param(value: SomeValueType) -> param.ParameterValue:
    return param.ParameterValue(value, value_type=str)


def remap_action(
    from_: Union[Text, Substitution], to: Union[Text, Substitution]
) -> dict[SomeSubstitutionsType, SomeSubstitutionsType]:
    """until https://github.com/ros2/ros2/issues/1312 is fixed"""
    return {
        (from_, "/_action/feedback"): (to, "/_action/feedback"),
        (from_, "/_action/status"): (to, "/_action/status"),
        (from_, "/_action/cancel_goal"): (to, "/_action/cancel_goal"),
        (from_, "/_action/get_result"): (to, "/_action/get_result"),
        (from_, "/_action/send_goal"): [to, "/_action/send_goal"],
    }


def remap_hidden(topic: str) -> dict[str, str]:
    return {topic: f"_hidden_{topic.strip('/')}"}
