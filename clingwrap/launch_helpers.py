from enum import Enum

from launch import SomeSubstitutionsType
from launch import substitutions as sub
from launch.utilities.type_utils import SomeValueType
from launch_ros import parameter_descriptions as param
from launch_ros import substitutions as ros_sub


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


def find_file(
    package: str,
    file_dir: SomeSubstitutionsType,
    file_name: SomeSubstitutionsType,
) -> sub.PathJoinSubstitution:
    return sub.PathJoinSubstitution([ros_sub.FindPackageShare(package), file_dir, file_name])


def as_str_param(value: SomeValueType) -> param.ParameterValue:
    return param.ParameterValue(value, value_type=str)
