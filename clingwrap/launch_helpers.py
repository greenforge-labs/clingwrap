from launch import SomeSubstitutionsType
from launch import substitutions as sub
from launch.utilities.type_utils import SomeValueType
from launch_ros import parameter_descriptions as param
from launch_ros import substitutions as ros_sub


def find_file(
    package: str,
    file_dir: SomeSubstitutionsType,
    file_name: SomeSubstitutionsType,
) -> sub.PathJoinSubstitution:
    return sub.PathJoinSubstitution([ros_sub.FindPackageShare(package), file_dir, file_name])


def as_str_param(value: SomeValueType) -> param.ParameterValue:
    return param.ParameterValue(value, value_type=str)
