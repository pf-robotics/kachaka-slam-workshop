from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    mode_arg = DeclareLaunchArgument("mode", default_value="localization")
    map_directory_arg = DeclareLaunchArgument("map_directory", default_value="testmap")
    map_directory = LaunchConfiguration("map_directory")
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("rosconjp"), "mcl.launch.py"])]
        ),
        launch_arguments={
            "map_directory": map_directory,
        }.items(),
        condition=LaunchConfigurationEquals("mode", "localization"),
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("rosconjp"), "mapping.launch.py"])]
        ),
        condition=LaunchConfigurationEquals("mode", "mapping"),
    )
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("rosconjp"), "odometry.launch.py"])]
        ),
        condition=LaunchConfigurationEquals("mode", "odometry"),
    )
    nodes = [
        mode_arg,
        map_directory_arg,
        localization_launch,
        mapping_launch,
        odometry_launch,
    ]
    return LaunchDescription(nodes)
