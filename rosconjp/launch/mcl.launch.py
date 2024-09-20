from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_directory_arg = DeclareLaunchArgument("map_directory", default_value="testmap")
    map_directory = LaunchConfiguration("map_directory")
    set_use_sime_time = SetParameter(
        name="use_sim_time",
        value=True,
    )
    map_absolute_directory = PathJoinSubstitution(["/resources", map_directory])
    map_yaml_path = PathJoinSubstitution(["/resources", map_directory, "map.yaml"])
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        emulate_tty=True,
        parameters=[{"yaml_filename": map_yaml_path}],
    )
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{"autostart": True}, {"node_names": ["map_server"]}],
    )
    mcl_config_path = PathJoinSubstitution(
        [FindPackageShare("rosconjp"), "config", "mcl_config.yaml"]
    )
    mcl_node = Node(
        package="rosconjp",
        executable="mcl",
        parameters=[mcl_config_path, {"image_map_dir": map_absolute_directory}],
        remappings=[
            ("/image", "/kachaka/front_camera/image_raw/compressed"),
            ("/odom", "/kachaka/wheel_odometry/wheel_odometry"),
            ("/scan", "/kachaka/lidar/scan"),
        ],
        output="screen",
    )
    actions = [
        map_directory_arg,
        set_use_sime_time,
        mcl_node,
        nav2_lifecycle_manager,
        nav2_map_server,
    ]
    return LaunchDescription(actions)
