from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    set_use_sime_time = SetParameter(
        name="use_sim_time",
        value=True,
    )
    mapping_config_path = (
        PathJoinSubstitution(
            [FindPackageShare("rosconjp"), "config", "mapping_config_icp.yaml"]
        ),
    )
    mapping_node = Node(
        package="rosconjp",
        executable="mapping",
        parameters=[mapping_config_path],
        remappings=[
            ("/odom", "/kachaka/wheel_odometry/wheel_odometry"),
            ("/scan", "/kachaka/lidar/scan"),
            (
                "/camera_front/image_raw/compressed",
                "/kachaka/front_camera/image_raw/compressed",
            ),
        ],
        output="screen",
    )
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        emulate_tty=True,
    )
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{"autostart": True}, {"node_names": ["map_saver"]}],
    )
    actions = [
        set_use_sime_time,
        mapping_node,
        nav2_map_saver,
        nav2_lifecycle_manager,
    ]
    return LaunchDescription(actions)
