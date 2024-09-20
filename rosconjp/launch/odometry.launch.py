from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    set_use_sime_time = SetParameter(
        name="use_sim_time",
        value=True,
    )
    odometry_node = Node(
        package="rosconjp",
        executable="odometry",
        parameters=[],
        remappings=[("/odom", "/kachaka/wheel_odometry/wheel_odometry")],
        output="screen",
    )
    actions = [
        set_use_sime_time,
        odometry_node,
    ]
    return LaunchDescription(actions)
