import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rosconjp.conversion import pose2d_to_pose_stamped_msg
from rosconjp.data import Pose2d
from rosconjp.profile import SensorProfile


class OdometryNode(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("odometry_node")
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, SensorProfile
        )
        self._pose_pub = self.create_publisher(PoseStamped, "/odom_pose", 10)

        self._last_odom_time: Time | None = None

        # 0. initialize pose
        self._odom_pose = Pose2d(x=0.0, y=0.0, yaw=0.0)

    def _odom_callback(self, odom_msg: Odometry) -> None:
        current_time = Time.from_msg(odom_msg.header.stamp)
        if self._last_odom_time is None:
            self._last_odom_time = current_time
            return

        # 1. calculate dt
        dt = (current_time.nanoseconds - self._last_odom_time.nanoseconds) / 1e9

        # 2. calculate relative pose from the last update
        vx = odom_msg.twist.twist.linear.x  # x
        vy = odom_msg.twist.twist.linear.y  # y
        va = odom_msg.twist.twist.angular.z  # angular around z axis
        relative_pose = Pose2d(
            x=vx * dt,
            y=vy * dt,
            yaw=va * dt,
        )

        # 3. accumulate relative pose with the current odom pose
        self._odom_pose = self._odom_pose.multiply(relative_pose)

        # publish
        self._pose_pub.publish(
            pose2d_to_pose_stamped_msg(
                self._odom_pose, "map", self.get_clock().now().to_msg()
            )
        )

        self._last_odom_time = current_time


def main() -> None:
    rclpy.init(args=None)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)

    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
