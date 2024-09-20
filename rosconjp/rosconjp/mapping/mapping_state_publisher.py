#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class MappingStatePublisher:
    def __init__(self, node: Node) -> None:
        self.map_points_pub = node.create_publisher(PointCloud2, "/map_points", 10)
        self.current_points_pub = node.create_publisher(
            PointCloud2, "/current_points", 10
        )
        self.transformed_points_pub = node.create_publisher(
            PointCloud2, "/transformed_points", 10
        )
        self.pose_pub = node.create_publisher(PoseStamped, "/pose", 10)

    def publish(
        self,
        current_points: PointCloud2,
        transformed_points: PointCloud2,
        map_points: PointCloud2,
        pose: PoseStamped,
    ) -> None:
        self.map_points_pub.publish(map_points)
        self.current_points_pub.publish(current_points)
        self.transformed_points_pub.publish(transformed_points)
        self.pose_pub.publish(pose)
