#!/usr/bin/env python3
from typing import List

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from rclpy.node import Node
from rosconjp.data import Pose2d, PoseWithPoints2d
from rosconjp.occupancy_grid_map import (
    OccupancyGrid,
    render_occupancy_grid_map_with_ray,
    to_occupancy_grid_msg,
)
from rosconjp.profile import MapProfile


class OccupancyGridMapPublisher:
    def __init__(self, node: Node) -> None:
        node.declare_parameter("occupancy_grid_resolution", rclpy.Parameter.Type.DOUBLE)

        self._resolution = (
            node.get_parameter("occupancy_grid_resolution")
            .get_parameter_value()
            .double_value
        )
        self._map_pub = node.create_publisher(
            OccupancyGridMsg,
            "/map",
            qos_profile=MapProfile,
        )

    def publish(
        self, points: np.ndarray, pose_with_points_list: List[PoseWithPoints2d]
    ) -> None:
        min_point = np.min(points, axis=0)
        max_point = np.max(points, axis=0)
        width = int((max_point[0] - min_point[0]) / self._resolution)
        height = int((max_point[1] - min_point[1]) / self._resolution)
        occupancy_grid_map = OccupancyGrid(
            width=width,
            height=height,
            resolution=self._resolution,
            cells=[
                OccupancyGrid.Cell(
                    data=OccupancyGrid.UNKNOWN,
                )
                for _ in range(width * height)
            ],
            origin=Pose2d(float(min_point[0]), float(min_point[1]), 0.0),
        )
        for pose_with_points in pose_with_points_list:
            render_occupancy_grid_map_with_ray(occupancy_grid_map, pose_with_points)

        map_msg = to_occupancy_grid_msg(occupancy_grid_map)
        self._map_pub.publish(map_msg)
