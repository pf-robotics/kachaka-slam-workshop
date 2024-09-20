from dataclasses import dataclass
from typing import Callable

import numpy as np
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from rosconjp.conversion import pose2d_to_pose_msg
from rosconjp.data import Pose2d, PoseWithPoints2d


@dataclass
class OccupancyGrid:
    UNKNOWN = -1
    OCCUPIED = 50
    FREE = -10

    @dataclass
    class Cell:
        data: int

    width: int
    height: int
    resolution: float
    cells: list[Cell]
    origin: Pose2d


def to_occupancy_grid_msg(data: OccupancyGrid) -> OccupancyGridMsg:
    msg = OccupancyGridMsg()
    msg.header.frame_id = "map"
    msg.info.width = data.width
    msg.info.height = data.height
    msg.info.resolution = data.resolution
    msg.info.origin = pose2d_to_pose_msg(data.origin)
    msg.data = [cell.data for cell in data.cells]
    return msg


def point_to_occupancy_grid_location(
    origin: Pose2d, resolution: float, point: np.ndarray
) -> np.ndarray:
    if point.ndim == 1:
        return np.ceil(origin.inv().transform(point[None])[0] / resolution)  # type: ignore
    else:
        return np.ceil(origin.inv().transform(point) / resolution)  # type: ignore


def render_occupancy_grid_map_with_ray(
    occupancy_grid_map: OccupancyGrid,
    pose_with_points: PoseWithPoints2d,
) -> None:
    center = np.array([pose_with_points.pose.x, pose_with_points.pose.y])
    center_on_grid = point_to_occupancy_grid_location(
        occupancy_grid_map.origin,
        occupancy_grid_map.resolution,
        center,
    )
    points = pose_with_points.pose.transform(pose_with_points.points.points)
    points_on_grid = point_to_occupancy_grid_location(
        occupancy_grid_map.origin,
        occupancy_grid_map.resolution,
        points,
    )
    margin = 0.1 / occupancy_grid_map.resolution

    def _ray_func(x: int, y: int, dist_to_point: float) -> None:
        dist = np.sqrt((x - center_on_grid[0]) ** 2 + (y - center_on_grid[1]) ** 2)
        if abs(dist - dist_to_point) < margin / 2.0:
            log_odds = OccupancyGrid.OCCUPIED
        else:
            log_odds = OccupancyGrid.FREE

        index = y * occupancy_grid_map.width + x
        if index < len(occupancy_grid_map.cells):
            occupancy_grid_map.cells[index].data = max(
                min(occupancy_grid_map.cells[index].data + log_odds, 100), 0
            )

    for p in points_on_grid:
        dist_to_point = float(np.linalg.norm(p - center_on_grid))
        ray(
            int(center_on_grid[0]),
            int(center_on_grid[1]),
            int(p[0]),
            int(p[1]),
            lambda x, y: _ray_func(x, y, dist_to_point),
        )


def ray(x0: int, y0: int, x1: int, y1: int, func: Callable[[int, int], None]) -> None:
    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0

    delta_x = x1 - x0
    delta_y = abs(y1 - y0)
    error = delta_x / 2

    y = y0
    ystep = 1 if y0 < y1 else -1
    for x in range(x0, x1, 1):
        if steep:
            func(y, x)
        else:
            func(x, y)
        error = error - delta_y
        if error < 0:
            y = y + ystep
            error = error + delta_x
