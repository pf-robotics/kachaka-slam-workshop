from typing import List, Optional, TypedDict

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from scipy.spatial import KDTree

from .data import Pose2d

HIT_THRESH = 80
IGNORE_THRESH = 20


class Location(TypedDict):
    x: float
    y: float


class DistanceMap:
    INVALID_VALUE = -1

    def __init__(
        self, width: int, height: int, resolution: float, origin: Pose2d
    ) -> None:
        self._width = width
        self._height = height
        self._resolution = resolution
        self._origin = origin
        self._data: List[float] = [0.0] * width * height

    @property
    def resolution(self) -> float:
        return self._resolution

    @property
    def data(self) -> List[float]:
        return self._data

    def index(self, loc: Location) -> int:
        offset_loc = Location(
            x=loc["x"] - self._origin.x,
            y=loc["y"] - self._origin.y,
        )
        # todo: rotation
        return int(offset_loc["x"] / self._resolution) + self._width * int(
            offset_loc["y"] / self._resolution
        )

    def get(self, i: int) -> float:
        if i < len(self._data) and i >= 0:
            return self._data[i]
        else:
            return DistanceMap.INVALID_VALUE

    def set(self, i: int, value: float) -> None:
        if i < len(self._data) and i >= 0:
            self._data[i] = value


def get_location_from_index(
    index: int,
    info: MapMetaData,
) -> np.ndarray:
    w = int(index % info.width)
    h = int(index / info.width)

    x = w * info.resolution + info.origin.position.x
    y = h * info.resolution + info.origin.position.y

    # todo: rotation

    return np.array([x, y])


def create_distance_map(
    map: OccupancyGrid,
) -> DistanceMap:
    occupied_th = 70.0
    points: list[np.ndarray] = []
    for i, d in enumerate(map.data):
        if d > occupied_th:
            points.append(get_location_from_index(i, map.info))

    tree = KDTree(np.stack(points))

    origin_pose2d = Pose2d(
        x=map.info.origin.position.x,
        y=map.info.origin.position.y,
        yaw=0.0,  # todo:
    )

    ret = DistanceMap(
        map.info.width, map.info.height, map.info.resolution, origin_pose2d
    )
    for i, _ in enumerate(map.data):
        pos = get_location_from_index(i, map.info)
        dist, _ = tree.query(pos, k=1)
        ret.set(i, dist)

    return ret


class DistanceMapManager:
    def __init__(self) -> None:
        self._distance_map: Optional[DistanceMap] = None

    def update_map(self, map: OccupancyGrid) -> None:
        self._distance_map = create_distance_map(map)

    def is_ready(self) -> bool:
        return self._distance_map is not None

    @property
    def resolution(self) -> Optional[float]:
        if self._distance_map is None:
            return None

        return self._distance_map._resolution

    @property
    def distance_map(self) -> Optional[DistanceMap]:
        return self._distance_map

    def get_value(self, loc: Location) -> Optional[float]:
        if self._distance_map is None:
            return None

        return self._distance_map.get(self._distance_map.index(loc))
