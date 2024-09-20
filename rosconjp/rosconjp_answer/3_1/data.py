from dataclasses import dataclass
from typing import List

import numpy as np
from typing_extensions import Self


@dataclass
class Points2d:
    points: np.ndarray


def yaw_from_rot_mat2d(rot: np.ndarray) -> float:
    assert rot.ndim == 2
    assert rot.shape[0] == rot.shape[1] == 2

    return np.arctan2(rot[1, 0], rot[0, 0])  # type: ignore


@dataclass
class Pose2d:
    x: float
    y: float
    yaw: float

    def mat(self) -> np.ndarray:
        return np.array(
            [
                [np.cos(self.yaw), -np.sin(self.yaw), self.x],
                [np.sin(self.yaw), np.cos(self.yaw), self.y],
                [0.0, 0.0, 1.0],
            ]
        )

    def inv(self) -> Self:
        return self.from_mat(np.linalg.inv(self.mat()))

    @classmethod
    def from_rot_and_trans(cls, rot: np.ndarray, trans: np.ndarray) -> Self:
        assert rot.ndim == 2 and rot.shape[0] == rot.shape[1] == 2
        assert trans.ndim == 1 and trans.shape[0] == 2
        return cls(x=trans[0], y=trans[1], yaw=yaw_from_rot_mat2d(rot))

    @classmethod
    def from_mat(cls, m: np.ndarray) -> Self:
        assert m.ndim == 2 and m.shape[0] == m.shape[1] == 3
        return cls.from_rot_and_trans(m[:2, :2], m[:2, 2])

    def transform(self, points: np.ndarray) -> np.ndarray:
        assert points.ndim == 2 and points.shape[1] == 2
        mat = self.mat()
        return points @ mat[:2, :2].T + mat[:2, 2]  # type: ignore

    def multiply(self, pose2d: Self) -> Self:
        return self.from_mat(self.mat() @ pose2d.mat())

    def jac(self, p: np.ndarray) -> np.ndarray:
        rot = self.mat()[:2, :2]
        skew = np.array(
            [
                [0, -1],
                [1, 0],
            ]
        )
        jac_mat: list[np.ndarray] = []
        for pi in p:
            m = np.concatenate([rot, rot @ skew @ pi[:, None]], axis=1)
            jac_mat += [m]

        return np.stack(jac_mat)


@dataclass
class PoseWithPoints2d:
    pose: Pose2d
    points: Points2d


@dataclass
class ScanSetting:
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass
class Scan:
    ranges: List[float]
    laser_to_base_pose: Pose2d


def scan_to_points2d(scan: Scan, setting: ScanSetting) -> Points2d:
    # scanデータを点群へ変換する
    def _calc_yaw(index: int) -> float:
        return setting.angle_min + setting.angle_increment * index

    points = np.stack(
        [
            r * np.array([np.cos(_calc_yaw(i)), np.sin(_calc_yaw(i))])
            for i, r in enumerate(scan.ranges)
            if r > setting.range_min and r < setting.range_max
        ]
    )
    # lidarの座標系からbase_footprint座標系へ変換する
    points = scan.laser_to_base_pose.transform(points)

    return Points2d(points)
