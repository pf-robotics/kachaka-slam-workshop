import copy
import math
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
from rosconjp.data import Pose2d, Scan, ScanSetting
from rosconjp.map import DistanceMap, DistanceMapManager
from rosconjp.params import MCLParams


@dataclass
class Particle:
    pose: Pose2d
    weight: float


@dataclass
class Movement:
    d_dist: float
    d_rot: float


class MCLModelType(Enum):
    BEAM_MODEL = auto()
    LIKELIHOOD_FIELD = auto()


@dataclass
class MCLLikelihoodFieldModelConfig:
    lfm_sigma: float
    map_resolution: float
    z_max: float
    z_rand: float
    z_hit: float
    scan_step: int


class MCLLikelihoodFieldModel:
    def __init__(
        self,
        config: MCLLikelihoodFieldModelConfig,
        map_manager: DistanceMapManager,
    ) -> None:
        self._config = config
        self._map_manager = map_manager
        self._scan_config: ScanSetting | None = None

    def _get_resolution(self) -> float | None:
        return self._map_manager.resolution

    def update_scan_config(self, setting: ScanSetting) -> None:
        self._scan_config = setting

    def calc_likelihood(self, particle: Particle, scan: Scan) -> float | None:
        """
        重み（尤度）計算
        """
        if self._scan_config is None:
            return None

        if not self._map_manager.is_ready():
            return None

        map_resolution = self._get_resolution()
        assert map_resolution is not None

        var = self._config.lfm_sigma**2
        norm = 1.0 / math.sqrt(2.0 * math.pi * var)
        # p_max = ...
        # p_rand = ...

        transform_mat = particle.pose.mat() @ scan.transform.mat()
        w = 0.0
        index = 0
        while index < len(scan.ranges):
            r = scan.ranges[index]
            if r < self._scan_config.range_min or self._scan_config.range_max < r:
                # inc = ...
                pass
            else:
                a = (
                    self._scan_config.angle_min
                    + self._scan_config.angle_increment * index
                )
                point = np.array([r * math.cos(a), r * math.sin(a)])
                point = transform_mat[:2, :2] @ point + transform_mat[:2, 2]

                # 距離画像の値を参照
                map_value = self._map_manager.get_value({"x": point[0], "y": point[1]})
                assert map_value is not None

                if map_value == DistanceMap.INVALID_VALUE:
                    # 距離画像の値が不正値
                    # inc = ...
                    pass
                else:
                    # inc = ...
                    pass
            # w = ...
            index += self._config.scan_step

        # return w


class MCL:
    def __init__(
        self,
        weigh_model: MCLLikelihoodFieldModel,
        mcl_params: MCLParams,
    ) -> None:
        self._weigh_model = weigh_model
        self._mcl_params = mcl_params

        self._last_estimated_pose: Pose2d | None = None
        self._effective_sample_size = float(mcl_params.effective_sample_size)
        self._num_particles = mcl_params.num_particles

        self._particles: list[Particle] = [
            Particle(Pose2d(0, 0, 0), 0.0) for _ in range(self._num_particles)
        ]
        self._rng = np.random.default_rng(seed=1)

    def initialize(self, initial_pose: Pose2d) -> None:
        for i in range(len(self._particles)):
            self._particles[i].pose.x = initial_pose.x + self._rng.normal(
                0.0, self._mcl_params.initial_noise_x
            )
            self._particles[i].pose.y = initial_pose.y + self._rng.normal(
                0.0, self._mcl_params.initial_noise_y
            )
            self._particles[i].pose.yaw = initial_pose.yaw + self._rng.normal(
                0.0, 3.0 * math.pi / 180.0
            )
            self._particles[i].weight = 1 / len(self._particles)

    @property
    def particles(self) -> list[Particle]:
        return self._particles

    def move_particles(self, movement: Movement) -> None:
        """
        パーティクルをオドメトリ情報を元に移動させる
        """
        pass

    def weigh_particles(self, scan: Scan) -> None:
        """
        パーティクルの重みを計算する
        """
        likelihoods = np.array(
            [self._weigh_model.calc_likelihood(p, scan) for p in self._particles]
        )
        sum_likelihood = float(np.sum(likelihoods))

        sum = 0.0
        for i in range(len(self._particles)):
            w = float(likelihoods[i]) / sum_likelihood
            self._particles[i].weight = w
            sum += w**2

        self._effective_sample_size = 1.0 / sum

    def estimate_pose(self) -> Pose2d:
        """
        姿勢を推定
        """
        return pose

    def resample(self) -> None:
        """
        リサンプリング
        """
        th = len(self._particles) * self._mcl_params.resample_thresh
        if self._effective_sample_size > th:
            return

        # resampling
