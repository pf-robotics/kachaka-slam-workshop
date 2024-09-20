import copy
from dataclasses import dataclass
from typing import Callable

import numpy as np
from rosconjp.data import Points2d, Pose2d
from rosconjp.mapping.icp_p2plane import PointToPlane
from rosconjp.mapping.icp_p2point import PointToPoint
from rosconjp.params import ICPParams
from scipy.spatial import KDTree


@dataclass
class ICPEvent:
    iter: int
    source: np.ndarray
    target: np.ndarray
    choice: np.ndarray
    transformed_source: np.ndarray
    distances: np.ndarray
    normals: np.ndarray | None


class ICP:
    def __init__(self, method: str, config: ICPParams) -> None:
        self._icp_method: PointToPoint | PointToPlane
        if method == "p2point":
            self._icp_method = PointToPoint()
        elif method == "p2plane":
            self._icp_method = PointToPlane(config.normal_search_radius)
        else:
            raise Exception(f"icp_method {method} is not supported")
        self._config = config
        self._reporter: Callable[[ICPEvent], None] | None = None

    def set_reporter(self, reporter: Callable[[ICPEvent], None]) -> None:
        self._reporter = reporter

    def estimate(
        self,
        source: Points2d,
        target: Points2d,
        initial_pose: Pose2d | None = None,
    ) -> tuple[Pose2d, np.ndarray]:
        if initial_pose is not None:
            transformed_source_points = initial_pose.transform(source.points)
            source_to_target = copy.copy(initial_pose)
        else:
            transformed_source_points = np.copy(source.points)
            source_to_target = Pose2d(0.0, 0.0, 0.0)

        tree = KDTree(target.points)
        if isinstance(self._icp_method, PointToPlane):
            self._icp_method.reset(tree, len(target.points))
        for iter in range(self._config.num_iter):
            distances, target_indices = tree.query(transformed_source_points)
            choice = distances <= self._config.min_distance_to_match
            if self._config.use_reciprocal:
                source_tree = KDTree(transformed_source_points)
                _, source_indices = source_tree.query(target.points[target_indices])
                print(f"before {np.sum(choice)=}", end="")
                choice = choice & (source_indices == list(range(len(target_indices))))
                print(f"| after {np.sum(choice)=}")

            source_to_target_i = self._icp_method.estimate_rigid_body_transformation(
                transformed_source_points, target.points, target_indices, choice
            )

            updated_transformed_source_points = source_to_target_i.transform(
                transformed_source_points
            )

            error = np.sum(
                np.abs(transformed_source_points - updated_transformed_source_points)
            )
            error_to_target = np.sum(
                np.abs(
                    target.points[target_indices] - updated_transformed_source_points
                )
            )
            error_before = np.sum(
                np.abs(target.points[target_indices] - transformed_source_points)
            )

            print(f"{error=}, {error_to_target=}, {error_before=}")

            if self._reporter:
                self._reporter(
                    ICPEvent(
                        iter=iter,
                        source=np.copy(transformed_source_points),
                        target=np.copy(target.points[target_indices]),
                        choice=np.copy(choice),
                        transformed_source=np.copy(updated_transformed_source_points),
                        distances=np.copy(distances),
                        normals=(
                            self._icp_method.get_normals()
                            if isinstance(self._icp_method, PointToPlane)
                            else None
                        ),
                    )
                )

            transformed_source_points = updated_transformed_source_points
            source_to_target = source_to_target_i.multiply(source_to_target)

            if error < 1e-9:
                break

        return source_to_target, transformed_source_points
