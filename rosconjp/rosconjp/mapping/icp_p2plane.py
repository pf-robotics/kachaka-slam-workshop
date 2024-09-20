import numpy as np
from rosconjp.data import Pose2d
from scipy.spatial import KDTree


def _estimate_normal_from_points(p: np.ndarray, cache: np.ndarray | None) -> np.ndarray:
    if cache is None:
        evals, evecs = np.linalg.eig(np.cov(p.T))
        normal: np.ndarray = evecs[:, np.argmin(evals)]
        normalized: np.ndarray = normal / np.linalg.norm(normal, ord=2)
        return normalized
    else:
        return cache


def _construct_normals(
    points: np.ndarray,
    cache_table: list[np.ndarray | None],
    kdtree: KDTree,
    search_radius: float,
) -> tuple[np.ndarray, np.ndarray]:
    neighbors_for_all_points = kdtree.query_ball_point(points, search_radius)

    normals = np.stack(
        [
            (
                _estimate_normal_from_points(
                    kdtree.data[
                        np.random.choice(
                            neighbors, min(30, len(neighbors)), replace=False
                        )
                    ],
                    cache,
                )
                if len(neighbors) >= 5
                else np.zeros(2)
            )
            for neighbors, cache in zip(neighbors_for_all_points, cache_table)
        ]
    )
    choice = np.abs(np.linalg.norm(normals, axis=-1) - 1.0) < 1e-8
    return choice, normals


class PointToPlane:
    def __init__(self, normal_search_radius: float) -> None:
        self._normal_search_radius = normal_search_radius
        self._kdtree: KDTree | None = None
        self._normals: np.ndarray | None = None
        self._normal_cache: list[np.ndarray | None] = []

    def reset(self, kdtree: KDTree, max_num_of_normals: int) -> None:
        self._kdtree = kdtree
        self._normal_cache = [None] * max_num_of_normals

    def get_normals(self) -> np.ndarray | None:
        return self._normals

    def estimate_rigid_body_transformation(
        self,
        source: np.ndarray,
        target: np.ndarray,
        indices: np.ndarray,
        choice: np.ndarray,
    ) -> Pose2d:
        if np.sum(choice) == 0:
            return Pose2d(0.0, 0.0, 0.0)

        if self._kdtree is None:
            return Pose2d(0.0, 0.0, 0.0)

        searched_target_points = target[indices]
        choice_normal, self._normals = _construct_normals(
            searched_target_points,
            [self._normal_cache[index] for index in indices],
            self._kdtree,
            self._normal_search_radius,
        )
        for i, index in enumerate(indices):
            self._normal_cache[index] = np.copy(self._normals[i])

        choice = choice & choice_normal
        choosen_source_points = source[choice]
        choosen_target_points = searched_target_points[choice]
        choosen_normals = self._normals[choice]

        # nx2x3
        jac_transform = Pose2d(0, 0, 0).jac(choosen_source_points)
        # nx3 <- nx2 @ nx2x3
        jac = np.einsum("ij,ijk->ik", choosen_normals, jac_transform)
        # nx2
        diff = choosen_source_points - choosen_target_points
        residual = np.einsum("ij,ij->i", choosen_normals, diff)
        # 3x3
        h_mat = np.linalg.inv(jac.transpose(1, 0) @ jac)
        # 3 <- 3x3 @ 3xn @ n
        update_vec = -np.einsum("ij,jk,k->i", h_mat, jac.transpose(1, 0), residual)
        return Pose2d(x=update_vec[0], y=update_vec[1], yaw=update_vec[2])
