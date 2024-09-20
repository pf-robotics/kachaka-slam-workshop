import numpy as np
from rosconjp.data import Pose2d


class PointToPoint:
    def estimate_rigid_body_transformation(
        self,
        source: np.ndarray,
        target: np.ndarray,
        indices: np.ndarray,
        choice: np.ndarray,
    ) -> Pose2d:
        if np.sum(choice) == 0:
            return Pose2d(0.0, 0.0, 0.0)

        source_points = source[choice]
        target_points = target[indices[choice]]
        centroid_source_points = np.mean(source_points, axis=0)
        centroid_target_points = np.mean(target_points, axis=0)

        h_mat = np.dot(
            (source_points - centroid_source_points).T,
            target_points - centroid_target_points,
        )
        u_mat, _, vt_mat = np.linalg.svd(h_mat)

        rot = vt_mat.T @ u_mat.T

        if np.linalg.det(rot) < 0:
            vt_mat[-1, :] *= -1
            rot = vt_mat.T @ u_mat.T

        trans = -rot @ centroid_source_points + centroid_target_points

        return Pose2d.from_rot_and_trans(rot, trans)
