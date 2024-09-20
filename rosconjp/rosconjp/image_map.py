import pickle
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from .data import Pose2d


@dataclass
class ImageFeature:
    keypoints: list[cv2.KeyPoint]
    descriptors: np.ndarray


def extract_image_feature(image: np.ndarray) -> ImageFeature:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create()
    keypoints = orb.detect(gray, None)
    keypoints, descriptors = orb.compute(gray, keypoints)
    return ImageFeature(keypoints, descriptors)


class SearchableImageMap:
    def __init__(self, poses: list[Pose2d], images: list[ImageFeature]) -> None:
        self._images = images
        self._poses = poses
        self._matcher = cv2.BFMatcher()
        self._ratio = 0.8

    def search(self, image: ImageFeature) -> Pose2d | None:
        max_matched_keypoints: int | None = None
        max_index: int | None = None
        for i, db_image in enumerate(self._images):
            matches = self._matcher.knnMatch(
                image.descriptors, db_image.descriptors, k=2
            )
            len_matched_keypoints = len(
                [m for m, n in matches if m.distance < self._ratio * n.distance]
            )
            if len_matched_keypoints > 20:
                if max_matched_keypoints is None:
                    max_matched_keypoints = len_matched_keypoints
                    max_index = i
                else:
                    if max_matched_keypoints < len_matched_keypoints:
                        max_matched_keypoints = len_matched_keypoints
                        max_index = i

        if max_index is not None:
            return self._poses[max_index]
        else:
            return None


class ImageMap:
    def __init__(self, contents: dict[int, tuple[Pose2d, ImageFeature]]) -> None:
        self._dict: dict[int, tuple[Pose2d, ImageFeature]] = contents

    def index(self, p: Pose2d) -> int:
        return hash((p.x, p.y, p.yaw))

    def add(self, pose: Pose2d, image: ImageFeature) -> None:
        i = self.index(pose)
        self._dict[i] = (pose, image)

    @property
    def image_map(self) -> dict[int, tuple[Pose2d, ImageFeature]]:
        return self._dict

    def create_searchable_map(self) -> SearchableImageMap:
        poses = [pose for pose, _ in self._dict.values()]
        images = [image for _, image in self._dict.values()]
        return SearchableImageMap(poses, images)


def dump_image_map(save_dir: Path, image_map: ImageMap) -> None:
    for hash_value, (_, image) in image_map.image_map.items():
        with open(save_dir / f"image_feature_{hash_value}.pickle", "wb") as f:
            pickle.dump(
                {
                    "keypoint": [
                        {
                            "angle": kp.angle,
                            "class_id": kp.class_id,
                            "octave": kp.octave,
                            "pt": {
                                "x": float(kp.pt[0]),
                                "y": float(kp.pt[1]),
                            },
                            "response": kp.response,
                            "size": kp.size,
                        }
                        for kp in image.keypoints
                    ],
                    "descriptor": image.descriptors,
                },
                f,
            )

    contents = {
        "contents": [
            {
                "pose": {
                    "x": float(pose.x),
                    "y": float(pose.y),
                    "yaw": float(pose.yaw),
                },
                "image_hash": int(hash_value),
            }
            for hash_value, (pose, _) in image_map.image_map.items()
        ]
    }

    with open(save_dir / "image_map.yaml", "w") as f:
        yaml.safe_dump(contents, f)


def load_image_map(save_dir: Path) -> ImageMap:
    with open(save_dir / "image_map.yaml") as f:
        contents = yaml.safe_load(f)

    image_map_items: dict[int, tuple[Pose2d, ImageFeature]] = {}
    for pose_and_image_hash in contents["contents"]:
        pose = Pose2d(
            x=pose_and_image_hash["pose"]["x"],
            y=pose_and_image_hash["pose"]["y"],
            yaw=pose_and_image_hash["pose"]["yaw"],
        )
        image_hash = pose_and_image_hash["image_hash"]
        with open(save_dir / f"image_feature_{image_hash}.pickle", "rb") as f:
            image_features = pickle.load(f)

        keypoints = [
            cv2.KeyPoint(
                kp["pt"]["x"],
                kp["pt"]["y"],
                kp["size"],
                kp["angle"],
                kp["response"],
                kp["octave"],
                kp["class_id"],
            )
            for kp in image_features["keypoint"]
        ]
        descriptors = image_features["descriptor"]

        image_map_items[image_hash] = (pose, ImageFeature(keypoints, descriptors))

    return ImageMap(image_map_items)
