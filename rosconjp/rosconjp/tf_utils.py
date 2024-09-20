#!/usr/bin/env python3
from typing import Optional

from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.time import Time
from rosconjp.conversion import transform_stamped_to_pose2d
from rosconjp.data import Pose2d
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformStamped
from tf2_ros.transform_listener import TransformListener

_tf_buffer: Optional[Buffer] = None
_tf_listener: Optional[TransformListener] = None
_tf_static_broadcaster: Optional[StaticTransformBroadcaster] = None


def init_buffer(node: Node, publish_extra_static_tf: bool = False) -> None:
    global _tf_buffer
    global _tf_listener
    global _tf_static_broadcaster
    if not _tf_buffer:
        _tf_buffer = Buffer()
        _tf_listener = TransformListener(_tf_buffer, node)
    if publish_extra_static_tf:
        _tf_static_broadcaster = StaticTransformBroadcaster(node)


def buffer() -> Buffer:
    if not _tf_buffer:
        raise RuntimeError("buffer is not initialized.")
    return _tf_buffer


def lookup_tf(
    target_frame: str,
    source_frame: str,
    time: Time,
    timeout: float = 10,
) -> Optional[TransformStamped]:
    try:
        return buffer().lookup_transform(
            target_frame=target_frame,
            source_frame=source_frame,
            time=time,
            timeout=Duration(seconds=timeout),
        )
    except Exception as e:
        get_logger("tf_utils").error(f"Could not transform {e}")
        return None


_static_poses: dict[tuple[str, str], Pose2d] = {}


def get_static_pose(
    target_frame: str, source_frame: str, extra_prefix: str = ""
) -> Pose2d | None:
    key = (target_frame, source_frame)
    if key in _static_poses:
        return _static_poses[key]

    transform_stamped = lookup_tf(target_frame, source_frame, Time())
    if transform_stamped is None:
        return None

    if extra_prefix != "" and _tf_static_broadcaster is not None:
        transform_stamped.header.frame_id = (
            extra_prefix + transform_stamped.header.frame_id
        )
        transform_stamped.child_frame_id = (
            extra_prefix + transform_stamped.child_frame_id
        )
        _tf_static_broadcaster.sendTransform(transform_stamped)

    pose2d = transform_stamped_to_pose2d(transform_stamped)
    _static_poses[key] = pose2d
    return pose2d
