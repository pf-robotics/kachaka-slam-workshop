#!/usr/bin/env python3
from typing import List, Tuple

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan as LaserScanMsg
from sensor_msgs.msg import PointCloud2 as PointCloud2Msg

from .data import Pose2d, Scan, ScanSetting


def pose_msg_to_pose2d(msg: PoseMsg) -> Pose2d:
    return Pose2d(
        msg.position.x,
        msg.position.y,
        Rotation.from_quat(
            np.array(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            ),
        ).as_euler("zyx")[0],
    )


def pose2d_to_pose_msg(pose2d: Pose2d) -> PoseMsg:
    rot = Rotation.from_euler("z", pose2d.yaw)
    quat = rot.as_quat()  # x, y, z, w

    msg = PoseMsg()
    msg.position.x = pose2d.x
    msg.position.y = pose2d.y
    msg.position.z = 0.0
    msg.orientation.x = quat[0]
    msg.orientation.y = quat[1]
    msg.orientation.z = quat[2]
    msg.orientation.w = quat[3]

    return msg


def pose2d_to_pose_stamped_msg(
    pose2d: Pose2d, frame_id: str, stamp: Time
) -> PoseStampedMsg:
    msg = PoseStampedMsg()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose = pose2d_to_pose_msg(pose2d)

    return msg


def transform_stamped_to_pose2d(transform: TransformStamped) -> Pose2d:
    yaw = Rotation.from_quat(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
    ).as_euler("zyx")[0]

    return Pose2d(
        x=transform.transform.translation.x,
        y=transform.transform.translation.y,
        yaw=yaw,
    )


def create_transform_stamped_msg_from_matrix(
    pose_matrix: np.ndarray,  # [4x4]
    parent_frame_id: str,
    child_frame_id: str,
    stamp: Time,
) -> TransformStamped:
    rot_3d = np.eye(3)
    rot_3d[:2, :2] = pose_matrix[:2, :2]
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = pose_matrix[0, 2]
    msg.transform.translation.y = pose_matrix[1, 2]
    msg.transform.translation.z = 0.0

    quaternion = Rotation.from_matrix(rot_3d).as_quat()
    msg.transform.rotation.x = quaternion[0]
    msg.transform.rotation.y = quaternion[1]
    msg.transform.rotation.z = quaternion[2]
    msg.transform.rotation.w = quaternion[3]
    return msg


def create_transform_stamped_msg_from_pose2d(
    pose2d: Pose2d,
    parent_frame_id: str,
    child_frame_id: str,
    stamp: Time,
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = pose2d.x
    msg.transform.translation.y = pose2d.y
    msg.transform.translation.z = 0.0

    quaternion = Rotation.from_euler("z", pose2d.yaw).as_quat()
    msg.transform.rotation.x = quaternion[0]
    msg.transform.rotation.y = quaternion[1]
    msg.transform.rotation.z = quaternion[2]
    msg.transform.rotation.w = quaternion[3]
    return msg


def create_occupancy_grid_map_msg(
    data: List[int],
    map_info: MapMetaData,
    frame_id: str,
    stamp: Time,
) -> OccupancyGrid:
    occupancy_grid_map = OccupancyGrid()
    occupancy_grid_map.header.frame_id = frame_id
    occupancy_grid_map.header.stamp = stamp
    occupancy_grid_map.info = map_info

    occupancy_grid_map.data = data
    return occupancy_grid_map


def laser_scan_msg_to_scan_and_setting(
    msg: LaserScanMsg, laser_to_base_pose: Pose2d | None
) -> Tuple[Scan, ScanSetting]:
    scan = Scan(
        ranges=msg.ranges,
        laser_to_base_pose=laser_to_base_pose
        if laser_to_base_pose is not None else Pose2d(0.0, 0.0, 0.0),
    )
    scan_setting = ScanSetting(
        angle_min=msg.angle_min,
        angle_max=msg.angle_max,
        angle_increment=msg.angle_increment,
        range_min=msg.range_min,
        range_max=msg.range_max,
    )

    return scan, scan_setting


def point_2d_array_to_pointcloud2_msg(
    points_2d: np.ndarray, parent_frame: str
) -> PointCloud2Msg:
    assert points_2d.ndim == 2 and points_2d.shape[1] == 2

    points = np.zeros((points_2d.shape[0], 3))
    points[:, :2] = points_2d

    from sensor_msgs.msg import PointField
    from std_msgs.msg import Header

    ros_dtype = PointField.FLOAT32
    numpy_dtype = np.float32
    itemsize = np.dtype(numpy_dtype).itemsize

    data = points.astype(numpy_dtype).tobytes()
    fields = [
        PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate("xyz")
    ]

    return PointCloud2Msg(
        header=Header(frame_id=parent_frame),
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=itemsize * 3,
        row_step=itemsize * 3 * points.shape[0],
        data=data,
    )
