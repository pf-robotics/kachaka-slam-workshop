#!/usr/bin/env python3
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.service
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rosconjp.conversion import (
    laser_scan_msg_to_scan_and_setting,
    point_2d_array_to_pointcloud2_msg,
    pose2d_to_pose_stamped_msg,
)
from rosconjp.data import Points2d, Pose2d, PoseWithPoints2d, scan_to_points2d
from rosconjp.image_map import ImageFeature, ImageMap, extract_image_feature
from rosconjp.mapping.icp import ICP
from rosconjp.mapping.map_saver import MapSaver
from rosconjp.mapping.mapping_state_publisher import MappingStatePublisher
from rosconjp.mapping.occupancy_grid_map_publisher import OccupancyGridMapPublisher
from rosconjp.params import ICPParams
from rosconjp.profile import SensorProfile
from rosconjp.tf_utils import get_static_pose, init_buffer
from rosconjp.voxel_map import VoxelMap
from rosconjp_interfaces.srv import SaveMap
from sensor_msgs.msg import CompressedImage, LaserScan


class MappingNode(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("mapping_node")

        # parameters
        self.declare_parameter("min_distance_to_match", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("num_iterations", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("use_reciprocal", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("normal_search_radius", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("icp_method", rclpy.Parameter.Type.STRING)

        # ICP setting
        icp_params = ICPParams(
            min_distance_to_match=self.get_parameter("min_distance_to_match")
            .get_parameter_value()
            .double_value,
            num_iter=self.get_parameter("num_iterations")
            .get_parameter_value()
            .integer_value,
            use_reciprocal=self.get_parameter("use_reciprocal")
            .get_parameter_value()
            .bool_value,
            normal_search_radius=self.get_parameter("normal_search_radius")
            .get_parameter_value()
            .double_value,
        )
        icp_method = self.get_parameter("icp_method").get_parameter_value().string_value
        self._icp: ICP = ICP(method=icp_method, config=icp_params)

        # publisher
        self._state_publisher = MappingStatePublisher(self)
        self._occupancy_grid_map_publisher = OccupancyGridMapPublisher(self)
        self._map_saver = MapSaver(self)

        # map
        self._accumulated_map = VoxelMap(0.01)
        self._pose_with_points_list: list[PoseWithPoints2d] = []

        # image map
        self._image_map = ImageMap({})

        # last states
        self._last_scan_points: Points2d | None = None
        self._last_scan_time_nanoseconds: int | None = None
        self._last_map_updated_nanoseconds: int | None = None
        self._last_odom: Odometry | None = None
        self._image_feature: ImageFeature | None = None

        # coordinate transformation
        self._odom_to_map = Pose2d(0.0, 0.0, 0.0)
        self._odom_pose = Pose2d(0.0, 0.0, 0.0)

        # initialize tf buffer
        init_buffer(self, publish_extra_static_tf=True)

        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, SensorProfile
        )
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, SensorProfile
        )
        self._image_sub = self.create_subscription(
            CompressedImage,
            "/camera_front/image_raw/compressed",
            self._image_callback,
            SensorProfile,
        )
        self._save_map_srv = self.create_service(
            SaveMap,
            "save_map",
            self._on_save_map,
        )

    def _update_map(self, scan_points: Points2d) -> None:
        """
        地図を更新するための関数
        """
        # 現在の補正項 (_odom_to_map) を用いて、bf_to_mapを一旦計算
        suggest_bf_to_map = self._odom_to_map.multiply(self._odom_pose)

        # voxel grid mapの点群
        map_points = Points2d(
            np.stack(
                [cell.point2d for cell in self._accumulated_map.voxel_map.values()]
            )
        )

        # 現在のbf_to_mapを初期姿勢に、mapの点群にscan点群を重ね合わせた時の姿勢変換を推定
        estimated_bf_to_map, scan_points_on_map = self._icp.estimate(
            scan_points, map_points, suggest_bf_to_map
        )

        # 推定した姿勢変換で、姿勢の補正項（odom -> map）を更新
        self._odom_to_map = estimated_bf_to_map.multiply(self._odom_pose.inv())

        # 推定した姿勢変換で地図を更新
        for p in scan_points_on_map:
            self._accumulated_map.add(p)

        # 画像特徴を推定姿勢とともに登録
        if self._image_feature is not None:
            self._image_map.add(estimated_bf_to_map, self._image_feature)

        # occupancy grid mapを更新するためのスキャンと姿勢のリストを更新
        self._pose_with_points_list.append(
            PoseWithPoints2d(
                pose=estimated_bf_to_map,
                points=scan_points,
            ),
        )

        # 状態をpublish
        self._state_publisher.publish(
            current_points=point_2d_array_to_pointcloud2_msg(scan_points.points, "map"),
            transformed_points=point_2d_array_to_pointcloud2_msg(
                scan_points_on_map, "map"
            ),
            map_points=point_2d_array_to_pointcloud2_msg(
                np.stack(
                    [cell.point2d for cell in self._accumulated_map.voxel_map.values()]
                ),
                "map",
            ),
            pose=pose2d_to_pose_stamped_msg(
                estimated_bf_to_map, "map", self.get_clock().now().to_msg()
            ),
        )

    def _scan_callback(self, scan_msg: LaserScan) -> None:
        """
        scanのコールバック関数
        """
        # laser_frame（LiDARの基準）とbase_footprint（ロボットの基準）の変換（固定）を取得
        laser_to_base = get_static_pose(
            "base_footprint", "laser_frame", extra_prefix="mcl_"
        )
        if laser_to_base is None:
            return

        # scanトピックをscan点群（base_footprint座標系）へ変換
        scan, scan_setting = laser_scan_msg_to_scan_and_setting(scan_msg, laser_to_base)
        scan_points = scan_to_points2d(scan, scan_setting)

        # scan time
        scan_time = Time.from_msg(scan_msg.header.stamp)

        # 初期化処理
        if self._last_scan_points is None or self._last_scan_time_nanoseconds is None:
            self._last_scan_points = scan_points
            self._last_scan_time_nanoseconds = int(scan_time.nanoseconds)
            for p in scan_points.points:
                self._accumulated_map.add(p)
            return

        # オドメトリが来るまでreturn
        if self._last_odom is None:
            return

        # 速度情報と、前回にscanコールバック関数が呼ばれた時間から、相対的な移動を推定
        dt = (scan_time.nanoseconds - self._last_scan_time_nanoseconds) / 1e9
        vx = self._last_odom.twist.twist.linear.x
        vy = self._last_odom.twist.twist.linear.y
        va = self._last_odom.twist.twist.angular.z
        odom = Pose2d(
            x=vx * dt,
            y=vy * dt,
            yaw=va * dt,
        )
        # ICPを使って、オドメトリによる相対的な移動を補正
        current_to_last, _ = self._icp.estimate(
            scan_points, self._last_scan_points, odom
        )

        # 相対的な移動変換を累積する
        self._odom_pose = self._odom_pose.multiply(current_to_last)

        # 地図の更新
        if (
                self._last_map_updated_nanoseconds is None
                or scan_time.nanoseconds - self._last_map_updated_nanoseconds > 1.0e9
        ):
            self._update_map(scan_points)
            self._last_map_updated_nanoseconds = scan_time.nanoseconds

        self._last_scan_points = scan_points
        self._last_scan_time_nanoseconds = int(scan_time.nanoseconds)

    def _odom_callback(self, odom: Odometry) -> None:
        """
        odomのコールバック関数
        """
        self._last_odom = odom

    def _image_callback(self, image_msg: CompressedImage) -> None:
        """
        画像のコールバック関数
        """
        bridge = CvBridge()
        image = bridge.compressed_imgmsg_to_cv2(image_msg)
        image_feature = extract_image_feature(image)
        self._image_feature = image_feature

    def _on_save_map(
        self,
        request: SaveMap.Request,
        response: SaveMap.Response,
    ) -> SaveMap.Response:
        """
        画像を保存するサービスのコールバック関数
        """
        self._occupancy_grid_map_publisher.publish(
            np.stack(
                [cell.point2d for cell in self._accumulated_map.voxel_map.values()]
            ),
            self._pose_with_points_list,
        )
        self._map_saver.save(Path(request.output_dir), self._image_map)

        response.success = True
        return response


def main() -> None:
    rclpy.init(args=None)

    node = MappingNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
