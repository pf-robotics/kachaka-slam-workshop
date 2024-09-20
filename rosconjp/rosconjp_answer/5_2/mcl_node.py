#!/usr/bin/env python3
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from rosconjp.conversion import laser_scan_msg_to_scan_and_setting, pose_msg_to_pose2d
from rosconjp.data import Pose2d, Scan
from rosconjp.image_map import SearchableImageMap, extract_image_feature, load_image_map
from rosconjp.localization.mcl import (
    MCL,
    MCLLikelihoodFieldModel,
    MCLLikelihoodFieldModelConfig,
    Movement,
)
from rosconjp.localization.mcl_state_publisher import MCLStatePublisher
from rosconjp.map import DistanceMapManager
from rosconjp.params import MCLParams
from rosconjp.profile import MapProfile, SensorProfile
from rosconjp.tf_utils import get_static_pose, init_buffer
from sensor_msgs.msg import CompressedImage, LaserScan
from std_srvs.srv import Trigger


class MCLNode(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("mcl_node")

        # parameters
        self.declare_parameter("odom_noise_a1", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("odom_noise_a2", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("odom_noise_a3", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("odom_noise_a4", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("resample_thresh", rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter("lfm_sigma", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("map_resolution", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("z_max", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("z_rand", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("z_hit", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("scan_step", rclpy.Parameter.Type.INTEGER)

        self.declare_parameter("use_global_localization", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("image_map_dir", rclpy.Parameter.Type.STRING)

        # MCL setting
        mcl_params = MCLParams(
            odom_noise_a1=self.get_parameter("odom_noise_a1")
            .get_parameter_value()
            .double_value,
            odom_noise_a2=self.get_parameter("odom_noise_a2")
            .get_parameter_value()
            .double_value,
            odom_noise_a3=self.get_parameter("odom_noise_a3")
            .get_parameter_value()
            .double_value,
            odom_noise_a4=self.get_parameter("odom_noise_a4")
            .get_parameter_value()
            .double_value,
            resample_thresh=self.get_parameter("resample_thresh")
            .get_parameter_value()
            .double_value,
        )
        mcl_likelihood_field_model_config = MCLLikelihoodFieldModelConfig(
            lfm_sigma=self.get_parameter("lfm_sigma")
            .get_parameter_value()
            .double_value,
            map_resolution=self.get_parameter("map_resolution")
            .get_parameter_value()
            .double_value,
            z_max=self.get_parameter("z_max").get_parameter_value().double_value,
            z_rand=self.get_parameter("z_rand").get_parameter_value().double_value,
            z_hit=self.get_parameter("z_hit").get_parameter_value().double_value,
            scan_step=self.get_parameter("scan_step")
            .get_parameter_value()
            .integer_value,
        )

        self._map_manager = DistanceMapManager()
        self._weigh_model = MCLLikelihoodFieldModel(
            config=mcl_likelihood_field_model_config,
            map_manager=self._map_manager,
        )
        self._model = MCL(
            weigh_model=self._weigh_model,
            mcl_params=mcl_params,
        )

        # publisher
        self._state_publisher = MCLStatePublisher(self)

        # image map
        self._image_map: SearchableImageMap | None = None
        if (
            self.get_parameter("use_global_localization")
            .get_parameter_value()
            .bool_value
        ):
            image_map_dir = Path(
                self.get_parameter("image_map_dir").get_parameter_value().string_value
            )
            self._image_map = load_image_map(image_map_dir).create_searchable_map()

        # last states
        self._last_scan_time: Optional[Time] = None
        self._last_odom: Optional[Odometry] = None
        self._last_image: np.ndarray | None = None

        # initial pose
        self._initial_pose = Pose2d(0.0, 0.0, 0.0)

        # initialize tf buffer
        init_buffer(self, publish_extra_static_tf=True)

        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, qos_profile=MapProfile
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            SensorProfile,
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            SensorProfile,
        )
        self._image_sub = self.create_subscription(
            CompressedImage,
            "/image",
            self._image_callback,
            SensorProfile,
        )
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_callback,
            1,
        )
        self._global_localize_srv = self.create_service(
            Trigger,
            "global_localize",
            self._on_global_localize,
        )

    def _map_callback(self, map: OccupancyGrid) -> None:
        """
        occupancy grid mapのコールバック関数
        """
        self.get_logger().info("waiting for initialization")

        self._map_manager.update_map(map)
        assert self._map_manager.distance_map

        self._model.initialize(self._initial_pose)

        self.get_logger().info("initialized")

        self._state_publisher.publish_distance_map(
            self._map_manager.distance_map,
            map.info,
            map.header.stamp,
        )

    def _odom_callback(self, odom_msg: Odometry) -> None:
        """
        odomのコールバック関数
        """
        self._last_odom = odom_msg

        # just publish state
        self._state_publisher.publish_odom_to_base(odom_msg)

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

        if self._last_odom is None:
            return

        scan_time = Time.from_msg(scan_msg.header.stamp)
        if self._last_scan_time is None:
            self._last_scan_time = scan_time
            return

        scan, scan_setting = laser_scan_msg_to_scan_and_setting(
            scan_msg,
            laser_to_base,
        )

        # 重みづけのモデルを更新
        self._weigh_model.update_scan_config(scan_setting)

        # MCLの実行
        self._run_mcl(self._last_odom, scan, scan_time, self._last_scan_time)

        self._last_scan_time = scan_time

        self._state_publisher.publish(
            self._model.estimate_pose(),
            self._last_odom,
            self._model.particles,
            scan_msg,
            scan_msg.header.stamp,
        )

    def _image_callback(self, image_msg: CompressedImage) -> None:
        """
        画像のコールバック関数
        """
        bridge = CvBridge()
        self._last_image = bridge.compressed_imgmsg_to_cv2(image_msg)

    def _initialpose_callback(self, initialpose: PoseWithCovarianceStamped) -> None:
        """
        initialposeのコールバック関数
        """
        self.get_logger().info("got initialpose")
        self._model.initialize(pose_msg_to_pose2d(initialpose.pose.pose))

    def _on_global_localize(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """
        global localizationのコールバック関数
        """
        if self._image_map is None:
            response.success = False
            response.message = "global localization is not supported"
            return response

        if self._last_image is None:
            response.success = False
            response.message = "no image subscriptions"
            return response

        self.get_logger().info("Into global localization")
        image_feature = extract_image_feature(self._last_image)

        pose = self._image_map.search(image_feature)
        if pose is None:
            response.success = False
            response.message = "image search is failed"
            return response

        self._model.initialize(pose)

        response.success = True
        response.message = ""
        return response

    def _run_mcl(
        self, odom: Odometry, scan: Scan, current_time: Time, prev_time: Time
    ) -> None:
        """
        MCLの実行
        """
        # オドメトリの計算
        d_dist = np.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2)
        d_rot = odom.twist.twist.angular.z
        dt = (current_time.nanoseconds - prev_time.nanoseconds) / 1e9
        movement = Movement(
            d_dist=d_dist * dt,
            d_rot=d_rot * dt,
        )

        # パーティクルの移動
        self._model.move_particles(movement)

        # パーティクルの重みづけ
        self._model.weigh_particles(scan)

        # リサンプリング
        self._model.resample()


def main() -> None:
    rclpy.init(args=None)

    node = MCLNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
