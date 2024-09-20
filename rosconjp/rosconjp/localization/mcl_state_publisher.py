#!/usr/bin/env python3
import copy
from typing import List

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Particle as ParticleMsg
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rosconjp.conversion import (
    create_occupancy_grid_map_msg,
    create_transform_stamped_msg_from_matrix,
    create_transform_stamped_msg_from_pose2d,
    pose2d_to_pose_msg,
    pose2d_to_pose_stamped_msg,
    pose_msg_to_pose2d,
)
from rosconjp.data import Pose2d
from rosconjp.localization.mcl import Particle
from rosconjp.map import DistanceMap
from rosconjp.profile import MapProfile
from sensor_msgs.msg import LaserScan
from tf2_ros.transform_broadcaster import TransformBroadcaster


class MCLStatePublisher:
    def __init__(self, node: Node) -> None:
        self._map_pub = node.create_publisher(
            OccupancyGrid, "/cost_map", qos_profile=MapProfile
        )
        self._pose_pub = node.create_publisher(
            PoseStamped,
            "/estimated_pose",
            10,
        )
        self._particles_pub = node.create_publisher(
            ParticleCloud,
            "/particles",
            10,
        )
        self._scan_pub = node.create_publisher(
            LaserScan,
            "/mcl/scan",
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(node)

    def publish(
        self,
        estimated_pose2d: Pose2d,
        odom_msg: Odometry,
        particles: List[Particle],
        scan_msg: LaserScan,
        stamp: Time,
    ) -> None:
        # publish estimated pose
        self._pose_pub.publish(
            pose2d_to_pose_stamped_msg(estimated_pose2d, "map", stamp)
        )
        # publish particles
        particles_msg = ParticleCloud()
        for particle in particles:
            particles_msg.particles.append(
                ParticleMsg(
                    pose=pose2d_to_pose_msg(particle.pose),
                    weight=particle.weight,
                )
            )
        particles_msg.header.frame_id = "map"
        self._particles_pub.publish(particles_msg)

        # publish mcl-based scan
        scan_msg_to_pub = copy.deepcopy(scan_msg)
        scan_msg_to_pub.header.frame_id = "mcl_laser_frame"
        self._scan_pub.publish(scan_msg_to_pub)

        # publish map_to_odom
        odom_pose2d = pose_msg_to_pose2d(odom_msg.pose.pose)
        map_to_odom_pose = estimated_pose2d.mat() @ np.linalg.inv(odom_pose2d.mat())
        map_to_odom = create_transform_stamped_msg_from_matrix(
            map_to_odom_pose, "map", "mcl_odom", stamp
        )
        self.tf_broadcaster.sendTransform(map_to_odom)

    def publish_distance_map(
        self, distance_map: DistanceMap, info: MapMetaData, stamp: Time
    ) -> None:
        distance_map_max_value = max(distance_map.data)
        distance_map_data = [
            int(d / distance_map_max_value * 127) for d in distance_map.data
        ]
        map_msg = create_occupancy_grid_map_msg(
            data=distance_map_data,
            map_info=info,
            frame_id="map",
            stamp=stamp,
        )
        self._map_pub.publish(map_msg)

    def publish_odom_to_base(self, odom_msg: Odometry) -> None:
        odom_to_base = create_transform_stamped_msg_from_pose2d(
            pose_msg_to_pose2d(odom_msg.pose.pose),
            "mcl_odom",
            "mcl_base_footprint",
            odom_msg.header.stamp,
        )
        self.tf_broadcaster.sendTransform(odom_to_base)
