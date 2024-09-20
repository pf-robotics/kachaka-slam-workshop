import shutil
from pathlib import Path

from nav2_msgs.srv import SaveMap
from rclpy.node import Node
from rosconjp.image_map import ImageMap, dump_image_map


class MapSaver:
    def __init__(self, node: Node) -> None:
        self._save_map_client = node.create_client(SaveMap, "/map_saver/save_map")
        self._node = node

    def save(self, output_dir: Path, image_map: ImageMap) -> None:
        if output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir()

        while not self._save_map_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info("service not available, waiting again...")

        req = SaveMap.Request()
        req.free_thresh = 0.2
        req.occupied_thresh = 0.65
        req.image_format = "pgm"
        req.map_mode = "trinary"
        req.map_url = f"{output_dir}/map"
        req.map_topic = "map"
        self._save_map_client.call_async(req)

        dump_image_map(output_dir, image_map)
