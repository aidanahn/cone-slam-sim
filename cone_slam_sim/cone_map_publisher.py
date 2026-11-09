from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class ConeMapPublisher(Node):
    def __init__(self):
        super().__init__('cone_map_publisher')

        track_path = (
            Path(get_package_share_directory('cone_slam_sim')) 
            / 'tracks' 
            / 'fsi.yaml'
        )

        self.cones_left, self.cones_right = self.load_track(track_path)

    @staticmethod
    def load_track(track_path):
        with track_path.open('r', encoding='utf-8') as file:
            track = yaml.safe_load(file)

        return track['cones_left'], track['cones_right']


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ConeMapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()