from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cone_interfaces.msg import Cone, ConeArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


class ConeMapPublisher(Node):
    def __init__(self):
        super().__init__('cone_map_publisher')

        track_path = (
            Path(get_package_share_directory('cone_slam_sim'))
            / 'tracks'
            / 'FSI.yaml'
        )

        self.cones_left, self.cones_right = self.load_track(track_path)

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(
            ConeArray,
            'cone_map',
            qos_profile,
        )

        self.publish_cone_map()

    def publish_cone_map(self):
        message = ConeArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'

        for x, y in self.cones_left:
            cone = Cone()
            cone.x = float(x)
            cone.y = float(y)
            cone.color = Cone.BLUE

            message.cones.append(cone)

        for x, y in self.cones_right:
            cone = Cone()
            cone.x = float(x)
            cone.y = float(y)
            cone.color = Cone.YELLOW

            message.cones.append(cone)

        self.publisher.publish(message)

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