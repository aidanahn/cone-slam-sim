from cone_interfaces.msg import Cone, ConeArray
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'cone_markers',
            qos_profile,
        )

        self.cone_subscription = self.create_subscription(
            ConeArray,
            'cone_map',
            self.cone_map_callback,
            qos_profile,
        )

    def cone_map_callback(self, message):
        blue_marker = self.create_marker(
            message,
            marker_id=0,
            red=0.0,
            green=0.0,
            blue=1.0,
        )

        yellow_marker = self.create_marker(
            message,
            marker_id=1,
            red=1.0,
            green=1.0,
            blue=0.0,
        )

        for cone in message.cones:
            point = Point()
            point.x = cone.x
            point.y = cone.y
            point.z = 0.0

            if cone.color == Cone.BLUE:
                blue_marker.points.append(point)
            elif cone.color == Cone.YELLOW:
                yellow_marker.points.append(point)

        marker_array = MarkerArray()
        marker_array.markers = [blue_marker, yellow_marker]

        self.marker_publisher.publish(marker_array)

    @staticmethod
    def create_marker(message, marker_id, red, green, blue):
        marker = Marker()

        marker.header = message.header
        marker.ns = 'cones'
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ConeVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()