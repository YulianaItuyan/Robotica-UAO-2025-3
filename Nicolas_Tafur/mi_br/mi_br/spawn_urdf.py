#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path

class URDFPublisher(Node):
    def __init__(self):
        super().__init__('urdf_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_description', 10)

        urdf_path = Path(__file__).parent.parent / 'urdf' / 'URDF_LINKS.urdf'
        with open(urdf_path, 'r') as urdf_file:
            urdf_content = urdf_file.read()

        msg = String()
        msg.data = urdf_content
        self.publisher_.publish(msg)
        self.get_logger().info('URDF publicado en /robot_description')

def main(args=None):
    rclpy.init(args=args)
    node = URDFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

