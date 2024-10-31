#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from node_listener.dot_generator import RosGraph


class GraphBuilder(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Empty,
            'ros_graph_update',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ros_graph = RosGraph(self)
        self.last_dotcode = None
        
    def listener_callback(self, msg):
        self.ros_graph._update_rosgraph()
        self.last_dotcode = self.ros_graph._generate_dotcode()
        self.get_logger().info('new graph update')
        self.get_logger().info('I heard: "%s"' % self.last_dotcode)


def main(args=None):
    rclpy.init(args=args)
    node = GraphBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()