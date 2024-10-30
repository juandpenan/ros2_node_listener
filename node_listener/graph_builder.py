#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rqt_graph.ros_graph import RosGraph


class GraphBuilder(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Empty,
            'graph_listener',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ros_graph = RosGraph(None)
        self.last_dotcode = None
        
    def listener_callback(self, msg):
        self.last_dotcode = self.ros_graph._generate_dotcode()
        self.get_logger().info('new graph update')
        self.get_logger().info('I heard: "%s"' % self.last_dotcode)


