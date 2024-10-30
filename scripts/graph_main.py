#!/usr/bin/env python3

from node_listener.graph_builder import GraphBuilder
import rclpy


def main():
    rclpy.init()
    minimal_subscriber = GraphBuilder()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 