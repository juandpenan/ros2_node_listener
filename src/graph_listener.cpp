#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

#include "node_listener/node_listener.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("graph_listener");

  auto graph_event = node->get_graph_event();

  rclcpp::QoS qos(1);

  std::string topic_name = "chatter";

  // Create a publisher and a subscription with incompatible QoS
  qos.best_effort();
  auto pub = node->create_publisher<example_interfaces::msg::String>(
    topic_name, qos);

  qos.reliable();
  auto data_callback =
      [node](example_interfaces::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(node->get_logger(), "I heard: [%s]", msg->data.c_str());
      };
  auto sub = node->create_subscription<example_interfaces::msg::String>(
    topic_name, qos, data_callback);

  while (rclcpp::ok())
  {
      node->wait_for_graph_change(graph_event, 100ms);
      bool triggered = graph_event->check_and_clear();
      if (triggered)
      {
          RCLCPP_INFO(node->get_logger(),
            "GRAPH_CHANGED:");
          RCLCPP_INFO(node->get_logger(),
            "    pub.count = %lu", node->count_publishers(topic_name));
          RCLCPP_INFO(node->get_logger(),
            "    sub.count = %lu", node->count_subscribers(topic_name));
          RCLCPP_INFO(node->get_logger(),
            "    nodes:");
          auto nodes = node->get_node_names();
          for (auto it = nodes.begin(); it != nodes.end(); it++)
          {
            RCLCPP_INFO(node->get_logger(),
              "        %s", it->c_str());
          }
      }
  }

  rclcpp::shutdown();
  return 0;
}
