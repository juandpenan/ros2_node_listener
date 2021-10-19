#include <cstdio>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

#include "node_listener/node_listener.hpp"

using namespace std::chrono_literals;

std::thread start_graph_thread(rclcpp::Node::SharedPtr node)
{
  auto thread_fn =
    [node]()
    {
      auto logger = node->get_logger();
      auto graph_event = node->get_graph_event();
      RCLCPP_INFO(logger, "GRAPH THREAD started...");
      while (rclcpp::ok())
      {
        node->wait_for_graph_change(graph_event, 100ms);
        bool triggered = graph_event->check_and_clear();
        if (triggered)
        {
            RCLCPP_INFO(logger,
              "GRAPH_CHANGED:");
            RCLCPP_INFO(logger,
              "    pub.count = %lu", node->count_publishers("chatter"));
            RCLCPP_INFO(logger,
              "    sub.count = %lu", node->count_subscribers("chatter"));
            RCLCPP_INFO(logger,
              "    nodes:");
            auto nodes = node->get_node_names();
            for (auto it = nodes.begin(); it != nodes.end(); it++)
            {
              RCLCPP_INFO(logger,
                "        %s", it->c_str());
            }
        }
      }
      RCLCPP_INFO(logger, "GRAPH THREAD stopped.");
    };
  return std::thread(thread_fn);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("graph_listener");

  rclcpp::QoS qos(1);
  size_t pub_count = 0;

  auto pub = node->create_publisher<example_interfaces::msg::String>(
    "chatter", qos);

  auto pub_timer = node->create_wall_timer(5s,
    [node, pub, &pub_count]() -> void
    {
      auto msg = std::make_unique<example_interfaces::msg::String>();
      msg->data = "Hello World: " + std::to_string(pub_count++);
      RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg->data.c_str());
      pub->publish(std::move(msg));
    });

  auto data_callback =
      [node](example_interfaces::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(node->get_logger(), "I heard: [%s]", msg->data.c_str());
      };
  auto sub = node->create_subscription<example_interfaces::msg::String>(
    "chatter", qos, data_callback);

  auto graph_thread = start_graph_thread(node);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  graph_thread.join();

  rclcpp::shutdown();
  return 0;
}
