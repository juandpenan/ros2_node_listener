// Copyright 2021 Real-Time Innovations, Inc. (RTI)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

#include "node_listener/node_listener.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("events_listener");

  auto listener = std::make_shared<node_listener::EventsListener>(node);

  rclcpp::QoS qos(1);
  rclcpp::PublisherOptions pub_opts;
  rclcpp::SubscriptionOptions sub_opts;

  node_listener::install(listener, pub_opts, sub_opts);

  // Create a publisher and a subscription with incompatible QoS
  qos.best_effort();
  auto pub = node->create_publisher<example_interfaces::msg::String>(
    "chatter", qos, pub_opts);

  qos.reliable();
  auto data_callback =
      [node](example_interfaces::msg::String::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(node->get_logger(), "I heard: [%s]", msg->data.c_str());
      };
  auto sub = node->create_subscription<example_interfaces::msg::String>(
    "chatter", qos, data_callback, sub_opts);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
