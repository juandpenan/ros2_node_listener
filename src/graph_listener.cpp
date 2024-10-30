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
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

std::thread start_graph_thread(rclcpp::Node::SharedPtr node)
{
  auto thread_fn =
    [node]()
    {
      auto logger = node->get_logger();
      auto graph_event = node->get_graph_event();
  // size_t pub_count = 0;

      auto pub = node->create_publisher<std_msgs::msg::Empty>(
        "ros_graph_update", 1);
      RCLCPP_INFO(logger, "GRAPH THREAD started...");
      while (rclcpp::ok())
      {
        node->wait_for_graph_change(graph_event, 100ms);
        bool triggered = graph_event->check_and_clear();
        if (triggered)
        {
          RCLCPP_INFO(logger, "GRAPH THREAD triggered...");
          auto msg = std::make_unique<std_msgs::msg::Empty>();
          pub->publish(std::move(msg));
        }
        rclcpp::sleep_for(750ms);
      }
    };
  return std::thread(thread_fn);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("graph_listener");

  auto graph_thread = start_graph_thread(node);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  graph_thread.join();

  rclcpp::shutdown();
  return 0;
}
