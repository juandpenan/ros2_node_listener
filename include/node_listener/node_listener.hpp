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

#ifndef NODE_LISTENER__NODE_LISTENER_HPP_
#define NODE_LISTENER__NODE_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"

namespace node_listener
{
class EventsListener
{
public:
  EventsListener(rclcpp::Node::SharedPtr node)
    : node_(node)
  { }

  void on_deadline_offered_missed(rclcpp::QOSDeadlineOfferedInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "DEADLINE_OFFERED_MISSED: total=%d, total_change=%d",
      e.total_count,
      e.total_count_change);
  }

  void on_liveliness_lost(rclcpp::QOSLivelinessLostInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "LIVELINESS_LOST: total=%d, total_change=%d",
      e.total_count,
      e.total_count_change);
  }

  void on_offered_incompatible_qos(rclcpp::QOSOfferedIncompatibleQoSInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "OFFERED_INCOMPATIBLE_QOS: total=%d, total_change=%d, last_policy=%d",
      e.total_count,
      e.total_count_change,
      e.last_policy_kind);
  }

  void on_deadline_requested_missed(rclcpp::QOSDeadlineRequestedInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "DEADLINE_REQUESTED_MISSED: total=%d, total_change=%d",
      e.total_count,
      e.total_count_change);
  }

  void on_liveliness_changed(rclcpp::QOSLivelinessChangedInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "LIVELINESS_CHANGED: alive=%d, alive_change=%d, not_alive=%d, not_alive_change=%d",
      e.alive_count,
      e.alive_count_change,
      e.not_alive_count,
      e.not_alive_count_change);
  }

  void on_requested_incompatible_qos(rclcpp::QOSRequestedIncompatibleQoSInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "REQUESTED_INCOMPATIBLE_QOS: total=%d, total_change=%d, last_policy=%d",
      e.total_count,
      e.total_count_change,
      e.last_policy_kind);
  }

  void on_message_lost(rclcpp::QOSMessageLostInfo & e)
  {
    RCLCPP_INFO(node_->get_logger(),
      "MESSAGE_LOST: total=%lu, total_change=%lu",
      e.total_count,
      e.total_count_change);
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

void install(
  std::shared_ptr<EventsListener> listener,
  rclcpp::PublisherOptions & pub_opts,
  rclcpp::SubscriptionOptions & sub_opts)
{
  pub_opts.event_callbacks.deadline_callback =
    [listener](rclcpp::QOSDeadlineOfferedInfo & e) {
      listener->on_deadline_offered_missed(e);
    };

  pub_opts.event_callbacks.liveliness_callback =
    [listener](rclcpp::QOSLivelinessLostInfo & e) {
      listener->on_liveliness_lost(e);
    };

  pub_opts.event_callbacks.incompatible_qos_callback =
    [listener](rclcpp::QOSOfferedIncompatibleQoSInfo & e) {
      listener->on_offered_incompatible_qos(e);
    };

  sub_opts.event_callbacks.deadline_callback =
    [listener](rclcpp::QOSDeadlineRequestedInfo & e) {
      listener->on_deadline_requested_missed(e);
    };

  sub_opts.event_callbacks.liveliness_callback =
    [listener](rclcpp::QOSLivelinessChangedInfo & e) {
      listener->on_liveliness_changed(e);
    };

  sub_opts.event_callbacks.incompatible_qos_callback =
    [listener](rclcpp::QOSRequestedIncompatibleQoSInfo & e) {
      listener->on_requested_incompatible_qos(e);
    };

  sub_opts.event_callbacks.message_lost_callback =
    [listener](rclcpp::QOSMessageLostInfo & e) {
      listener->on_message_lost(e);
    };
}
}  // namespace node_listener

#endif  // NODE_LISTENER__NODE_LISTENER_HPP_
