
#pragma once

#include <memory>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using STRING = std_msgs::msg::String;
/**
 * @brief Class (subclass of Node) and registers a member function as a callback
 * from the topic.
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber();

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  rclcpp::Subscription<STRING>::SharedPtr subscription_;
};
