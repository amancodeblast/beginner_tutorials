/**
 * @file subscriber_member_function.cpp
 * @author Aman Sharma (amankrsharma3@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <signal.h>

#include "../include/beginner_tutorials/MinimalSubscriber.hpp"

/**
 * @brief Construct a new Minimal Subscriber:: Minimal Subscriber object
 *
 */
MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

  if (this->count_publishers("topic") == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "No publishers on this topic to listen");
  }
}

void MinimalSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) const {
  RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->data);
}

/**
 * @brief node to be forced stop
 *
 * @param signum
 */
void node_forcestop(int signum) {
  if (signum == 2) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Force stopped! Bye!");
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                        "Call to end node worked ");
  }
}

int main(int argc, char* argv[]) {
  signal(SIGINT, node_forcestop);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
