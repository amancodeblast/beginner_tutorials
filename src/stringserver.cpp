/**
 * @file stringserver.cpp
 * @author Aman Sharma (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/custom_msg.hpp"
#include "string"
#include <memory>
using ModifyString = beginner_tutorials::srv::CustomMsg;

void add(const std::shared_ptr<ModifyString::Request> request,
         std::shared_ptr<ModifyString::Response> response) {
  response->f = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request!!");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]",
              response->f.c_str());
}
/**
 * @brief main function to initiate the server
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("stringserver");

  rclcpp::Service<ModifyString>::SharedPtr service =
      node->create_service<ModifyString>("add_two_strings", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add strings");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
