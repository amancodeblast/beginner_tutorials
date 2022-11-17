/**
 * @file new_publisher.cpp
 * @author Aman Sharma(amankrsharma3@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-11-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using sharedFuture =
    rclcpp::Client<beginner_tutorials::srv::CustomMsg>::SharedFuture;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() // Object for node is created using a constructor
      : Node("minimal_publisher"), count_(0) {
    // Declaring parameter
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);
    // Fetching value from the parameter server
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG(this->get_logger(),
                 "Declared parameter freq and set to 2.0 hz");

    // Creating a subscriber for Parameter
    // and setting up call back to change frequency
    m_param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    RCLCPP_DEBUG(this->get_logger(), "Parameter event Handler  is Created");
    auto paramCallbackPtr =
        std::bind(&MinimalPublisher::param_callback, this, _1);
    m_paramHandle_ =
        m_param_subscriber_->add_parameter_callback("freq", paramCallbackPtr);

    // Creating a Publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG(this->get_logger(), "Publisher is Created");
    auto period = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        period, std::bind(&MinimalPublisher::timer_callback, this));

    // Creating a Client
    client =
        this->create_client<beginner_tutorials::srv::CustomMsg>("custom_msg");
    RCLCPP_DEBUG(this->get_logger(), "Client is Created");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        exit(EXIT_FAILURE);
      }
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }
  }

private:
  // Variables
  std::string Msg = "Aman";
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Client<beginner_tutorials::srv::CustomMsg>::SharedPtr client;

  size_t count_;
  std::shared_ptr<rclcpp::ParameterEventHandler> m_param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_paramHandle_;
  /**
   * @brief Method that runs after every 1000ms (Set as base frequency)
   *
   */
  void timer_callback() {
    // C++ stream style
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "The Node Has been Setup");
    auto message = std_msgs::msg::String();
    message.data = "Hello, " + Msg + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Soft dev : '%s'", message.data.c_str());
    publisher_->publish(message);
    if (count_ % 10 == 0) {
      call_service();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steady_clock, 10000,
                                 "Node Health full");
  }

  /**
   * @brief Method to call service by sending a request
   *
   * @return int
   */
  int call_service() {
    auto request =
        std::make_shared<beginner_tutorials::srv::CustomMsg::Request>();
    request->a = "Aman Sharma";
    request->b = "Client server working";
    RCLCPP_INFO(this->get_logger(), "String change using service");
    auto callbackPtr =
        std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }
  /**
   * @brief Callback to update the base string if there is response from server
   *
   * @param future
   */
  void response_callback(sharedFuture future) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "String Received is here: %s",
                future.get()->f.c_str());
    Msg = future.get()->f.c_str();
  }
  /**
   * @brief Callback Function to handle Parameter calls
   *
   * @param param
   */
  void param_callback(const rclcpp::Parameter &param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());
    RCLCPP_WARN(this->get_logger(), "You have changed the base frequency this "
                                    "might affect some functionality");

    RCLCPP_FATAL_EXPRESSION(this->get_logger(), param.as_double() == 0.0,
                            "Frequency cannot be zero. Runtime error!!");
    if (param.as_double() == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency unchanged");
    } else {
      auto period = std::chrono::milliseconds(
          static_cast<int>((1000 / param.as_double())));
      timer_ = this->create_wall_timer(
          period, std::bind(&MinimalPublisher::timer_callback, this));
    }
  }
};

/**
 * @brief Main function Entrypoint for program
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}