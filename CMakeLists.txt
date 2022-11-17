cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(srv_files
  "CustomMsg.srv"
  )
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CustomMsg.srv"
)

add_executable(talker src/new_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)


add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(server src/stringserver.cpp)
ament_target_dependencies(server rclcpp std_msgs)

install(TARGETS
  talker
  listener
  server
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(server 
${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(talker 
${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

ament_package()
