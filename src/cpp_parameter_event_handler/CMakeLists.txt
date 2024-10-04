cmake_minimum_required(VERSION 3.8)
project(cpp_parameter_event_handler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(parameter_event_handler src/parameter_event_handler.cpp)
ament_target_dependencies(parameter_event_handler rclcpp)

install(TARGETS
  parameter_event_handler
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
