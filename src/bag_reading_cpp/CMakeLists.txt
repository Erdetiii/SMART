cmake_minimum_required(VERSION 3.8)
project(bag_reading_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(simple_bag_reader src/simple_bag_reader.cpp)
ament_target_dependencies(simple_bag_reader rclcpp rosbag2_transport sensor_msgs)

add_executable(simple_bag_reader_test src/simple_bag_reader_test.cpp)
ament_target_dependencies(simple_bag_reader_test rclcpp rosbag2_transport sensor_msgs)

install(TARGETS
  simple_bag_reader
  simple_bag_reader_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
