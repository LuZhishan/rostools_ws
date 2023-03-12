cmake_minimum_required(VERSION 3.5)
project(topic_republish)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sub_pub src/sub_pub.cpp)
ament_target_dependencies(sub_pub rclcpp sensor_msgs)

install(TARGETS
  sub_pub

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
