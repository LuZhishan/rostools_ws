cmake_minimum_required(VERSION 3.5)
project(mul_cam_to_img)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5.5 REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sub src/sub.cpp)
target_link_libraries (sub ${OpenCV_LIBRARIES})
ament_target_dependencies(sub rclcpp sensor_msgs cv_bridge)

install(TARGETS
  sub
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
