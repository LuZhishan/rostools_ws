cmake_minimum_required(VERSION 3.5)
project(topic_pcd)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(pcd_to_topic src/pcd_to_topic.cpp)
target_link_libraries (pcd_to_topic ${PCL_LIBRARIES})
ament_target_dependencies(pcd_to_topic rclcpp sensor_msgs pcl_conversions)
add_executable(topic_to_pcd src/topic_to_pcd.cpp)
target_link_libraries (topic_to_pcd ${PCL_LIBRARIES})
ament_target_dependencies(topic_to_pcd rclcpp sensor_msgs pcl_conversions)

install(TARGETS
  pcd_to_topic
  topic_to_pcd

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
