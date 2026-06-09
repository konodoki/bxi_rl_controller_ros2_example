cmake_minimum_required(VERSION 3.8)
project(realsense_depth_pub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(depth_publisher_node src/depth_publisher_node.cpp)

ament_target_dependencies(depth_publisher_node
  rclcpp
  sensor_msgs
)

if(TARGET realsense2::realsense2)
  target_link_libraries(depth_publisher_node realsense2::realsense2)
else()
  target_include_directories(depth_publisher_node PRIVATE ${realsense2_INCLUDE_DIR})
  target_link_libraries(depth_publisher_node ${realsense2_LIBRARY})
endif()

install(TARGETS depth_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
