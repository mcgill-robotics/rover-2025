cmake_minimum_required(VERSION 3.8)
project(rover_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

add_executable(bt_ros_node src/bt_ros_node.cpp)
target_link_libraries(bt_ros_node
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  behaviortree_cpp::behaviortree_cpp
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS bt_ros_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()