cmake_minimum_required(VERSION 3.8)
project(gazebo_grasp_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(gazebo_grasp_fix SHARED
  src/gazebo_grasp_fix.cpp
)

target_compile_features(gazebo_grasp_fix PUBLIC cxx_std_17)

ament_target_dependencies(gazebo_grasp_fix
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
)

install(TARGETS gazebo_grasp_fix
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

