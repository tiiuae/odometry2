cmake_minimum_required(VERSION 3.5)
project(odometry2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
add_definitions("-O3") # because normal release is just -O2

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fog_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                      odometry2 package                     |
## --------------------------------------------------------------

add_library(odometry2 SHARED
  src/odometry2.cpp
  )

ament_target_dependencies(odometry2
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs 
  fog_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

rclcpp_components_register_nodes(odometry2 
  PLUGIN "${PROJECT_NAME}::Odometry2" 
  EXECUTABLE odometry2)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
  odometry2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY plotjuggler
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
