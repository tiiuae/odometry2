cmake_minimum_required(VERSION 3.5)
project(odometry2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fog_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(mrs_lib REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_library(odometry2 SHARED
  src/odometry2.cpp
  src/altitude_estimator.cpp
  src/heading_estimator.cpp
  src/lateral_estimator.cpp
  )

target_compile_definitions(odometry2
  PRIVATE "${PROJECT_NAME}_BUILDING_DLL")

ament_target_dependencies(odometry2
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs 
  std_srvs
  fog_msgs
  px4_msgs
  geometry_msgs 
  visualization_msgs
  tf2
  tf2_ros
  MAVSDK
  mrs_lib
  )

target_link_libraries(odometry2
  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_mission
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk
  )

rclcpp_components_register_nodes(odometry2 PLUGIN "${PROJECT_NAME}::Odometry2" EXECUTABLE odometry2)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
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

ament_package()
