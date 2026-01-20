cmake_minimum_required(VERSION 3.5)
project(lidar_detection)

if(DEFINED HUMBLE_ROS)
  message(STATUS "HUMBLE_ROS variable is set to: ${HUMBLE_ROS}")
endif()

if(DEFINED ROS_EDITION)
  message(STATUS "ROS_EDITION variable is set to: ${ROS_EDITION}")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensing_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  vision_msgs
  sensing_msgs
  ament_index_cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleDetection.msg"
  "msg/ObstacleDetectionArray.msg"
  # "msg/BallRelative.msg"
  # "msg/RobotState.msg"
  # "msg/BallState.msg"
  DEPENDENCIES vision_msgs std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

## 
add_executable(obstacle_detector_node
  src/obstacle_detector.cpp
  src/utils.cpp
)
target_include_directories(obstacle_detector_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obstacle_detector_node ${dependencies})
target_link_libraries(obstacle_detector_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)

##
add_executable(odom_trans_node
  src/odom_to_baselink_transformer.cpp
  src/utils.cpp
)
target_include_directories(odom_trans_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(odom_trans_node ${dependencies})
target_link_libraries(odom_trans_node ${PCL_LIBRARIES} ${cpp_typesupport_target})
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(odom_trans_node ${cpp_typesupport_target})

## 
add_executable(obstacle_to_other_frame_node
  src/obstacle_to_other_frame_transformer.cpp
  src/utils.cpp
)
target_include_directories(obstacle_to_other_frame_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obstacle_to_other_frame_node ${dependencies})
target_link_libraries(obstacle_to_other_frame_node ${PCL_LIBRARIES} ${cpp_typesupport_target})
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(obstacle_to_baselink_node ${cpp_typesupport_target})

## 
add_executable(interface_node
  src/interface.cpp
)
target_include_directories(interface_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(interface_node ${dependencies})
target_link_libraries(interface_node ${PCL_LIBRARIES} ${cpp_typesupport_target})
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(interface_node ${cpp_typesupport_target})

##
add_executable(map_launcher_node
  src/map_launcher.cpp
)
target_include_directories(map_launcher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(map_launcher_node ${dependencies})
target_link_libraries(map_launcher_node ${PCL_LIBRARIES} ${cpp_typesupport_target})
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(map_launcher_node ${cpp_typesupport_target})

## 
install(TARGETS
  obstacle_detector_node
  odom_trans_node
  interface_node
  map_launcher_node
  obstacle_to_other_frame_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch rviz scripts config pcd DESTINATION share/${PROJECT_NAME})

# install(PROGRAMS
#   scripts/param_tuner.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()