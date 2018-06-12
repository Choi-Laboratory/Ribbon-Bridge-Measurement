cmake_minimum_required(VERSION 2.8.3)
project(ribbon_bridge_measurement)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  darknet_ros_msgs
)

add_message_files(
    FILES
    RibbonBridge.msg
    RibbonBridges.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
    geometry_msgs
)

######################
#       OpenCV       #
######################

find_package(OpenCV REQUIRED
  NO_MODULE
  PATHS /usr/local
  NO_DEFAULT_PATH
)


include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ribbon_bridge_lsd_node src/ribbon_bridge_lsd.cpp)
target_link_libraries(ribbon_bridge_lsd_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ribbon_bridge_lsd_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(ribbon_bridge_corner_node src/ribbon_bridge_corner.cpp)
target_link_libraries(ribbon_bridge_corner_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ribbon_bridge_corner_node ${PROJECT_NAME}_generate_messages_cpp)
