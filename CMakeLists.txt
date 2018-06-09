cmake_minimum_required(VERSION 2.8.3)
project(Ribbon-Bridge-Measurement)

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
  darknet_ros_msgs
)

catkin_package()


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

add_executable(${PROJECT_NAME}_node src/boat_measurement_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
