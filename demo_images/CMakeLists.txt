cmake_minimum_required(VERSION 3.0.2)
project(demo_images)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  message_filters
  pcl_conversions 
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp 
                  rospy 
                  sensor_msgs 
                  std_msgs 
                  cv_bridge 
                  image_transport 
                  message_filters 
                  pcl_conversions 
                  pcl_ros
)

#boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${BOOST_LIBRARIES})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_subscriber_node_cpp
  src/image_subscriber_node_cpp.cpp
)
add_dependencies(image_subscriber_node_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_subscriber_node_cpp ${catkin_LIBRARIES})

add_executable(image_no_sync_subscriber_node
  src/image_no_sync_subscriber_node.cpp
)
add_dependencies(image_no_sync_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_no_sync_subscriber_node ${catkin_LIBRARIES})