cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(subscriber)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GTest REQUIRED)

find_package(Ceres REQUIRED)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  pcl_msgs
  sensor_msgs
  distributed_mapper_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})  
include_directories("/usr/include/eigen3")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories( ${PCL_INCLUDE_DIRS})  
add_executable(subscriber subscriber.cpp)  
target_link_libraries(subscriber ${catkin_LIBRARIES})  
#target_link_libraries (subscriber ${PCL_LIBRARIES})
