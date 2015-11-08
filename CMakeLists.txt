cmake_minimum_required(VERSION 2.8.3)
project(usma_velodyne)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy 
  nodelet 
  angles 
  sensor_msgs 
  urdf 
  tf 
  pcl_ros
  pcl_conversions
)

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  gazebo_ros_gpu_velodyne
  
  CATKIN_DEPENDS 
  message_generation 
  gazebo_msgs 
  roscpp 
  nodelet 
  angles 
  sensor_msgs 
  urdf 
  tf 
  DEPENDS 
    gazebo 
    SDF
  )
add_library(gazebo_ros_gpu_velodyne src/gazebo_ros_gpu_velodyne.cpp)
target_link_libraries(gazebo_ros_gpu_velodyne ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} GpuRayPlugin)
###
### Add your new plugin here
###
