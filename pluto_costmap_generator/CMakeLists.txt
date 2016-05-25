cmake_minimum_required(VERSION 2.8.3)
project(pluto_costmap_generator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

##find_package(OpenCV REQUIRED)
set (OpenCV_DIR ~/opencv/opencv3.0.0/share/OpenCV)
find_package(OpenCV 3.0 EXACT REQUIRED)

find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pluto_costmap_generator
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  ~/opencv/opencv3.0.0/include
  ~/opencv/opencv3.0.0/build/opencv_contrib/modules/ximgproc/include
  ~/opencv/opencv3.0.0/build/opencv_contrib/modules/xfeatures2d/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ~/opencv/opencv3.0.0/build/lib
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cloudbased_costmap_generator src/cloudbased_costmap_generator.cpp)

target_link_libraries(cloudbased_costmap_generator
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
