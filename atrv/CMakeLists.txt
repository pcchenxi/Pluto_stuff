cmake_minimum_required(VERSION 2.8.3)
project(atrv)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf angles dynamic_reconfigure diagnostic_updater)

include_directories(include ${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/ATRV.cfg)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf angles dynamic_reconfigure diagnostic_updater
)

#add_library(rflex src/rflex_driver.cc)

add_library(atrv src/atrv.cc)
add_executable(atrvnode src/atrvnode.cc)
add_dependencies(atrvnode ${PROJECT_NAME}_gencfg)
target_link_libraries(atrvnode atrv  ${catkin_LIBRARIES})

install(TARGETS atrv atrvnode 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
