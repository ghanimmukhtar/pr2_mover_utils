cmake_minimum_required(VERSION 2.8.3)
project(pr2_mover_utils)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

## Generate services in the 'srv' folder
add_service_files(
        FILES
        move_pr2_arm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pr2_mover_utils
  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pr2_mover_utils
        src/pr2_mover.cpp
        src/libs/helpers_methods.cpp
        )
target_link_libraries(pr2_mover_utils ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS pr2_mover_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

