cmake_minimum_required(VERSION 2.8.3)
project(arm_mimic_capstone)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  kinova_msgs
  kinova_driver
)


add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()



find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
  FILES
  HandPose.msg
  HandStampedPose.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  kinova_msgs
)



catkin_package(
  CATKIN_DEPENDS
    message_generation
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)



###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(myPlanner src/myPlanner.cpp)
target_link_libraries(myPlanner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS myPlanner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(myDummyPub src/myDummyPub.cpp)
target_link_libraries(myDummyPub ${catkin_LIBRARIES})
