cmake_minimum_required(VERSION 3.0.2)
project(joint_trajectory_ros_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
add_definitions(-Wno-deprecated)

find_package(catkin QUIET COMPONENTS
  pr2_controllers_msgs
)
if(pr2_controllers_msgs_FOUND)
  set(PR2_CONTROLLERS_MSGS_PACKAGE pr2_controllers_msgs)
  add_definitions("-DUSE_PR2_CONTROLLERS_MSGS")
endif()
message(STATUS "check for pr2 messages, will compile with ${PR2_CONTROLLERS_MSGS_PACKAGE}.")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sequence_player
  ${PR2_CONTROLLERS_MSGS_PACKAGE})

find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS roscpp actionlib control_msgs sequence_player ${PR2_CONTROLLERS_MSGS_PACKAGE}
  DEPENDS openrtm_aist
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  ${openrtm_aist_LIBRARY_DIRS}
)

add_executable(HrpsysJointTrajectoryBridge2
  src/HrpsysJointTrajectoryBridge2.cpp
  src/HrpsysJointTrajectoryBridge2Comp.cpp
)

set_target_properties(HrpsysJointTrajectoryBridge2 PROPERTIES PREFIX "")

target_link_libraries(HrpsysJointTrajectoryBridge2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
)

install(TARGETS HrpsysJointTrajectoryBridge2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

