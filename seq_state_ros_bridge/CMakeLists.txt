cmake_minimum_required(VERSION 3.0.2)
project(seq_state_ros_bridge)

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
  geometry_msgs
  control_msgs
  sensor_msgs
  message_generation
  nav_msgs
  dynamic_reconfigure
  tf
  sequence_player
  robot_hardware
  ${PR2_CONTROLLERS_MSGS_PACKAGE})

find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

find_package(hrpsys QUIET)
if(NOT ${hrpsys_FOUND})
  pkg_check_modules(hrpsys hrpsys-base REQUIRED)
endif()

add_message_files(FILES
  MotorStates.msg
  ContactState.msg
  ContactStateStamped.msg
  ContactStatesStamped.msg
)
add_service_files(FILES
  SetSensorTransformation.srv
)
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS roscpp actionlib geometry_msgs control_msgs sensor_msgs nav_msgs message_runtime dynamic_reconfigure tf sequence_player ${PR2_CONTROLLERS_MSGS_PACKAGE}
  DEPENDS openrtm_aist
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  ${hrpsys_INCLUDE_DIRS}
)

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  ${openrtm_aist_LIBRARY_DIRS}
)

add_executable(HrpsysSeqStateROSBridge2
  src/HrpsysSeqStateROSBridge2.cpp
  src/HrpsysSeqStateROSBridge2Impl.cpp
  src/HrpsysSeqStateROSBridge2Comp.cpp
)

set_target_properties(HrpsysSeqStateROSBridge2 PROPERTIES PREFIX "")
add_dependencies(HrpsysSeqStateROSBridge2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(HrpsysSeqStateROSBridge2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  HRPDataTypesSkel HRPDataTypesStub
)

install(TARGETS HrpsysSeqStateROSBridge2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

