cmake_minimum_required(VERSION 2.8.3)
project(roomba_hardware)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic)

find_package(libcreate REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_message_files(
  FILES
  Bumper.msg
  ChargingState.msg
  DefineSong.msg
  Mode.msg
  PlaySong.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    diagnostic_msgs
    diagnostic_updater
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
  DEPENDS libcreate
)

roslint_cpp()
roslint_add_test()

include_directories(
  include
  ${libcreate_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(create_driver src/create_driver.cpp)

target_link_libraries(create_driver
  ${libcreate_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(create_driver ${catkin_EXPORTED_TARGETS} roomba_hardware_generate_messages_cpp)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
if (${ARCHITECTURE} MATCHES "arm")
  add_executable(compass src/compass.cpp)
  target_link_libraries(compass ${catkin_LIBRARIES} -lwiringPi)
endif()

