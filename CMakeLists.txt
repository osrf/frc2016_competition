cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (frc2016_competition)
include(GNUInstallDirs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(ignition-msgs0 0.1 QUIET)
if (NOT ignition-msgs0_FOUND)
  message(FATAL "Looking for ignition-msgs0-config.cmake - not found")
else()
  message(STATUS "Looking for ignition-msgs0-config.cmake - found")
endif()

find_package(ignition-transport1 QUIET)
if (NOT ignition-transport1_FOUND)
  message(FATAL "Looking for ignition-transport1-config.cmake - not found")
else()
  message(STATUS "Looking for ignition-msgs0-config.cmake - found")
endif()

add_subdirectory(plugins)
