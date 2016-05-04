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

install(DIRECTORY
  launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/run_gazebo
  scripts/run_gazebo_paused
  scripts/run_gzclient
  scripts/run_gzserver
  scripts/run_gzserver_callgrind
  scripts/run_gzserver_cheats
  scripts/run_gzserver_gdb
  scripts/run_gzserver_memcheck
  scripts/run_gzserver_perf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

# Configure the setup script
configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
catkin_add_env_hooks(setup SHELLS sh DIRECTORY ${PROJECT_BINARY_DIR})

if(UNIX AND NOT APPLE)
  string(SUBSTRING "${CMAKE_INSTALL_PREFIX}" 0 8 PREFIX_START)
  if (${PREFIX_START} STREQUAL "/opt/ros")
    install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION /usr/share/frc2016_competition)
  else()
    install(FILES ${PROJECT_BINARY_DIR}/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/frc2016_competition)
  endif()
endif()


find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(ignition-msgs0 QUIET)
if (NOT ignition-msgs0_FOUND)
  message(FATAL_ERROR "Looking for ignition-msgs0-config.cmake - not found")
else()
  message(STATUS "Looking for ignition-msgs0-config.cmake - found")
endif()
include_directories(${IGNITION-MSGS_INCLUDE_DIRS})
link_directories(${IGNITION-MSGS_LIBRARY_DIRS})

find_package(ignition-transport1 QUIET)
if (NOT ignition-transport1_FOUND)
  message(FATAL_ERROR "Looking for ignition-transport1-config.cmake - not found")
else()
  message(STATUS "Looking for ignition-transport1-config.cmake - found")
endif()
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

add_subdirectory(plugins)

