cmake_minimum_required(VERSION 2.8.3)
project(mrs_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  cmake_modules
  mrs_msgs
  dynamic_reconfigure
  mrs_mav_manager
  message_generation
  mrs_lib
  )

generate_dynamic_reconfigure_options(
  cfg/pid_gains.cfg
  cfg/lsf_gains.cfg
  )

add_message_files(
  FILES
  debug.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs geometry_msgs mrs_msgs mrs_mav_manager mrs_lib
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# PID controller

add_library(PidController
  src/pid_controller/pid_controller.cpp
  )

add_dependencies(PidController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(PidController
  ${catkin_LIBRARIES}
  )

# LSF controller

add_library(LsfController
  src/lsf_controller/lsf_controller.cpp
  )

add_dependencies(LsfController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(LsfController
  ${catkin_LIBRARIES}
  )

# Failsafe controller

add_library(FailsafeController
  src/failsafe_controller/failsafe_controller.cpp
  )

add_dependencies(FailsafeController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(FailsafeController
  ${catkin_LIBRARIES}
  )
