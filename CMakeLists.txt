cmake_minimum_required(VERSION 3.15)
project(mrs_uav_controllers)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_controllers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  pluginlib
  geometry_msgs
  sensor_msgs
  mrs_uav_managers
  Eigen3
  mrs_mpc_solvers
  mrs_uav_testing
)

set(LIBRARIES
  MrsUavControllers_MpcController
  MrsUavControllers_Se3Controller
  MrsUavControllers_FailsafeController
  MrsUavControllers_MidairActivationController
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_uav_managers_INCLUDE_DIRS}
  ${mrs_mpc_solvers_INCLUDE_DIRS}
  ${mrs_uav_testing_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# MPC Controller

add_library(MrsUavControllers_MpcController SHARED
  src/mpc_controller.cpp
  src/common.cpp
  )

ament_target_dependencies(MrsUavControllers_MpcController
  mrs_mpc_solvers
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavControllers_MpcController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# SE3 Controller

add_library(MrsUavControllers_Se3Controller SHARED
  src/se3_controller.cpp
  src/common.cpp
  )

ament_target_dependencies(MrsUavControllers_Se3Controller
  mrs_mpc_solvers
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavControllers_Se3Controller PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Failsafe Controller

add_library(MrsUavControllers_FailsafeController SHARED
  src/failsafe_controller.cpp
  src/common.cpp
  )

ament_target_dependencies(MrsUavControllers_FailsafeController
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavControllers_FailsafeController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Midair Activation Controller

add_library(MrsUavControllers_MidairActivationController SHARED
  src/midair_activation_controller.cpp
  src/common.cpp
  )

ament_target_dependencies(MrsUavControllers_MidairActivationController
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsUavControllers_MidairActivationController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
