cmake_minimum_required(VERSION 2.8.3)
project(mrs_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  cmake_modules
  mrs_msgs
  dynamic_reconfigure
  mrs_uav_manager
  mrs_lib
  tf
  )

generate_dynamic_reconfigure_options(
  cfg/nsf_controller.cfg
  cfg/so3_controller.cfg
  cfg/attitude_controller.cfg
  cfg/mpc_controller.cfg
  cfg/acceleration_controller.cfg
  cfg/partial_landing_controller.cfg
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MpcControllerCvxgen
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_msgs mrs_uav_manager mrs_lib tf
  DEPENDS Eigen
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# CVX wrapper

add_library(MpcControllerCvxgen src/cvx_wrapper/cvx_wrapper.cpp
  src/cvx_wrapper/cvxgen/ldl.c
  src/cvx_wrapper/cvxgen/matrix_support.c
  src/cvx_wrapper/cvxgen/solver.c
  src/cvx_wrapper/cvxgen/util.c
  )

# NSF controller

add_library(NsfController
  src/nsf_controller/nsf_controller.cpp
  )

add_dependencies(NsfController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(NsfController
  ${catkin_LIBRARIES}
  )

# SO3 controller

add_library(So3Controller
  src/so3_controller/so3_controller.cpp
  )

add_dependencies(So3Controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(So3Controller
  ${catkin_LIBRARIES}
  )

# Mpc controller

add_library(MpcController
  src/mpc_controller/mpc_controller.cpp
  )

add_dependencies(MpcController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(MpcController
  ${catkin_LIBRARIES}
  MpcControllerCvxgen
  )

# acceleration controller

add_library(AccelerationController
  src/acceleration_controller/acceleration_controller.cpp
  )

add_dependencies(AccelerationController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(AccelerationController
  ${catkin_LIBRARIES}
  MpcControllerCvxgen
  )

# Attitude controller

add_library(AttitudeController
  src/attitude_controller/attitude_controller.cpp
  )

add_dependencies(AttitudeController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(AttitudeController
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

# Partial Landing Controller

add_library(PartialLandingController
  src/partial_landing_controller/partial_landing_controller.cpp
  )

add_dependencies(PartialLandingController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PartialLandingController
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

install(TARGETS
  MpcController
  So3Controller
  AccelerationController
  FailsafeController
  AttitudeController
  NsfController
  PartialLandingController
  MpcControllerCvxgen
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install(FILES
  plugins.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
