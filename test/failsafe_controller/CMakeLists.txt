get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

set(MODALITIES
  actuators
  control_group
  attitude_rate
  attitude
  # acceleration_hdg_rate
  # acceleration_hdg
  # velocity_hdg_rate
  # velocity_hdg
  # position
  )

add_executable(test_${TEST_NAME}
  test.cpp
  )

ament_target_dependencies(test_${TEST_NAME}
  rclcpp
  mrs_lib
  mrs_msgs
  mrs_uav_testing
  backward_ros
  )

install(TARGETS
  test_${TEST_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

foreach(MODALITY ${MODALITIES})

  add_ros_isolated_launch_test(test.py
    TIMEOUT 300
    TARGET "${TEST_NAME}_${MODALITY}"
    ARGS "modality:=${MODALITY}"
  )

endforeach()
