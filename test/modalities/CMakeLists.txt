set(MODALITIES
  actuators
  control_group
  attitude_rate
  attitude
  acceleration_hdg_rate
  acceleration_hdg
  velocity_hdg_rate
  velocity_hdg
  position
  )

set(CONTROLLERS
  se3_controller
  mpc_controller
  )

catkin_add_executable_with_gtest(test_modalities
  test.cpp
  )

target_link_libraries(test_modalities
  ${catkin_LIBRARIES}
  )

add_dependencies(test_modalities
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

foreach(CONTROLLER ${CONTROLLERS})

  foreach(MODALITY ${MODALITIES})

    add_rostest(modalities.test ARGS controller:=${CONTROLLER} modality:=${MODALITY})

  endforeach()

endforeach()
