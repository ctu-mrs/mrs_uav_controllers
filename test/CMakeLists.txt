find_package(rostest REQUIRED)

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

# Se3Controller modalities

foreach(MODALITY ${MODALITIES})

  add_rostest_gtest(Se3Controller_${MODALITY}
    se3_controller/modalities/se3_controller_modalities_${MODALITY}.test
    se3_controller/modalities/test.cpp
    )

  target_link_libraries(Se3Controller_${MODALITY}
    ${catkin_LIBRARIES}
    )

  add_dependencies(Se3Controller_${MODALITY}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

endforeach()

# MpcController modalities

foreach(MODALITY ${MODALITIES})

  add_rostest_gtest(MpcController_${MODALITY}
    mpc_controller/modalities/mpc_controller_modalities_${MODALITY}.test
    mpc_controller/modalities/test.cpp
    )

  target_link_libraries(MpcController_${MODALITY}
    ${catkin_LIBRARIES}
    )

  add_dependencies(MpcController_${MODALITY}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

endforeach()

# FailsafeController modalities

set(MODALITIES
  actuators
  control_group
  attitude_rate
  attitude
  acceleration_hdg_rate
  acceleration_hdg
  velocity_hdg_rate
  velocity_hdg
  )

foreach(MODALITY ${MODALITIES})

  add_rostest_gtest(FailsafeController_${MODALITY}
    failsafe_controller/modalities/failsafe_controller_modalities_${MODALITY}.test
    failsafe_controller/modalities/test.cpp
    )

  target_link_libraries(FailsafeController_${MODALITY}
    ${catkin_LIBRARIES}
    )

  add_dependencies(FailsafeController_${MODALITY}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

endforeach()
