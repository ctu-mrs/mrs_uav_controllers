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

catkin_add_executable_with_gtest(test_${TEST_NAME}
  test.cpp
  )

target_link_libraries(test_${TEST_NAME}
  ${catkin_LIBRARIES}
  )

add_dependencies(test_${TEST_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

foreach(MODALITY ${MODALITIES})

  add_rostest(${TEST_NAME}.test
    ARGS
    modality:=${MODALITY}
    )

endforeach()
