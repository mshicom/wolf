# NodeLinked class test
ADD_EXECUTABLE(test_node_linked test_node_linked.cpp)
TARGET_LINK_LIBRARIES(test_node_linked ${CERES_LIBRARIES} ${PROJECT_NAME})

# numeric test
ADD_EXECUTABLE(test_ceres_wrapper_numeric test_ceres_wrapper_numeric.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_numeric ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test
ADD_EXECUTABLE(test_ceres_wrapper_jet test_ceres_wrapper_jet.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet individual blocks test
ADD_EXECUTABLE(test_ceres_wrapper_jet_ind_block test_ceres_wrapper_jet_ind_block.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet_ind_block ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test with 2 sensors
ADD_EXECUTABLE(test_ceres_wrapper_jet_2_sensors test_ceres_wrapper_jet_2_sensors.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet_2_sensors ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test different bla
ADD_EXECUTABLE(test_ceres_wrapper_state_units test_ceres_wrapper_state_units.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_state_units ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test manager
# ADD_EXECUTABLE(test_ceres_manager test_ceres_manager.cpp)
# TARGET_LINK_LIBRARIES(test_ceres_manager ${CERES_LIBRARIES} ${PROJECT_NAME})

# Testing a full wolf tree avoiding template classes for NodeLinked derived classes
ADD_EXECUTABLE(test_ceres_odom_batch test_ceres_odom_batch.cpp)
TARGET_LINK_LIBRARIES(test_ceres_odom_batch ${CERES_LIBRARIES} ${PROJECT_NAME})

# Testing a full wolf tree avoiding template classes for NodeLinked derived classes
ADD_EXECUTABLE(test_ceres_odom_iterative test_ceres_odom_iterative.cpp)
TARGET_LINK_LIBRARIES(test_ceres_odom_iterative ${CERES_LIBRARIES} ${PROJECT_NAME})
