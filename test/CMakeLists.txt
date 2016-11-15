# gtest part

#add_subdirectory(gtest)
include_directories(../src)

# A hack of gmock due to its conflict with catkin
set(gmock_SOURCE_DIR gtest/googlemock)
include_directories(${gmock_SOURCE_DIR}/include
                    ${gmock_SOURCE_DIR}
                    ${gmock_SOURCE_DIR}/../googletest/include
                    ${gmock_SOURCE_DIR}/../googletest)
add_library(gmock
            ${gmock_SOURCE_DIR}/../googletest/src/gtest-all.cc
            ${gmock_SOURCE_DIR}/src/gmock-all.cc
            ${gmock_SOURCE_DIR}/src/gmock_main.cc)
target_link_libraries(gmock pthread)
# Replace catkin variable
set(GTEST_LIBRARIES gmock)



add_executable(test_node test_node.cpp)
target_link_libraries(test_node wolf pthread gmock ) #gtest_main
add_test(test_node ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_node)

include_directories(/usr/include/eigen3)
add_executable(test_ceres test_ceres.cpp read_g2o.cc)
target_link_libraries(test_ceres wolf pthread gmock ) #gtest_main
add_test(test_ceres ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_ceres)
