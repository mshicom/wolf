
#Start WOLF build
MESSAGE("Starting WOLF CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#find dependencies. 
FIND_PACKAGE(Ceres QUIET) #Ceres is not required
IF(Ceres_FOUND)
    MESSAGE("Ceres Library FOUND: ceres_wrapper will be built.")
ENDIF(Ceres_FOUND)

#include directories
INCLUDE_DIRECTORIES(.)
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

#headers
SET(HDRS
    capture_base.h
    capture_gps_fix.h
    capture_odom_2D.h
    correspondence_base.h
    correspondence_sparse.h
    correspondence_gps_2D.h
    feature_base.h
    feature_gps_fix.h
    feature_odom_2D.h
    frame_base.h
    node_base.h
    node_terminus.h
    node_linked.h
    sensor_base.h
    state_base.h
    state_point.h
    state_complex_angle.h
    time_stamp.h
    wolf.h)
    
#sources
SET(SRCS
    capture_base.cpp
    capture_gps_fix.cpp
    capture_odom_2D.cpp
    correspondence_base.cpp
    correspondence_gps_2D.cpp
    feature_base.cpp
    feature_gps_fix.cpp
    feature_odom_2D.cpp
    frame_base.cpp
    node_base.cpp
    node_terminus.cpp
    sensor_base.cpp
    state_base.cpp
    state_complex_angle.cpp
    time_stamp.cpp)

#optional HDRS and SRCS
IF (Ceres_FOUND)
    SET(SRCS ${SRCS} ceres_wrapper/complex_angle_parametrization.cpp)
    SET(HDRS ${HDRS} ceres_wrapper/complex_angle_parametrization.h)
ENDIF(Ceres_FOUND)

# create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS})

#Link the created library with ceres
IF (Ceres_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES})
ENDIF(Ceres_FOUND)


#install library
INSTALL(TARGETS ${PROJECT_NAME}
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)
        
#install headers         
INSTALL(FILES ${HDRS} 
      DESTINATION include/iri-algorithms)

INSTALL(FILES ../Findwolf.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

#Build examples & tests
MESSAGE("Building examples and tests.")
ADD_SUBDIRECTORY(examples)