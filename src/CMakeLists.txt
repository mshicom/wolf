
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
#    capture_base.h
    correspondence_base.h
    feature_base.h
    frame_base.h
    node_base.h
    node_terminus.h
    node_linked.h
    time_stamp.h
    wolf.h)
    
#sources
SET(SRCS
#    capture_base.cpp
    frame_base.cpp
    node_base.cpp
    node_terminus.cpp
    time_stamp.cpp)

#optional HDRS and SRCS
#IF (Ceres_FOUND)
#    SET(SRCS ${SRCS} ceres_wrapper/XXX.cpp)
#    SET(HDRS ${HDRS} ceres_wrapper/XXX.h)
#ENDIF(Ceres_FOUND)

# create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS})

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


