# Install script for directory: /home/jsola/dev/labrobotica/algorithms/wolf/trunk/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms" TYPE SHARED_LIBRARY FILES "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/lib/libwolf.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/iri-algorithms/libwolf.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri-algorithms/wolf" TYPE FILE FILES "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/wolf.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri-algorithms/wolf" TYPE FILE FILES
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/capture_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/capture_relative.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/capture_fix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/capture_gps_fix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/capture_odom_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_sparse.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_gps_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_fix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_odom_2D_theta.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_odom_2D_complex_angle.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_corner_2D_theta.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/constraint_container.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/feature_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/feature_corner_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/feature_gps_fix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/feature_fix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/feature_odom_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/frame_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/landmark_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/landmark_corner_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/landmark_container.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/map_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/node_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/node_terminus.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/node_linked.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/sensor_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/sensor_laser_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/sensor_odom_2D.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/sensor_gps_fix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/state_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/state_point.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/state_orientation.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/state_quaternion.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/state_theta.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/state_complex_angle.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/time_stamp.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/trajectory_base.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/wolf_problem.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/wolf_manager.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri-algorithms/wolf/data_association" TYPE FILE FILES
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/data_association/matrix.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/data_association/association_solver.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/data_association/association_node.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/data_association/association_tree.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/data_association/association_nnls.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/iri-algorithms/wolf/ceres_wrapper" TYPE FILE FILES
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/ceres_wrapper/complex_angle_parametrization.h"
    "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/ceres_wrapper/ceres_manager.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/share/cmake-2.8/Modules/Findwolf.cmake")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/share/cmake-2.8/Modules" TYPE FILE FILES "/home/jsola/dev/labrobotica/algorithms/wolf/trunk/src/../Findwolf.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/jsola/dev/labrobotica/algorithms/wolf/trunk/build/src/solver/cmake_install.cmake")
  INCLUDE("/home/jsola/dev/labrobotica/algorithms/wolf/trunk/build/src/examples/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

