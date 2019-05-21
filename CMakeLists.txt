cmake_minimum_required(VERSION 3.0)

## LIBSURVIVE
include(ExternalProject)
# ExternalProject_Add( libsurvive
#   PREFIX libsurvivelib
#   GIT_REPOSITORY https://github.com/cnlohr/libsurvive.git
#   BUILD_IN_SOURCE
#   BUILD_COMMAND cmake --build .
#   INSTALL_COMMAND ""
# )
#   
# find_path(SURVIVE_INCLUDE_DIRS libsurvive/libsurvive.h
#   HINTS ${SURVIVE_INCLUDEDIR})
# 
# find_library(SURVIVE_LIBRARIES
#   NAMES libsurvive
#   HINTS ${SURVIVE_LIBDIR})


ExternalProject_Add(deepdive
  GIT_REPOSITORY https://github.com/asymingt/libdeepdive
  GIT_TAG master
  BUILD_COMMAND cmake --build .
  INSTALL_COMMAND "")
ExternalProject_Get_Property(deepdive source_dir)
set(DEEPDIVE_INCLUDE_DIRS ${source_dir}/src)
ExternalProject_Get_Property(deepdive binary_dir)
set(DEEPDIVE_LIBRARIES ${binary_dir}/libdeepdive.so)


project(hive)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(gflags REQUIRED)
find_package(VISP REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ViveCalibration.msg
  ViveCalibrationGeneral.msg
  ViveCalibrationLighthouse.msg
  ViveCalibrationLighthouseArray.msg
  ViveCalibrationTracker.msg
  ViveCalibrationTrackerArray.msg
  ViveCalibrationTracker2.msg
  ViveCalibrationTrackerArray2.msg
  ViveExtrinsics.msg
  ViveLight.msg
  ViveLightSample.msg
  ViveMotor.msg
# New formats
  HiveData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ViveConfig.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES hive
  CATKIN_DEPENDS message_runtime
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${DEEPDIVE_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hive.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(hive_server src/vive_server.cc src/vive.cc src/vive_solve.cc src/vive_visualization.cc src/vive_calibrate.cc)
add_executable(hive_base_solve src/vive_base_solve.cc src/vive_base.cc src/vive.cc src/vive_solve.cc src/vive_visualization.cc)
add_executable(hive_base_calibrate src/vive_base_calibrate.cc src/vive.cc src/vive_solve.cc src/vive_visualization.cc)
add_executable(hive_print tools/vive_print.cc src/vive.cc src/vive_solve.cc src/vive_visualization.cc src/vive_calibrate.cc)
add_executable(hive_tool tools/vive_tool.cc)
add_executable(hive_optimize tools/vive_optimize.cc src/vive.cc src/vive_solve.cc)
add_executable(hive_bridge src/vive_bridge.cc)
add_executable(hive_beta tools/vive_beta.cc src/vive.cc src/vive_solve.cc)
add_executable(hive_offset tools/vive_offset.cc src/hive_offset.cc src/vive.cc src/hive_solver.cc)
add_executable(hive_print_offset tools/hive_print_offset.cc src/vive.cc src/hive_solver.cc)
add_executable(hive_refine src/vive_refine.cc src/vive.cc src/vive_solve.cc)
# add_executable(hive_filter src/vive_filter.cc src/vive.cc)
add_executable(hive_pgo src/vive_pgo.cc src/vive.cc)
add_executable(hive_analytics tools/hive_analytics.cc src/vive.cc src/hive_solver.cc)

add_executable(hive_calibrate tools/hive_calibrate.cc src/vive.cc src/vive_solve.cc src/hive_calibrator.cc)
add_executable(hive_solve tools/hive_solve.cc src/vive.cc src/vive_filter.cc src/hive_solver.cc)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hive_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hive_server hive_generate_messages_cpp)
add_dependencies(hive_base_solve hive_generate_messages_cpp)
add_dependencies(hive_base_calibrate hive_generate_messages_cpp)
add_dependencies(hive_print hive_generate_messages_cpp)
add_dependencies(hive_tool hive_generate_messages_cpp)
add_dependencies(hive_optimize hive_generate_messages_cpp)
add_dependencies(hive_bridge hive_generate_messages_cpp deepdive)
add_dependencies(hive_beta hive_generate_messages_cpp)
add_dependencies(hive_offset hive_generate_messages_cpp)
add_dependencies(hive_print_offset hive_generate_messages_cpp)
add_dependencies(hive_refine hive_generate_messages_cpp)
# add_dependencies(hive_filter hive_generate_messages_cpp)
add_dependencies(hive_pgo hive_generate_messages_cpp)
add_dependencies(hive_analytics hive_generate_messages_cpp)

add_dependencies(hive_calibrate hive_generate_messages_cpp)
add_dependencies(hive_solve hive_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hive_server
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)
target_link_libraries(hive_base_solve
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)
target_link_libraries(hive_base_calibrate
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)
target_link_libraries(hive_print
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)
target_link_libraries(hive_tool
  ${catkin_LIBRARIES}
  ${gflags_LIBRARIES}
)
target_link_libraries(hive_optimize
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)
target_link_libraries(hive_bridge
  ${catkin_LIBRARIES}
  ${DEEPDIVE_LIBRARIES}
)
target_link_libraries(hive_beta
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)
target_link_libraries(hive_offset
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${VISP_LIBRARIES}
)

target_link_libraries(hive_print_offset
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${CERES_LIBRARIES}
)

target_link_libraries(hive_analytics
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${CERES_LIBRARIES}
)

target_link_libraries(hive_refine
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

# target_link_libraries(hive_filter
#   ${catkin_LIBRARIES}
#   ${CERES_LIBRARIES}
#   ${EIGEN_LIBRARIES}
# )

target_link_libraries(hive_pgo
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

target_link_libraries(hive_calibrate
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

target_link_libraries(hive_solve
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

# add_executable(hive_solver src/hive_solver.cc src/vive.cc)
# add_dependencies(hive_solver hive_generate_messages_cpp)
# target_link_libraries(hive_solver
#   ${catkin_LIBRARIES}
#   ${CERES_LIBRARIES}
#   ${EIGEN_LIBRARIES}
# )
