cmake_minimum_required(VERSION 2.8.3)
project(ecn_calib)


## System dependencies are found with CMake's conventions
find_package(VISP REQUIRED)
add_compile_options(-std=c++11)
###########

## Specify additional locations of header files
include_directories(
  include
  ${VISP_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

add_library(calib SHARED
    include/generic_camera.h
    include/perspective_camera.h
    include/distortion_camera.h
    include/grid_tracker.h src/grid_tracker.cpp
    include/cb_tracker.h src/cb_tracker.cpp
    include/vvs.h src/vvs.cpp
    )

target_link_libraries(calib ${VISP_LIBRARIES})

## Declare a cpp executable
add_executable(calibration main/calibration.cpp)
target_link_libraries(calibration calib)

#add_executable(posecomputation main/posecomputation.cpp)
#target_link_libraries(posecomputation calib)

add_executable(record main/record.cpp)
target_link_libraries(record ${VISP_LIBRARIES})
