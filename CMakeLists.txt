cmake_minimum_required(VERSION 2.8.3)

project(simplified_eye_hand_calibration)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_calib
CATKIN_DEPENDS roscpp geometry_msgs
#  DEPENDS system_lib
)

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
include
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${realsense2_INCLUDE_DIRS})


add_executable(getColorImage
src/captureRSCameraStream.cpp
src/captureRSCameraStream.hpp
src/main_RSCamera.cpp)
target_include_directories(getColorImage PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
)
target_link_libraries(getColorImage PRIVATE
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${catkin_LIBRARIES}
)

add_executable(eye_in_hand_calibration src/eye_in_hand_calibration.cpp src/solver.cpp)
target_link_libraries(eye_in_hand_calibration ${OpenCV_LIBRARIES})

add_executable(eye_to_hand_calibration src/eye_to_hand_calibration.cpp)
target_link_libraries(eye_to_hand_calibration ${OpenCV_LIBRARIES})