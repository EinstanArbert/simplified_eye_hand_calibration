cmake_minimum_required(VERSION 2.8.3)

project(simplified_eye_hand_calibration)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

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

)

add_executable(eye_in_hand_calibration src/eye_in_hand_calibration.cpp)
target_link_libraries(eye_in_hand_calibration ${OpenCV_LIBRARIES})

add_executable(eye_to_hand_calibration src/eye_to_hand_calibration.cpp)
target_link_libraries(eye_to_hand_calibration ${OpenCV_LIBRARIES})