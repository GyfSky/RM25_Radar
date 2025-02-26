#=================cmakelist 最低版本==================
cmake_minimum_required(VERSION 3.14.4)
#======================设置变量=======================
project(Camera_hk_mlt)
set(CMAKE_CXX_STANDARD 17)
set(TARGET_NAME UnitTest_hk)
add_executable(${TARGET_NAME} src/HikCamera.cpp src/Camera_mlt.cpp) #main.cpp

#=====================链接外部库========================

#opencv
find_package(OpenCV REQUIRED)
target_include_directories(${TARGET_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} PUBLIC ${OpenCV_LIBRARIES})

# HikCamera
target_include_directories(${TARGET_NAME}  PUBLIC /opt/MVS/include/)
target_link_libraries(${TARGET_NAME} PUBLIC /opt/MVS/lib/64/libMvCameraControl.so)

#yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(/usr/local/include/yaml-cpp)
target_link_libraries(${TARGET_NAME} PUBLIC /usr/local/lib/libyaml-cpp.so)

#pcl
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})