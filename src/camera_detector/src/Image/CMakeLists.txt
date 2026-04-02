#=================cmakelist 最低版本==================
cmake_minimum_required(VERSION 3.14.4)
#======================设置变量=======================
project(Get_Image)
set(CMAKE_CXX_STANDARD 17)
set(TARGET_NAME UnitTest_hk)
add_executable(${TARGET_NAME}
        src/Image.cpp
        ../General/src/General.cpp
        ../Locate/src/Place.cpp
        ../Camera_hk/src/HikCamera_old.cpp
        ../Camera_hk/src/Camera_mlt.cpp
) #main.cpp

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
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#eigen3
include_directories(/usr/include/eigen3)

#fmt
target_link_libraries(${TARGET_NAME} PUBLIC fmt)

#其他
target_link_libraries(${TARGET_NAME} PUBLIC dl pthread m)
# find_package(interfaces REQUIRED)
# include_directories(/home/thesky/RM_radardemo24/install/interfaces/include/interfaces/interfaces/msg)
# target_link_libraries(${TARGET_NAME} PUBLIC interfaces)
