 #=================cmakelist 最低版本==================
cmake_minimum_required(VERSION 3.14.4)
#=================  =====设置变量=======================
project(camera_detector)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O0  -Wall -fopenmp")
set(CUDA_GEN_CODE "-gencode=arch=compute_86,code=sm_86")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++17 -O0 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDRADAR24_ROSED")


add_library(General src/General/src/General.cpp)


#=====================链接外部库========================
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
		${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
		${TensorRT_INCLUDE_DIRS}
		/opt/MVS/include/
		${YAML_CPP_INCLUDE_DIR}
)

#opencv
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()  # 里面有find_package(CUDA)

find_package(OpenCV REQUIRED)
target_link_libraries(General PUBLIC ${OpenCV_LIBRARIES})

# HikCamera
target_link_libraries(General PUBLIC /opt/MVS/lib/64/libMvCameraControl.so)

#yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(General PUBLIC yaml-cpp)

#boost
target_link_libraries(General PUBLIC ${Boost_LIBRARY})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

#fmt
find_package(fmt REQUIRED)
target_link_libraries(General PUBLIC fmt)

# 在Linux系统下，多线程模块并非自动链接的，需要额外链接。
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(General PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

#其他
target_link_libraries(General PUBLIC dl pthread m)

#carkin
target_include_directories(General PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

#CUAD and tensorrt
find_package(CUDA REQUIRED)
set_target_properties(General PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(General PUBLIC nvinfer)
target_link_libraries(General PUBLIC cuda)
target_link_libraries(General PUBLIC nvonnxparser)
target_link_libraries(General PUBLIC cudart)
target_link_libraries(General PUBLIC ${TensorRT_LIBS})


link_libraries(General)


add_library(Mouse 
        src/General/src/Mouse.cpp)

add_library(SensorParam
		 src/General/src/SensorParam.cpp)

add_library(Camera 
        src/Camera_hk/src/Camera_mlt.cpp
        src/Camera_hk/src/HikCamera.cpp)

add_library(Image 
        src/Image/src/Image.cpp)
target_link_libraries(Image PUBLIC Camera)

add_library(Locate
        src/Locate/src/CoordinateSystem.cpp
        src/Locate/src/MapAOV.cpp
)
target_link_libraries(Locate  ${CERES_LIBRARIES})

cuda_add_library(Net 
        src/Net/src/Net.cpp
        src/Net/src/Inference.cpp
        src/Net/CUDA/preprocess.cu
        )

add_library(ByteTrack
		src/ByteTrack/src/BYTETracker.cpp
		src/ByteTrack/src/kalmanFilter.cpp
		src/ByteTrack/src/lapjv.cpp
		src/ByteTrack/src/utils.cpp
		src/ByteTrack/src/STrack.cpp
		src/ByteTrack/src/CostMatrix.cpp
		src/ByteTrack/src/PretreatObjs.cpp
)

add_library(Port
		src/Port/src/CRC_Check.cpp
		src/Port/src/Port.cpp
		src/Port/src/serialport.cpp
		src/Port/src/Timer.cpp
)

add_executable(camera_main
        src/Radar/src/Radar.cpp
        src/Radar/src/main.cpp)

add_executable(calib_main
        src/Radar/calib/main.cpp
		src/Radar/src/Radar.cpp)


# ament_auto_find_build_dependencies ()

ament_target_dependencies(camera_main rclcpp interfaces std_msgs cv_bridge sensor_msgs Boost message_filters)
ament_target_dependencies(calib_main rclcpp interfaces rclpy std_msgs Boost rosbag2 sensor_msgs message_filters)
ament_target_dependencies(General PUBLIC rclcpp cv_bridge std_msgs interfaces)


target_link_libraries(camera_main
SensorParam Image Locate Net Mouse ByteTrack Port cudart)
target_link_libraries(calib_main
SensorParam Image Locate Net Mouse ByteTrack Port cudart)

install(TARGETS
		camera_main
		calib_main
		DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
		DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
