cmake_minimum_required(VERSION 3.14.4)
project(newGraph_test)

set(TARGET_NAME jsoncpp_test)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O0  -Wall -fopenmp")
set(CUDA_GEN_CODE "-gencode=arch=compute_86,code=sm_86")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++14 -O0 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

add_library(General
        ../General/src/General.cpp
        ../General/include/General.h
)

#jsoncpp
#find_package(Jsoncpp REQUIRED)
target_include_directories(General PUBLIC /usr/local/json/)
target_link_libraries(General PUBLIC /usr/local/lib/libjsoncpp.a)

#opencv
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()  # 里面有find_package(CUDA)
find_package(OpenCV REQUIRED)
target_include_directories(General PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(General PUBLIC ${OpenCV_LIBRARIES})

# HikCamera
target_include_directories(General PUBLIC /opt/MVS/include/)
target_link_libraries(General PUBLIC /opt/MVS/lib/64/libMvCameraControl.so)

#yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(/usr/local/include/yaml-cpp)
target_link_libraries(General PUBLIC /usr/local/lib/libyaml-cpp.so)

#fmt
target_link_libraries(General PUBLIC fmt)

#eigen3
include_directories(/usr/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

#CUAD and tensorrt
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)
include_directories(/usr/local/cuda-12.6/targets/x86_64-linux/lib)
set_target_properties(General PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(General PUBLIC nvinfer)
# target_link_libraries(General PUBLIC cudart)
target_link_libraries(General PUBLIC /usr/local/cuda-12.6/targets/x86_64-linux/lib/libcudart.so)
target_link_libraries(General PUBLIC nvonnxparser)
target_link_libraries(General PUBLIC cuda)
target_link_libraries(General PUBLIC ${TensorRT_LIBS})


#其他
target_link_libraries(General PUBLIC dl pthread m)

# 在Linux系统下，多线程模块并非自动链接的，需要额外链接。
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(General PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

#其他
target_link_libraries(General PUBLIC dl pthread m)

link_libraries(General)

add_library(Mouse
        ../General/src/Mouse.cpp)


cuda_add_library(Net
        ../Net/src/Net.cpp
        ../Net/src/Inference.cpp
        ../Net/CUDA/preprocess.cu
)

add_library(Camera
        ../Camera_hk/src/Camera_mlt.cpp
        ../Camera_hk/src/HikCamera_old.cpp)

add_library(Image
        ../Image/src/Image.cpp)
target_link_libraries(Image PUBLIC Camera)

add_library(Locate
        ../Locate/src/CoordinateSystem.cpp
        ../Locate/src/KRepresent.cpp
        ../Locate/src/MapAOV.cpp
        ../Hero/src/CoordSolver.cpp
        )

#add_library(Detector
#        test/detector/src/Detector.cpp
#        test/detector/src/Hungarian.cpp
#        test/detector/src/KalmanTracker.cpp
#        test/detector/src/Sort.cpp
#
#)
#
#add_library(Deep_byteTrack
#        test/deep_byteTrack/Kalman/src/Kalman.cpp
#)

add_library(ByteTrack
        ../ByteTrack/src/BYTETracker.cpp
        ../ByteTrack/src/kalmanFilter.cpp
        ../ByteTrack/src/lapjv.cpp
        ../ByteTrack/src/utils.cpp
        ../ByteTrack/src/STrack.cpp
        ../ByteTrack/src/CostMatrix.cpp
        ../ByteTrack/src/PretreatObjs.cpp
)

add_executable(
        ${TARGET_NAME}
        test/test_new_graph.cpp
)




target_link_libraries(${TARGET_NAME} PUBLIC Locate Net Mouse Image ByteTrack)