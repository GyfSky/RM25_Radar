cmake_minimum_required(VERSION 3.14.4)

project(TRTInferSample_inCode)

#add_definitions(-std=c++17)

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=16000000")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall -fopenmp")
set(CUDA_GEN_CODE "-gencode=arch=compute_86,code=sm_86")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++17 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/../TRTInfer/include)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
#include_directories(/data/cuda/cuda-11.8/cuda/include)
#link_directories(/data/cuda/cuda-11.8/cuda/lib)
# cudnn
#include_directories(/data/cuda/cuda-11.8/cudnn/v8.0.4/include)
#link_directories(/data/cuda/cuda-11.8/cudnn/v8.0.4/lib64)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
)


cuda_add_executable(TRTInferSample ${PROJECT_SOURCE_DIR}/main.cpp
                              ${PROJECT_SOURCE_DIR}/../src/Inference.cpp
                              ${PROJECT_SOURCE_DIR}/../CUDA/preprocess.cu)
set_target_properties(TRTInferSample PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(TRTInferSample ${OpenCV_LIBRARIES})
target_link_libraries(TRTInferSample nvinfer)
target_link_libraries(TRTInferSample cudart)
target_link_libraries(TRTInferSample nvonnxparser)
target_link_libraries(TRTInferSample Eigen3::Eigen)
target_link_libraries(TRTInferSample cuda)
target_link_libraries(TRTInferSample ${TensorRT_LIBS})

# 在Linux系统下，多线程模块并非自动链接的，需要额外链接。
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    target_link_libraries(TRTInferSample ${CMAKE_THREAD_LIBS_INIT})
endif()

add_definitions(-O3 -pthread)