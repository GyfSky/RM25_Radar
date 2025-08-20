 #=================cmakelist 最低版本==================
cmake_minimum_required(VERSION 3.14.4)
#=================  =====设置变量=======================
project(rps_radar24)
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

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        /opt/MVS/include/
        /usr/local/include/yaml-cpp
        /usr/local/cuda-12.6/targets/x86_64-linux/lib
        /usr/include/eigen3        #eigen3

#        /home/thesky/RM_radardemo24/install/#22.04 ros2 为install
#        /home/thesky/RM_radardemo24/install/interfaces/include/interfaces/interfaces/msg/
)

add_library(General src/General/src/General.cpp)


#=====================链接外部库========================
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(PCL REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# catkin_package(
#         #  INCLUDE_DIRS include
#         #  LIBRARIES beginner_tutorials
#                 CATKIN_DEPENDS sensor_msgs roscpp rospy std_msgs message_runtime
#         #  DEPENDS system_lib
#                 # pcl_ros
#         )
                
#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# ## make sure the livox_sdk_static library is installed
# find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a /usr/local/lib)

# if((NOT LIVOX_SDK_LIBRARY) OR (NOT EXISTS ${LIVOX_SDK_LIBRARY}))
# 	# couldn't find the livox sdk library
# 	message("Coudn't find livox sdk library!")
# 	message("Download Livox-SDK from github and build&install it please!")
# 	message("Download Livox-SDK from github and build&install it please!")
# else()
# 	message("find livox sdk library success")
# endif()

## PCL library
link_directories(${PCL_LIBRARY_DIRS})   # 有待考量ing
add_definitions(${PCL_DEFINITIONS})


#opencv
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()  # 里面有find_package(CUDA)
find_package(OpenCV 4.8.0 REQUIRED)
target_link_libraries(General PUBLIC ${OpenCV_LIBRARIES})

# HikCamera
target_link_libraries(General PUBLIC /opt/MVS/lib/64/libMvCameraControl.so.4.5.0.3)

## GalaxyCamera
#add_subdirectory(GalaxyCameraReader)
#target_include_directories(${PROJECT_NAME} PUBLIC  GalaxyCameraReader)
#target_link_libraries(${PROJECT_NAME} PUBLIC "GalaxyCameraReader")

#yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(General PUBLIC /usr/local/lib/libyaml-cpp.so)

#pcl
target_link_libraries(General PUBLIC ${PCL_LIBRARIES})
# target_link_libraries(General PUBLIC /usr/lib/x86_64-linux-gnu/libpcl_common.so)
# target_link_libraries(General PUBLIC /usr/local/lib/libpcl_common.so)

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
# target_include_directories(General PUBLIC ${catkin_INCLUDE_DIRS}) //          #must
#target_link_libraries(General PUBLIC rclcpp::rclcpp cv_bridge::cv_bridge ${std_msgs_TARGETS} ${interfaces_TARGETS} )
# target_link_libraries(General PUBLIC ${catkin_LIBRARIES})

#CUAD and tensorrt
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)
set_target_properties(General PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(General PUBLIC nvinfer)
# target_link_libraries(General PUBLIC cudart)
target_link_libraries(General PUBLIC /usr/local/cuda-12.6/targets/x86_64-linux/lib/libcudart.so)
target_link_libraries(General PUBLIC nvonnxparser)
target_link_libraries(General PUBLIC cudart)
target_link_libraries(General PUBLIC ${TensorRT_LIBS})

#############
## Testing ##
#############

link_libraries(General)


add_library(Mouse 
        src/General/src/Mouse.cpp)

add_library(SensorParam
		 src/General/src/SensorParam.cpp)

add_library(Camera 
        src/Camera_hk/src/Camera_mlt.cpp
        src/Camera_hk/src/HikCamera.cpp)


# add_library(Livox 
#         src/Livox/src/Mid70ros1.cpp)
# target_link_libraries(Livox PUBLIC Mouse)


add_library(Image 
        src/Image/src/Image.cpp
        src/Image/src/ImageStitch.cpp)
target_link_libraries(Image PUBLIC Camera)

add_library(Locate
        src/Locate/src/CoordinateSystem.cpp
        src/Locate/src/KRepresent.cpp
        src/Locate/src/Predict.cpp
        src/Locate/src/MapAOV.cpp
		src/Hero/src/CoordSolver.cpp
)       
target_link_libraries(Locate  ${CERES_LIBRARIES})


cuda_add_library(Net 
        src/Net/src/Net.cpp
        src/Net/src/Inference.cpp
        src/Net/src/Inference_changes.cpp
        src/Net/CUDA/preprocess.cu
        )


# add_executable(MouseTest  General/test/Mouse.cpp)
# target_link_libraries(MouseTest PUBLIC Mouse)


# add_executable(NetTest    Net/test/Net.cpp)
# target_link_libraries(NetTest PUBLIC Net Image)

# add_executable(LivoxTest src/Livox/test/Mid70_test.cpp)
# target_link_libraries(LivoxTest PUBLIC Livox)

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
		src/Port/src/Content.cpp
		src/Port/src/serialport.cpp
		src/Port/src/Timer.cpp
)

add_executable(RadarMain
        src/Radar/src/Radar.cpp
        src/Radar/src/main.cpp)

add_executable(calib_main
        src/Radar/test/main.cpp
		src/Radar/src/Radar.cpp)

add_executable(test_contract_img
	 src/Radar/test/main_test_contract_img.cpp)

# ament_auto_find_build_dependencies ()

ament_target_dependencies(RadarMain rclcpp interfaces std_msgs cv_bridge sensor_msgs Boost message_filters)
 ament_target_dependencies(calib_main rclcpp interfaces rclpy std_msgs Boost rosbag2 sensor_msgs PCL message_filters)
 ament_target_dependencies(test_contract_img rclcpp interfaces rclpy std_msgs Boost rosbag2 sensor_msgs PCL message_filters)
 ament_target_dependencies(General PUBLIC rclcpp cv_bridge std_msgs interfaces)



target_link_libraries(RadarMain  
SensorParam Image Locate Net Mouse ByteTrack Port cudart)
target_link_libraries(calib_main
SensorParam Image Locate Net Mouse ByteTrack Port cudart)
target_link_libraries(test_contract_img  
SensorParam Image Locate Net Mouse ByteTrack Port cudart)

install(TARGETS
		RadarMain
		calib_main
		DESTINATION lib/${PROJECT_NAME}
)

 install(DIRECTORY
		 DESTINATION share/${PROJECT_NAME}/
 )

# ament_package(
#         DEPENDS 
#         sensor_msgs 
#         rclcpp 
#         rclpy 
#         std_msgs 
# )

# ament_export_dependencies(rosidl_default_runtime)
 ament_auto_package()
