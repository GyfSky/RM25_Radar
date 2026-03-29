cmake_minimum_required(VERSION 3.8)
project(prepare_calib_cam_lidar)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(prepare_calib_cam_lidar src/prepare_calib_cam_lidar.cpp)
ament_target_dependencies(prepare_calib_cam_lidar rclcpp cv_bridge sensor_msgs OpenCV)

install(TARGETS
        prepare_calib_cam_lidar
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
