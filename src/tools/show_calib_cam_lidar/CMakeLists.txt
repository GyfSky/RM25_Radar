cmake_minimum_required(VERSION 3.8)
project(show_calib_cam_lidar)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(show_calib_cam_lidar src/show_calib_cam_lidar.cpp)
ament_target_dependencies(show_calib_cam_lidar rclcpp sensor_msgs PCL pcl_ros OpenCV)

install(TARGETS
        show_calib_cam_lidar
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
