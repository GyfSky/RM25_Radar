cmake_minimum_required(VERSION 3.8)
project(lidar_registration)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

include_directories(include)

# find dependencies
find_package(OpenMP REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

set(LIDAR_REGISTRATION "lidar_registration")
file(GLOB_RECURSE LIDAR_REGISTRATION_SRCS
    "src/*.cpp"
)

ament_auto_add_library(${LIDAR_REGISTRATION} SHARED
    ${LIDAR_REGISTRATION_SRCS}
)
target_link_libraries(${LIDAR_REGISTRATION} ${PCL_LIBRARIES})

rclcpp_components_register_node(${LIDAR_REGISTRATION}
    PLUGIN "upc_radar::LidarRegistration"
    EXECUTABLE ${LIDAR_REGISTRATION}_node
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
