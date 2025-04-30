cmake_minimum_required(VERSION 3.8)
project(dynamic_cloud)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

include_directories(include)

find_package(rclcpp REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DYNAMIC_CLOUD "dynamic_cloud")
file(GLOB_RECURSE DYNAMIC_CLOUD_SRCS "src/*.cpp")

ament_auto_add_library(${DYNAMIC_CLOUD} SHARED
        ${DYNAMIC_CLOUD_SRCS}
)
# target_link_libraries(${DYNAMIC_CLOUD} ${PCL_LIBRARIES})

rclcpp_components_register_node(${DYNAMIC_CLOUD}
        PLUGIN "upc_radar::DynamicCloud"
        EXECUTABLE ${DYNAMIC_CLOUD}_node
)

install(TARGETS
        dynamic_cloud
        DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
