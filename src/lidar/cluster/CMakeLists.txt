cmake_minimum_required(VERSION 3.8)
project(cluster)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

set(CLUSTER_NODE "cluster")
file(GLOB_RECURSE CLUSTER_NODE_SRCS "src/*.cpp")

ament_auto_add_library(${CLUSTER_NODE} SHARED
    ${CLUSTER_NODE_SRCS}
)

target_link_libraries(${CLUSTER_NODE} ${PCL_LIBRARIES} Open3D::Open3D Eigen3::Eigen TBB::tbb)

rclcpp_components_register_node(${CLUSTER_NODE}
    PLUGIN "upc_radar::Cluster"
    EXECUTABLE ${CLUSTER_NODE}_node
)

install(TARGETS cluster
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()