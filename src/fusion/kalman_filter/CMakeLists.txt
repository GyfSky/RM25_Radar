cmake_minimum_required(VERSION 3.8)
project(kalman_filter)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()

include_directories(include)

set(KALMAN_FILTER "kalman_filter")
file(GLOB_RECURSE KALMAN_FILTER_SRCS
    "src/*.cpp"
)

ament_auto_add_library(${KALMAN_FILTER} SHARED
    ${KALMAN_FILTER_SRCS}
)
target_link_libraries(${KALMAN_FILTER} ${PCL_LIBRARIES})

rclcpp_components_register_node(${KALMAN_FILTER}
    PLUGIN "upc_radar::KalmanFilter"
    EXECUTABLE ${KALMAN_FILTER}_node
)
install(TARGETS
        kalman_filter
        DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package ()
