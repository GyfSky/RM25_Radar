cmake_minimum_required(VERSION 3.10)

project(calib_two_cam)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TARGET_NAME ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(${TARGET_NAME}
        src/main.cpp
        src/ImageStitch.cpp
)

target_include_directories(${TARGET_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${TARGET_NAME} rclcpp)

target_link_libraries(${TARGET_NAME}
        ${OpenCV_LIBRARIES}
        Threads::Threads
        ${CMAKE_DL_LIBS}
        m
)

install(TARGETS
        ${TARGET_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
