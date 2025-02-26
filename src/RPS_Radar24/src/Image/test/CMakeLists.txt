#=================cmakelist 最低版本==================
cmake_minimum_required(VERSION 3.10)
#======================设置变量=======================
project(ImageStitch)
set(CMAKE_CXX_STANDARD 17)
set(TARGET_NAME ImageStitch)
add_executable(${TARGET_NAME}
        ../src/ImageStitch.cpp
        test_ImageStitch.cpp
) #main.cpp

#=====================链接外部库========================

#opencv
find_package(OpenCV REQUIRED)
target_include_directories(${TARGET_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} PUBLIC ${OpenCV_LIBRARIES})

#其他
target_link_libraries(${TARGET_NAME} PUBLIC dl pthread m)