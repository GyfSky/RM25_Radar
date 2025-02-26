cmake_minimum_required(VERSION 3.11 )

project(PortTest)

set(set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_FLAGS "-std=c++17"))

add_executable(${PROJECT_NAME}
        main_testPort.cpp
        ../../src/CRC_Check.cpp
        ../../src/Content.cpp
        ../../src/serialport.cpp
)

#yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(/usr/local/include/yaml-cpp)
target_link_libraries(${PROJECT_NAME} /usr/local/lib/libyaml-cpp.so)

# 在Linux系统下，多线程模块并非自动链接的，需要额外链接。
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(${PROJECT_NAME}  ${CMAKE_THREAD_LIBS_INIT})
endif()

#其他
target_link_libraries(${PROJECT_NAME}  dl pthread m)
