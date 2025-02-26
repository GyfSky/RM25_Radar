cmake_minimum_required(VERSION 3.14.4)
project(Hik_demo_in)

find_package(OpenCV REQUIRED)

add_executable(
        ${PROJECT_NAME}
        main_test.cpp
        ../src/HikCamera.cpp
)



target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})

## GalaxyCamera
#add_subdirectory(GalaxyCameraReader)
#target_include_directories(${PROJECT_NAME} PUBLIC  GalaxyCameraReader)
#target_link_libraries(${PROJECT_NAME} PUBLIC "GalaxyCameraReader")

# HikCamera

target_include_directories(${PROJECT_NAME} PUBLIC ${MVCAM_SDK_PATH}/include )
target_include_directories(${PROJECT_NAME} PUBLIC /opt/MVS/include/)
target_link_libraries(${PROJECT_NAME}  /opt/MVS/lib/64/libMvCameraControl.so )
target_link_libraries(${PROJECT_NAME}  ${LD_LIBRARY_PATH} )

# 在Linux系统下，多线程模块并非自动链接的，需要额外链接。
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(${PROJECT_NAME}  ${CMAKE_THREAD_LIBS_INIT})
endif()

#其他
target_link_libraries(${PROJECT_NAME}  dl pthread m)

#-----------------------------------------------------------------------------------------------------------------------

add_executable(
        MultipleCamera
        ../src/MultipleCamera.cpp
        main_multipleCamera.cpp
)

target_include_directories(MultipleCamera PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(MultipleCamera  ${OpenCV_LIBS})

# HikCamera
target_include_directories(MultipleCamera PUBLIC ${MVCAM_SDK_PATH}/include )
target_include_directories(MultipleCamera PUBLIC /opt/MVS/include/)
target_link_libraries(MultipleCamera  /opt/MVS/lib/64/libMvCameraControl.so )
target_link_libraries(MultipleCamera  ${LD_LIBRARY_PATH} )
target_include_directories(MultipleCamera PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(MultipleCamera ${OpenCV_LIBS})
# 在Linux系统下，多线程模块并非自动链接的，需要额外链接。
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(MultipleCamera ${CMAKE_THREAD_LIBS_INIT})
endif()

#其他
target_link_libraries(MultipleCamera  dl pthread m)

