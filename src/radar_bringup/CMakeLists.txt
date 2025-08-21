cmake_minimum_required(VERSION 3.8)
project(radar_bringup)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/define/define_template.py.in
        ${CMAKE_CURRENT_SOURCE_DIR}/define/define.py
        @ONLY
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  define
)
