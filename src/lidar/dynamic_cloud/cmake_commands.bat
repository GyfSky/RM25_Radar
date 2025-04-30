/usr/bin/cmake /home/thesky/RM25_Radar/src/lidar/dynamic_cloud -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/thesky/RM25_Radar/install/dynamic_cloud
/usr/bin/cmake --build /home/thesky/RM25_Radar/build/dynamic_cloud -- -j16 -l16
/usr/bin/cmake --install /home/thesky/RM25_Radar/build/dynamic_cloud