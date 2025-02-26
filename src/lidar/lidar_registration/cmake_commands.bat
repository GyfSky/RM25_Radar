/usr/bin/cmake /home/thesky/RM_radardemo24/src/lidar/lidar_registration -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/thesky/RM_radardemo24/install/lidar_registration
/usr/bin/cmake --build /home/thesky/RM_radardemo24/build/lidar_registration -- -j16 -l16
/usr/bin/cmake --install /home/thesky/RM_radardemo24/build/lidar_registration