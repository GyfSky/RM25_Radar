/usr/bin/cmake /home/thesky/RM_radardemo24/src/RPS_Radar24 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/thesky/RM_radardemo24/install/lidar_registration
/usr/bin/cmake --build /home/thesky/RM_radardemo24/build/rps_radar24 -- -j16 -l16
/usr/bin/cmake --install /home/thesky/RM_radardemo24/build/rps_radar24