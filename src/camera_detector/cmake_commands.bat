/usr/bin/cmake /home/thesky/RM25_Radar/src/RPS_Radar24 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/thesky/RM25_Radar/install/rps_radar24
/usr/bin/cmake --build /home/thesky/RM25_Radar/build/rps_radar24 -- -j16 -l16
/usr/bin/cmake --install /home/thesky/RM25_Radar/build/rps_radar24