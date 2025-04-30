/usr/bin/cmake /home/thesky/RM25_Radar/src/fusion/kalman_filter -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=/home/thesky/RM25_Radar/install/kalman_filter
/usr/bin/cmake --build /home/thesky/RM25_Radar/build/kalman_filter -- -j16 -l16
/usr/bin/cmake --install /home/thesky/RM25_Radar/build/kalman_filter