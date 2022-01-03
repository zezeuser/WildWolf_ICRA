# CMake generated Testfile for 
# Source directory: /home/zeze/catkin_ws/src/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/cmake_import_minver
# Build directory: /home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/cmake_import_minver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cmake_import_minver_configure "/usr/bin/cmake" "-G" "Unix Makefiles" "-A" "" "-DCMAKE_CXX_COMPILER=/usr/bin/c++" "-Dnadjieb_mjpeg_streamer_DIR=/home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer" "/home/zeze/catkin_ws/src/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/cmake_import_minver/project")
set_tests_properties(cmake_import_minver_configure PROPERTIES  FIXTURES_SETUP "cmake_import_minver")
add_test(cmake_import_minver_build "/usr/bin/cmake" "--build" ".")
set_tests_properties(cmake_import_minver_build PROPERTIES  FIXTURES_REQUIRED "cmake_import_minver")
