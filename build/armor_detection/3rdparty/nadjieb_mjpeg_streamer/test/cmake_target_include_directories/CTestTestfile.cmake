# CMake generated Testfile for 
# Source directory: /home/zeze/catkin_ws/src/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/cmake_target_include_directories
# Build directory: /home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/cmake_target_include_directories
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cmake_target_include_directories_configure "/usr/bin/cmake" "-G" "Unix Makefiles" "-DCMAKE_CXX_COMPILER=/usr/bin/c++" "-Dnadjieb_mjpeg_streamer_source=/home/zeze/catkin_ws/src/armor_detection/3rdparty/nadjieb_mjpeg_streamer" "/home/zeze/catkin_ws/src/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/cmake_target_include_directories/project")
set_tests_properties(cmake_target_include_directories_configure PROPERTIES  FIXTURES_SETUP "cmake_target_include_directories")
add_test(cmake_target_include_directories_build "/usr/bin/cmake" "--build" ".")
set_tests_properties(cmake_target_include_directories_build PROPERTIES  FIXTURES_REQUIRED "cmake_target_include_directories")