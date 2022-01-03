# CMake generated Testfile for 
# Source directory: /home/zeze/catkin_ws/src/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test
# Build directory: /home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test-version "/home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/test-version" "--no-skip")
set_tests_properties(test-version PROPERTIES  FIXTURES_REQUIRED "TEST_DATA" LABELS "all" WORKING_DIRECTORY "/home/zeze/catkin_ws/src")
add_test(test-http-message "/home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/test-http-message" "--no-skip")
set_tests_properties(test-http-message PROPERTIES  FIXTURES_REQUIRED "TEST_DATA" LABELS "all" WORKING_DIRECTORY "/home/zeze/catkin_ws/src")
add_test(test-streamer "/home/zeze/catkin_ws/src/build/armor_detection/3rdparty/nadjieb_mjpeg_streamer/test/test-streamer" "--no-skip")
set_tests_properties(test-streamer PROPERTIES  FIXTURES_REQUIRED "TEST_DATA" LABELS "all" WORKING_DIRECTORY "/home/zeze/catkin_ws/src")
subdirs("cmake_import")
subdirs("cmake_import_minver")
subdirs("cmake_add_subdirectory")
subdirs("cmake_fetch_content")
subdirs("cmake_target_include_directories")
