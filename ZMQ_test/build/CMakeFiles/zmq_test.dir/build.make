# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zeze/ZMQ_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zeze/ZMQ_test/build

# Include any dependencies generated for this target.
include CMakeFiles/zmq_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/zmq_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/zmq_test.dir/flags.make

CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o: CMakeFiles/zmq_test.dir/flags.make
CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o: ../ZMQ_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zeze/ZMQ_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o -c /home/zeze/ZMQ_test/ZMQ_server.cpp

CMakeFiles/zmq_test.dir/ZMQ_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/zmq_test.dir/ZMQ_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zeze/ZMQ_test/ZMQ_server.cpp > CMakeFiles/zmq_test.dir/ZMQ_server.cpp.i

CMakeFiles/zmq_test.dir/ZMQ_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/zmq_test.dir/ZMQ_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zeze/ZMQ_test/ZMQ_server.cpp -o CMakeFiles/zmq_test.dir/ZMQ_server.cpp.s

CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.requires:

.PHONY : CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.requires

CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.provides: CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.requires
	$(MAKE) -f CMakeFiles/zmq_test.dir/build.make CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.provides.build
.PHONY : CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.provides

CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.provides.build: CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o


# Object files for target zmq_test
zmq_test_OBJECTS = \
"CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o"

# External object files for target zmq_test
zmq_test_EXTERNAL_OBJECTS =

zmq_test: CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o
zmq_test: CMakeFiles/zmq_test.dir/build.make
zmq_test: CMakeFiles/zmq_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zeze/ZMQ_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable zmq_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/zmq_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/zmq_test.dir/build: zmq_test

.PHONY : CMakeFiles/zmq_test.dir/build

CMakeFiles/zmq_test.dir/requires: CMakeFiles/zmq_test.dir/ZMQ_server.cpp.o.requires

.PHONY : CMakeFiles/zmq_test.dir/requires

CMakeFiles/zmq_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/zmq_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/zmq_test.dir/clean

CMakeFiles/zmq_test.dir/depend:
	cd /home/zeze/ZMQ_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zeze/ZMQ_test /home/zeze/ZMQ_test /home/zeze/ZMQ_test/build /home/zeze/ZMQ_test/build /home/zeze/ZMQ_test/build/CMakeFiles/zmq_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/zmq_test.dir/depend
