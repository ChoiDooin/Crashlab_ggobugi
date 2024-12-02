# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/crashlab_rpi/robot_ws/src/crash_motor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crashlab_rpi/robot_ws/src/crash_motor/build

# Include any dependencies generated for this target.
include CMakeFiles/motor_test_utils.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/motor_test_utils.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/motor_test_utils.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor_test_utils.dir/flags.make

CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o: CMakeFiles/motor_test_utils.dir/flags.make
CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o: ../src/motor_test_util.cpp
CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o: CMakeFiles/motor_test_utils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crashlab_rpi/robot_ws/src/crash_motor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o -MF CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o.d -o CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o -c /home/crashlab_rpi/robot_ws/src/crash_motor/src/motor_test_util.cpp

CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.i"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crashlab_rpi/robot_ws/src/crash_motor/src/motor_test_util.cpp > CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.i

CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.s"
	/usr/bin/aarch64-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crashlab_rpi/robot_ws/src/crash_motor/src/motor_test_util.cpp -o CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.s

# Object files for target motor_test_utils
motor_test_utils_OBJECTS = \
"CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o"

# External object files for target motor_test_utils
motor_test_utils_EXTERNAL_OBJECTS =

libmotor_test_utils.a: CMakeFiles/motor_test_utils.dir/src/motor_test_util.cpp.o
libmotor_test_utils.a: CMakeFiles/motor_test_utils.dir/build.make
libmotor_test_utils.a: CMakeFiles/motor_test_utils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/crashlab_rpi/robot_ws/src/crash_motor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmotor_test_utils.a"
	$(CMAKE_COMMAND) -P CMakeFiles/motor_test_utils.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_test_utils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motor_test_utils.dir/build: libmotor_test_utils.a
.PHONY : CMakeFiles/motor_test_utils.dir/build

CMakeFiles/motor_test_utils.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motor_test_utils.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motor_test_utils.dir/clean

CMakeFiles/motor_test_utils.dir/depend:
	cd /home/crashlab_rpi/robot_ws/src/crash_motor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crashlab_rpi/robot_ws/src/crash_motor /home/crashlab_rpi/robot_ws/src/crash_motor /home/crashlab_rpi/robot_ws/src/crash_motor/build /home/crashlab_rpi/robot_ws/src/crash_motor/build /home/crashlab_rpi/robot_ws/src/crash_motor/build/CMakeFiles/motor_test_utils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motor_test_utils.dir/depend

