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
CMAKE_SOURCE_DIR = /home/shihao_feng/fsh_file/unitree_motor_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shihao_feng/fsh_file/unitree_motor_control/build

# Include any dependencies generated for this target.
include CMakeFiles/calibration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/calibration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/calibration.dir/flags.make

CMakeFiles/calibration.dir/src/tools/calibration.cpp.o: CMakeFiles/calibration.dir/flags.make
CMakeFiles/calibration.dir/src/tools/calibration.cpp.o: ../src/tools/calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shihao_feng/fsh_file/unitree_motor_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/calibration.dir/src/tools/calibration.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibration.dir/src/tools/calibration.cpp.o -c /home/shihao_feng/fsh_file/unitree_motor_control/src/tools/calibration.cpp

CMakeFiles/calibration.dir/src/tools/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibration.dir/src/tools/calibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shihao_feng/fsh_file/unitree_motor_control/src/tools/calibration.cpp > CMakeFiles/calibration.dir/src/tools/calibration.cpp.i

CMakeFiles/calibration.dir/src/tools/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibration.dir/src/tools/calibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shihao_feng/fsh_file/unitree_motor_control/src/tools/calibration.cpp -o CMakeFiles/calibration.dir/src/tools/calibration.cpp.s

CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.requires:

.PHONY : CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.requires

CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.provides: CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.requires
	$(MAKE) -f CMakeFiles/calibration.dir/build.make CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.provides.build
.PHONY : CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.provides

CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.provides.build: CMakeFiles/calibration.dir/src/tools/calibration.cpp.o


CMakeFiles/calibration.dir/src/motor_control.cpp.o: CMakeFiles/calibration.dir/flags.make
CMakeFiles/calibration.dir/src/motor_control.cpp.o: ../src/motor_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shihao_feng/fsh_file/unitree_motor_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/calibration.dir/src/motor_control.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibration.dir/src/motor_control.cpp.o -c /home/shihao_feng/fsh_file/unitree_motor_control/src/motor_control.cpp

CMakeFiles/calibration.dir/src/motor_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibration.dir/src/motor_control.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shihao_feng/fsh_file/unitree_motor_control/src/motor_control.cpp > CMakeFiles/calibration.dir/src/motor_control.cpp.i

CMakeFiles/calibration.dir/src/motor_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibration.dir/src/motor_control.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shihao_feng/fsh_file/unitree_motor_control/src/motor_control.cpp -o CMakeFiles/calibration.dir/src/motor_control.cpp.s

CMakeFiles/calibration.dir/src/motor_control.cpp.o.requires:

.PHONY : CMakeFiles/calibration.dir/src/motor_control.cpp.o.requires

CMakeFiles/calibration.dir/src/motor_control.cpp.o.provides: CMakeFiles/calibration.dir/src/motor_control.cpp.o.requires
	$(MAKE) -f CMakeFiles/calibration.dir/build.make CMakeFiles/calibration.dir/src/motor_control.cpp.o.provides.build
.PHONY : CMakeFiles/calibration.dir/src/motor_control.cpp.o.provides

CMakeFiles/calibration.dir/src/motor_control.cpp.o.provides.build: CMakeFiles/calibration.dir/src/motor_control.cpp.o


CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o: CMakeFiles/calibration.dir/flags.make
CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o: ../src/periodic_rt_task.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shihao_feng/fsh_file/unitree_motor_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o -c /home/shihao_feng/fsh_file/unitree_motor_control/src/periodic_rt_task.cpp

CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shihao_feng/fsh_file/unitree_motor_control/src/periodic_rt_task.cpp > CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.i

CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shihao_feng/fsh_file/unitree_motor_control/src/periodic_rt_task.cpp -o CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.s

CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.requires:

.PHONY : CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.requires

CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.provides: CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.requires
	$(MAKE) -f CMakeFiles/calibration.dir/build.make CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.provides.build
.PHONY : CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.provides

CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.provides.build: CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o


# Object files for target calibration
calibration_OBJECTS = \
"CMakeFiles/calibration.dir/src/tools/calibration.cpp.o" \
"CMakeFiles/calibration.dir/src/motor_control.cpp.o" \
"CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o"

# External object files for target calibration
calibration_EXTERNAL_OBJECTS =

../bin/calibration: CMakeFiles/calibration.dir/src/tools/calibration.cpp.o
../bin/calibration: CMakeFiles/calibration.dir/src/motor_control.cpp.o
../bin/calibration: CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o
../bin/calibration: CMakeFiles/calibration.dir/build.make
../bin/calibration: CMakeFiles/calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shihao_feng/fsh_file/unitree_motor_control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../bin/calibration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/calibration.dir/build: ../bin/calibration

.PHONY : CMakeFiles/calibration.dir/build

CMakeFiles/calibration.dir/requires: CMakeFiles/calibration.dir/src/tools/calibration.cpp.o.requires
CMakeFiles/calibration.dir/requires: CMakeFiles/calibration.dir/src/motor_control.cpp.o.requires
CMakeFiles/calibration.dir/requires: CMakeFiles/calibration.dir/src/periodic_rt_task.cpp.o.requires

.PHONY : CMakeFiles/calibration.dir/requires

CMakeFiles/calibration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/calibration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/calibration.dir/clean

CMakeFiles/calibration.dir/depend:
	cd /home/shihao_feng/fsh_file/unitree_motor_control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shihao_feng/fsh_file/unitree_motor_control /home/shihao_feng/fsh_file/unitree_motor_control /home/shihao_feng/fsh_file/unitree_motor_control/build /home/shihao_feng/fsh_file/unitree_motor_control/build /home/shihao_feng/fsh_file/unitree_motor_control/build/CMakeFiles/calibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/calibration.dir/depend

