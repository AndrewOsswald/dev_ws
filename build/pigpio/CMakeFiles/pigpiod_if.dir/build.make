# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ubuntu/dev_ws/pigpio-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/dev_ws/build/pigpio

# Include any dependencies generated for this target.
include CMakeFiles/pigpiod_if.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pigpiod_if.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pigpiod_if.dir/flags.make

CMakeFiles/pigpiod_if.dir/pigpiod_if.c.o: CMakeFiles/pigpiod_if.dir/flags.make
CMakeFiles/pigpiod_if.dir/pigpiod_if.c.o: /home/ubuntu/dev_ws/pigpio-master/pigpiod_if.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/dev_ws/build/pigpio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/pigpiod_if.dir/pigpiod_if.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigpiod_if.dir/pigpiod_if.c.o   -c /home/ubuntu/dev_ws/pigpio-master/pigpiod_if.c

CMakeFiles/pigpiod_if.dir/pigpiod_if.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigpiod_if.dir/pigpiod_if.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ubuntu/dev_ws/pigpio-master/pigpiod_if.c > CMakeFiles/pigpiod_if.dir/pigpiod_if.c.i

CMakeFiles/pigpiod_if.dir/pigpiod_if.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigpiod_if.dir/pigpiod_if.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ubuntu/dev_ws/pigpio-master/pigpiod_if.c -o CMakeFiles/pigpiod_if.dir/pigpiod_if.c.s

CMakeFiles/pigpiod_if.dir/command.c.o: CMakeFiles/pigpiod_if.dir/flags.make
CMakeFiles/pigpiod_if.dir/command.c.o: /home/ubuntu/dev_ws/pigpio-master/command.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/dev_ws/build/pigpio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/pigpiod_if.dir/command.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pigpiod_if.dir/command.c.o   -c /home/ubuntu/dev_ws/pigpio-master/command.c

CMakeFiles/pigpiod_if.dir/command.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pigpiod_if.dir/command.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ubuntu/dev_ws/pigpio-master/command.c > CMakeFiles/pigpiod_if.dir/command.c.i

CMakeFiles/pigpiod_if.dir/command.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pigpiod_if.dir/command.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ubuntu/dev_ws/pigpio-master/command.c -o CMakeFiles/pigpiod_if.dir/command.c.s

# Object files for target pigpiod_if
pigpiod_if_OBJECTS = \
"CMakeFiles/pigpiod_if.dir/pigpiod_if.c.o" \
"CMakeFiles/pigpiod_if.dir/command.c.o"

# External object files for target pigpiod_if
pigpiod_if_EXTERNAL_OBJECTS =

libpigpiod_if.so: CMakeFiles/pigpiod_if.dir/pigpiod_if.c.o
libpigpiod_if.so: CMakeFiles/pigpiod_if.dir/command.c.o
libpigpiod_if.so: CMakeFiles/pigpiod_if.dir/build.make
libpigpiod_if.so: CMakeFiles/pigpiod_if.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/dev_ws/build/pigpio/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C shared library libpigpiod_if.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pigpiod_if.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pigpiod_if.dir/build: libpigpiod_if.so

.PHONY : CMakeFiles/pigpiod_if.dir/build

CMakeFiles/pigpiod_if.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pigpiod_if.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pigpiod_if.dir/clean

CMakeFiles/pigpiod_if.dir/depend:
	cd /home/ubuntu/dev_ws/build/pigpio && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/dev_ws/pigpio-master /home/ubuntu/dev_ws/pigpio-master /home/ubuntu/dev_ws/build/pigpio /home/ubuntu/dev_ws/build/pigpio /home/ubuntu/dev_ws/build/pigpio/CMakeFiles/pigpiod_if.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pigpiod_if.dir/depend

