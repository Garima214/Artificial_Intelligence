# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build"

# Include any dependencies generated for this target.
include CMakeFiles/assignmet04.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/assignmet04.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/assignmet04.dir/flags.make

CMakeFiles/assignmet04.dir/src/main.cpp.o: CMakeFiles/assignmet04.dir/flags.make
CMakeFiles/assignmet04.dir/src/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/assignmet04.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/assignmet04.dir/src/main.cpp.o -c "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/main.cpp"

CMakeFiles/assignmet04.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignmet04.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/main.cpp" > CMakeFiles/assignmet04.dir/src/main.cpp.i

CMakeFiles/assignmet04.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignmet04.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/main.cpp" -o CMakeFiles/assignmet04.dir/src/main.cpp.s

CMakeFiles/assignmet04.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/assignmet04.dir/src/main.cpp.o.requires

CMakeFiles/assignmet04.dir/src/main.cpp.o.provides: CMakeFiles/assignmet04.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/assignmet04.dir/build.make CMakeFiles/assignmet04.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/assignmet04.dir/src/main.cpp.o.provides

CMakeFiles/assignmet04.dir/src/main.cpp.o.provides.build: CMakeFiles/assignmet04.dir/src/main.cpp.o

CMakeFiles/assignmet04.dir/src/agent.cpp.o: CMakeFiles/assignmet04.dir/flags.make
CMakeFiles/assignmet04.dir/src/agent.cpp.o: ../src/agent.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build/CMakeFiles" $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/assignmet04.dir/src/agent.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/assignmet04.dir/src/agent.cpp.o -c "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/agent.cpp"

CMakeFiles/assignmet04.dir/src/agent.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignmet04.dir/src/agent.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/agent.cpp" > CMakeFiles/assignmet04.dir/src/agent.cpp.i

CMakeFiles/assignmet04.dir/src/agent.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignmet04.dir/src/agent.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/agent.cpp" -o CMakeFiles/assignmet04.dir/src/agent.cpp.s

CMakeFiles/assignmet04.dir/src/agent.cpp.o.requires:
.PHONY : CMakeFiles/assignmet04.dir/src/agent.cpp.o.requires

CMakeFiles/assignmet04.dir/src/agent.cpp.o.provides: CMakeFiles/assignmet04.dir/src/agent.cpp.o.requires
	$(MAKE) -f CMakeFiles/assignmet04.dir/build.make CMakeFiles/assignmet04.dir/src/agent.cpp.o.provides.build
.PHONY : CMakeFiles/assignmet04.dir/src/agent.cpp.o.provides

CMakeFiles/assignmet04.dir/src/agent.cpp.o.provides.build: CMakeFiles/assignmet04.dir/src/agent.cpp.o

CMakeFiles/assignmet04.dir/src/environment.cpp.o: CMakeFiles/assignmet04.dir/flags.make
CMakeFiles/assignmet04.dir/src/environment.cpp.o: ../src/environment.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build/CMakeFiles" $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/assignmet04.dir/src/environment.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/assignmet04.dir/src/environment.cpp.o -c "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/environment.cpp"

CMakeFiles/assignmet04.dir/src/environment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignmet04.dir/src/environment.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/environment.cpp" > CMakeFiles/assignmet04.dir/src/environment.cpp.i

CMakeFiles/assignmet04.dir/src/environment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignmet04.dir/src/environment.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/src/environment.cpp" -o CMakeFiles/assignmet04.dir/src/environment.cpp.s

CMakeFiles/assignmet04.dir/src/environment.cpp.o.requires:
.PHONY : CMakeFiles/assignmet04.dir/src/environment.cpp.o.requires

CMakeFiles/assignmet04.dir/src/environment.cpp.o.provides: CMakeFiles/assignmet04.dir/src/environment.cpp.o.requires
	$(MAKE) -f CMakeFiles/assignmet04.dir/build.make CMakeFiles/assignmet04.dir/src/environment.cpp.o.provides.build
.PHONY : CMakeFiles/assignmet04.dir/src/environment.cpp.o.provides

CMakeFiles/assignmet04.dir/src/environment.cpp.o.provides.build: CMakeFiles/assignmet04.dir/src/environment.cpp.o

# Object files for target assignmet04
assignmet04_OBJECTS = \
"CMakeFiles/assignmet04.dir/src/main.cpp.o" \
"CMakeFiles/assignmet04.dir/src/agent.cpp.o" \
"CMakeFiles/assignmet04.dir/src/environment.cpp.o"

# External object files for target assignmet04
assignmet04_EXTERNAL_OBJECTS =

../bin/assignmet04: CMakeFiles/assignmet04.dir/src/main.cpp.o
../bin/assignmet04: CMakeFiles/assignmet04.dir/src/agent.cpp.o
../bin/assignmet04: CMakeFiles/assignmet04.dir/src/environment.cpp.o
../bin/assignmet04: CMakeFiles/assignmet04.dir/build.make
../bin/assignmet04: CMakeFiles/assignmet04.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/assignmet04"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/assignmet04.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/assignmet04.dir/build: ../bin/assignmet04
.PHONY : CMakeFiles/assignmet04.dir/build

CMakeFiles/assignmet04.dir/requires: CMakeFiles/assignmet04.dir/src/main.cpp.o.requires
CMakeFiles/assignmet04.dir/requires: CMakeFiles/assignmet04.dir/src/agent.cpp.o.requires
CMakeFiles/assignmet04.dir/requires: CMakeFiles/assignmet04.dir/src/environment.cpp.o.requires
.PHONY : CMakeFiles/assignmet04.dir/requires

CMakeFiles/assignmet04.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/assignmet04.dir/cmake_clean.cmake
.PHONY : CMakeFiles/assignmet04.dir/clean

CMakeFiles/assignmet04.dir/depend:
	cd "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04" "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04" "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build" "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build" "/home/garima/Documents/5. AI/26-04-2016/air_assignment_04/air_assignment_04/build/CMakeFiles/assignmet04.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/assignmet04.dir/depend

