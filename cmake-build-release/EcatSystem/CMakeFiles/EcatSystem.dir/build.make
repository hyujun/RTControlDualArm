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
CMAKE_SOURCE_DIR = /home/baek

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/baek/cmake-build-release

# Include any dependencies generated for this target.
include EcatSystem/CMakeFiles/EcatSystem.dir/depend.make

# Include the progress variables for this target.
include EcatSystem/CMakeFiles/EcatSystem.dir/progress.make

# Include the compile flags for this target's objects.
include EcatSystem/CMakeFiles/EcatSystem.dir/flags.make

EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o: EcatSystem/CMakeFiles/EcatSystem.dir/flags.make
EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o: ../EcatSystem/Ecat_Master.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/baek/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o"
	cd /home/baek/cmake-build-release/EcatSystem && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o -c /home/baek/EcatSystem/Ecat_Master.cpp

EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.i"
	cd /home/baek/cmake-build-release/EcatSystem && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/baek/EcatSystem/Ecat_Master.cpp > CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.i

EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.s"
	cd /home/baek/cmake-build-release/EcatSystem && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/baek/EcatSystem/Ecat_Master.cpp -o CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.s

EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.requires:

.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.requires

EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.provides: EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.requires
	$(MAKE) -f EcatSystem/CMakeFiles/EcatSystem.dir/build.make EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.provides.build
.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.provides

EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.provides.build: EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o


EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o: EcatSystem/CMakeFiles/EcatSystem.dir/flags.make
EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o: ../EcatSystem/PDOConfig.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/baek/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o"
	cd /home/baek/cmake-build-release/EcatSystem && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/EcatSystem.dir/PDOConfig.c.o   -c /home/baek/EcatSystem/PDOConfig.c

EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/EcatSystem.dir/PDOConfig.c.i"
	cd /home/baek/cmake-build-release/EcatSystem && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/baek/EcatSystem/PDOConfig.c > CMakeFiles/EcatSystem.dir/PDOConfig.c.i

EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/EcatSystem.dir/PDOConfig.c.s"
	cd /home/baek/cmake-build-release/EcatSystem && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/baek/EcatSystem/PDOConfig.c -o CMakeFiles/EcatSystem.dir/PDOConfig.c.s

EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.requires:

.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.requires

EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.provides: EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.requires
	$(MAKE) -f EcatSystem/CMakeFiles/EcatSystem.dir/build.make EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.provides.build
.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.provides

EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.provides.build: EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o


# Object files for target EcatSystem
EcatSystem_OBJECTS = \
"CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o" \
"CMakeFiles/EcatSystem.dir/PDOConfig.c.o"

# External object files for target EcatSystem
EcatSystem_EXTERNAL_OBJECTS =

EcatSystem/libEcatSystem.a: EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o
EcatSystem/libEcatSystem.a: EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o
EcatSystem/libEcatSystem.a: EcatSystem/CMakeFiles/EcatSystem.dir/build.make
EcatSystem/libEcatSystem.a: EcatSystem/CMakeFiles/EcatSystem.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/baek/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libEcatSystem.a"
	cd /home/baek/cmake-build-release/EcatSystem && $(CMAKE_COMMAND) -P CMakeFiles/EcatSystem.dir/cmake_clean_target.cmake
	cd /home/baek/cmake-build-release/EcatSystem && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/EcatSystem.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
EcatSystem/CMakeFiles/EcatSystem.dir/build: EcatSystem/libEcatSystem.a

.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/build

EcatSystem/CMakeFiles/EcatSystem.dir/requires: EcatSystem/CMakeFiles/EcatSystem.dir/Ecat_Master.cpp.o.requires
EcatSystem/CMakeFiles/EcatSystem.dir/requires: EcatSystem/CMakeFiles/EcatSystem.dir/PDOConfig.c.o.requires

.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/requires

EcatSystem/CMakeFiles/EcatSystem.dir/clean:
	cd /home/baek/cmake-build-release/EcatSystem && $(CMAKE_COMMAND) -P CMakeFiles/EcatSystem.dir/cmake_clean.cmake
.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/clean

EcatSystem/CMakeFiles/EcatSystem.dir/depend:
	cd /home/baek/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/baek /home/baek/EcatSystem /home/baek/cmake-build-release /home/baek/cmake-build-release/EcatSystem /home/baek/cmake-build-release/EcatSystem/CMakeFiles/EcatSystem.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : EcatSystem/CMakeFiles/EcatSystem.dir/depend

