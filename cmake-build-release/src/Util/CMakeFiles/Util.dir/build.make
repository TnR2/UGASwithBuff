# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /snap/clion/237/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/237/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alliance/Desktop/UGAS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alliance/Desktop/UGAS/cmake-build-release

# Include any dependencies generated for this target.
include src/Util/CMakeFiles/Util.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/Util/CMakeFiles/Util.dir/compiler_depend.make

# Include the progress variables for this target.
include src/Util/CMakeFiles/Util.dir/progress.make

# Include the compile flags for this target's objects.
include src/Util/CMakeFiles/Util.dir/flags.make

src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o: src/Util/CMakeFiles/Util.dir/flags.make
src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o: /home/alliance/Desktop/UGAS/src/Util/Debug/DebugCanvas.cpp
src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o: src/Util/CMakeFiles/Util.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o -MF CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o.d -o CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o -c /home/alliance/Desktop/UGAS/src/Util/Debug/DebugCanvas.cpp

src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/Util/Debug/DebugCanvas.cpp > CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.i

src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/Util/Debug/DebugCanvas.cpp -o CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.s

src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.o: src/Util/CMakeFiles/Util.dir/flags.make
src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.o: /home/alliance/Desktop/UGAS/src/Util/Debug/Log.cpp
src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.o: src/Util/CMakeFiles/Util.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.o -MF CMakeFiles/Util.dir/Debug/Log.cpp.o.d -o CMakeFiles/Util.dir/Debug/Log.cpp.o -c /home/alliance/Desktop/UGAS/src/Util/Debug/Log.cpp

src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Util.dir/Debug/Log.cpp.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/Util/Debug/Log.cpp > CMakeFiles/Util.dir/Debug/Log.cpp.i

src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Util.dir/Debug/Log.cpp.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/Util/Debug/Log.cpp -o CMakeFiles/Util.dir/Debug/Log.cpp.s

src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o: src/Util/CMakeFiles/Util.dir/flags.make
src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o: /home/alliance/Desktop/UGAS/src/Util/Debug/MatForm/MatForm.cpp
src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o: src/Util/CMakeFiles/Util.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o -MF CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o.d -o CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o -c /home/alliance/Desktop/UGAS/src/Util/Debug/MatForm/MatForm.cpp

src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/Util/Debug/MatForm/MatForm.cpp > CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.i

src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/Util/Debug/MatForm/MatForm.cpp -o CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.s

src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.o: src/Util/CMakeFiles/Util.dir/flags.make
src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.o: /home/alliance/Desktop/UGAS/src/Util/Parameter/Parameters.cpp
src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.o: src/Util/CMakeFiles/Util.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.o -MF CMakeFiles/Util.dir/Parameter/Parameters.cpp.o.d -o CMakeFiles/Util.dir/Parameter/Parameters.cpp.o -c /home/alliance/Desktop/UGAS/src/Util/Parameter/Parameters.cpp

src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Util.dir/Parameter/Parameters.cpp.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/Util/Parameter/Parameters.cpp > CMakeFiles/Util.dir/Parameter/Parameters.cpp.i

src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Util.dir/Parameter/Parameters.cpp.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/Util/Parameter/Parameters.cpp -o CMakeFiles/Util.dir/Parameter/Parameters.cpp.s

src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o: src/Util/CMakeFiles/Util.dir/flags.make
src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o: /home/alliance/Desktop/UGAS/src/Util/TimeStamp/TimeStampCounter.cpp
src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o: src/Util/CMakeFiles/Util.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o -MF CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o.d -o CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o -c /home/alliance/Desktop/UGAS/src/Util/TimeStamp/TimeStampCounter.cpp

src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/Util/TimeStamp/TimeStampCounter.cpp > CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.i

src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/Util/TimeStamp/TimeStampCounter.cpp -o CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.s

# Object files for target Util
Util_OBJECTS = \
"CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o" \
"CMakeFiles/Util.dir/Debug/Log.cpp.o" \
"CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o" \
"CMakeFiles/Util.dir/Parameter/Parameters.cpp.o" \
"CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o"

# External object files for target Util
Util_EXTERNAL_OBJECTS =

/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/Debug/DebugCanvas.cpp.o
/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/Debug/Log.cpp.o
/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/Debug/MatForm/MatForm.cpp.o
/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/Parameter/Parameters.cpp.o
/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/TimeStamp/TimeStampCounter.cpp.o
/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/build.make
/home/alliance/Desktop/UGAS/lib/libUtil.a: src/Util/CMakeFiles/Util.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library /home/alliance/Desktop/UGAS/lib/libUtil.a"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && $(CMAKE_COMMAND) -P CMakeFiles/Util.dir/cmake_clean_target.cmake
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Util.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/Util/CMakeFiles/Util.dir/build: /home/alliance/Desktop/UGAS/lib/libUtil.a
.PHONY : src/Util/CMakeFiles/Util.dir/build

src/Util/CMakeFiles/Util.dir/clean:
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/Util && $(CMAKE_COMMAND) -P CMakeFiles/Util.dir/cmake_clean.cmake
.PHONY : src/Util/CMakeFiles/Util.dir/clean

src/Util/CMakeFiles/Util.dir/depend:
	cd /home/alliance/Desktop/UGAS/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alliance/Desktop/UGAS /home/alliance/Desktop/UGAS/src/Util /home/alliance/Desktop/UGAS/cmake-build-release /home/alliance/Desktop/UGAS/cmake-build-release/src/Util /home/alliance/Desktop/UGAS/cmake-build-release/src/Util/CMakeFiles/Util.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Util/CMakeFiles/Util.dir/depend

