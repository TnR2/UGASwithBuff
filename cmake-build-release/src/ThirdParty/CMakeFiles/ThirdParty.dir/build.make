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
include src/ThirdParty/CMakeFiles/ThirdParty.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.make

# Include the progress variables for this target.
include src/ThirdParty/CMakeFiles/ThirdParty.dir/progress.make

# Include the compile flags for this target's objects.
include src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make

src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/EasyLogging++/EasyLogging++.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o -MF CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o.d -o CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/EasyLogging++/EasyLogging++.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/EasyLogging++/EasyLogging++.cc > CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/EasyLogging++/EasyLogging++.cc -o CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.s

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_linux.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o -MF CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o.d -o CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_linux.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_linux.cc > CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_linux.cc -o CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.s

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_osx.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o -MF CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o.d -o CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_osx.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_osx.cc > CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_osx.cc -o CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.s

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_win.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o -MF CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o.d -o CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_win.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_win.cc > CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/list_ports/list_ports_win.cc -o CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.s

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/unix.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o -MF CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o.d -o CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/unix.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/unix.cc > CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/unix.cc -o CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.s

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/win.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o -MF CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o.d -o CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/win.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/win.cc > CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/impl/win.cc -o CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.s

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/flags.make
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.o: /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/serial.cc
src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.o: src/ThirdParty/CMakeFiles/ThirdParty.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.o"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.o -MF CMakeFiles/ThirdParty.dir/Serial/serial.cc.o.d -o CMakeFiles/ThirdParty.dir/Serial/serial.cc.o -c /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/serial.cc

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThirdParty.dir/Serial/serial.cc.i"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/serial.cc > CMakeFiles/ThirdParty.dir/Serial/serial.cc.i

src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThirdParty.dir/Serial/serial.cc.s"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alliance/Desktop/UGAS/src/ThirdParty/Serial/serial.cc -o CMakeFiles/ThirdParty.dir/Serial/serial.cc.s

# Object files for target ThirdParty
ThirdParty_OBJECTS = \
"CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o" \
"CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o" \
"CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o" \
"CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o" \
"CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o" \
"CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o" \
"CMakeFiles/ThirdParty.dir/Serial/serial.cc.o"

# External object files for target ThirdParty
ThirdParty_EXTERNAL_OBJECTS =

/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/EasyLogging++/EasyLogging++.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_linux.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_osx.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/list_ports/list_ports_win.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/unix.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/impl/win.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/Serial/serial.cc.o
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/build.make
/home/alliance/Desktop/UGAS/lib/libThirdParty.a: src/ThirdParty/CMakeFiles/ThirdParty.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alliance/Desktop/UGAS/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library /home/alliance/Desktop/UGAS/lib/libThirdParty.a"
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && $(CMAKE_COMMAND) -P CMakeFiles/ThirdParty.dir/cmake_clean_target.cmake
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ThirdParty.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/ThirdParty/CMakeFiles/ThirdParty.dir/build: /home/alliance/Desktop/UGAS/lib/libThirdParty.a
.PHONY : src/ThirdParty/CMakeFiles/ThirdParty.dir/build

src/ThirdParty/CMakeFiles/ThirdParty.dir/clean:
	cd /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty && $(CMAKE_COMMAND) -P CMakeFiles/ThirdParty.dir/cmake_clean.cmake
.PHONY : src/ThirdParty/CMakeFiles/ThirdParty.dir/clean

src/ThirdParty/CMakeFiles/ThirdParty.dir/depend:
	cd /home/alliance/Desktop/UGAS/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alliance/Desktop/UGAS /home/alliance/Desktop/UGAS/src/ThirdParty /home/alliance/Desktop/UGAS/cmake-build-release /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty /home/alliance/Desktop/UGAS/cmake-build-release/src/ThirdParty/CMakeFiles/ThirdParty.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/ThirdParty/CMakeFiles/ThirdParty.dir/depend
