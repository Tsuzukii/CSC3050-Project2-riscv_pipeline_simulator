# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.29.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.29.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/felixyan/Downloads/122090897

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/felixyan/Downloads/122090897/build

# Include any dependencies generated for this target.
include CMakeFiles/Simulator.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Simulator.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Simulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Simulator.dir/flags.make

CMakeFiles/Simulator.dir/src/main.cpp.o: CMakeFiles/Simulator.dir/flags.make
CMakeFiles/Simulator.dir/src/main.cpp.o: /Users/felixyan/Downloads/122090897/src/main.cpp
CMakeFiles/Simulator.dir/src/main.cpp.o: CMakeFiles/Simulator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/felixyan/Downloads/122090897/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Simulator.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Simulator.dir/src/main.cpp.o -MF CMakeFiles/Simulator.dir/src/main.cpp.o.d -o CMakeFiles/Simulator.dir/src/main.cpp.o -c /Users/felixyan/Downloads/122090897/src/main.cpp

CMakeFiles/Simulator.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Simulator.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/felixyan/Downloads/122090897/src/main.cpp > CMakeFiles/Simulator.dir/src/main.cpp.i

CMakeFiles/Simulator.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Simulator.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/felixyan/Downloads/122090897/src/main.cpp -o CMakeFiles/Simulator.dir/src/main.cpp.s

CMakeFiles/Simulator.dir/src/simulator.cpp.o: CMakeFiles/Simulator.dir/flags.make
CMakeFiles/Simulator.dir/src/simulator.cpp.o: /Users/felixyan/Downloads/122090897/src/simulator.cpp
CMakeFiles/Simulator.dir/src/simulator.cpp.o: CMakeFiles/Simulator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/felixyan/Downloads/122090897/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Simulator.dir/src/simulator.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Simulator.dir/src/simulator.cpp.o -MF CMakeFiles/Simulator.dir/src/simulator.cpp.o.d -o CMakeFiles/Simulator.dir/src/simulator.cpp.o -c /Users/felixyan/Downloads/122090897/src/simulator.cpp

CMakeFiles/Simulator.dir/src/simulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Simulator.dir/src/simulator.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/felixyan/Downloads/122090897/src/simulator.cpp > CMakeFiles/Simulator.dir/src/simulator.cpp.i

CMakeFiles/Simulator.dir/src/simulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Simulator.dir/src/simulator.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/felixyan/Downloads/122090897/src/simulator.cpp -o CMakeFiles/Simulator.dir/src/simulator.cpp.s

CMakeFiles/Simulator.dir/src/memory_manager.cpp.o: CMakeFiles/Simulator.dir/flags.make
CMakeFiles/Simulator.dir/src/memory_manager.cpp.o: /Users/felixyan/Downloads/122090897/src/memory_manager.cpp
CMakeFiles/Simulator.dir/src/memory_manager.cpp.o: CMakeFiles/Simulator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/felixyan/Downloads/122090897/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Simulator.dir/src/memory_manager.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Simulator.dir/src/memory_manager.cpp.o -MF CMakeFiles/Simulator.dir/src/memory_manager.cpp.o.d -o CMakeFiles/Simulator.dir/src/memory_manager.cpp.o -c /Users/felixyan/Downloads/122090897/src/memory_manager.cpp

CMakeFiles/Simulator.dir/src/memory_manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Simulator.dir/src/memory_manager.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/felixyan/Downloads/122090897/src/memory_manager.cpp > CMakeFiles/Simulator.dir/src/memory_manager.cpp.i

CMakeFiles/Simulator.dir/src/memory_manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Simulator.dir/src/memory_manager.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/felixyan/Downloads/122090897/src/memory_manager.cpp -o CMakeFiles/Simulator.dir/src/memory_manager.cpp.s

# Object files for target Simulator
Simulator_OBJECTS = \
"CMakeFiles/Simulator.dir/src/main.cpp.o" \
"CMakeFiles/Simulator.dir/src/simulator.cpp.o" \
"CMakeFiles/Simulator.dir/src/memory_manager.cpp.o"

# External object files for target Simulator
Simulator_EXTERNAL_OBJECTS =

Simulator: CMakeFiles/Simulator.dir/src/main.cpp.o
Simulator: CMakeFiles/Simulator.dir/src/simulator.cpp.o
Simulator: CMakeFiles/Simulator.dir/src/memory_manager.cpp.o
Simulator: CMakeFiles/Simulator.dir/build.make
Simulator: CMakeFiles/Simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/felixyan/Downloads/122090897/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Simulator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Simulator.dir/build: Simulator
.PHONY : CMakeFiles/Simulator.dir/build

CMakeFiles/Simulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Simulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Simulator.dir/clean

CMakeFiles/Simulator.dir/depend:
	cd /Users/felixyan/Downloads/122090897/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/felixyan/Downloads/122090897 /Users/felixyan/Downloads/122090897 /Users/felixyan/Downloads/122090897/build /Users/felixyan/Downloads/122090897/build /Users/felixyan/Downloads/122090897/build/CMakeFiles/Simulator.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/Simulator.dir/depend

