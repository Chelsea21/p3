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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chelsea/Documents/GraphicsProjects/p3/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chelsea/Documents/GraphicsProjects/p3/build

# Include any dependencies generated for this target.
include p3/CMakeFiles/p3.dir/depend.make

# Include the progress variables for this target.
include p3/CMakeFiles/p3.dir/progress.make

# Include the compile flags for this target's objects.
include p3/CMakeFiles/p3.dir/flags.make

p3/CMakeFiles/p3.dir/main.cpp.o: p3/CMakeFiles/p3.dir/flags.make
p3/CMakeFiles/p3.dir/main.cpp.o: /home/chelsea/Documents/GraphicsProjects/p3/src/p3/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p3/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p3/CMakeFiles/p3.dir/main.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p3.dir/main.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p3/src/p3/main.cpp

p3/CMakeFiles/p3.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p3.dir/main.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p3/src/p3/main.cpp > CMakeFiles/p3.dir/main.cpp.i

p3/CMakeFiles/p3.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p3.dir/main.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p3/src/p3/main.cpp -o CMakeFiles/p3.dir/main.cpp.s

p3/CMakeFiles/p3.dir/main.cpp.o.requires:
.PHONY : p3/CMakeFiles/p3.dir/main.cpp.o.requires

p3/CMakeFiles/p3.dir/main.cpp.o.provides: p3/CMakeFiles/p3.dir/main.cpp.o.requires
	$(MAKE) -f p3/CMakeFiles/p3.dir/build.make p3/CMakeFiles/p3.dir/main.cpp.o.provides.build
.PHONY : p3/CMakeFiles/p3.dir/main.cpp.o.provides

p3/CMakeFiles/p3.dir/main.cpp.o.provides.build: p3/CMakeFiles/p3.dir/main.cpp.o

p3/CMakeFiles/p3.dir/raytracer.cpp.o: p3/CMakeFiles/p3.dir/flags.make
p3/CMakeFiles/p3.dir/raytracer.cpp.o: /home/chelsea/Documents/GraphicsProjects/p3/src/p3/raytracer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p3/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p3/CMakeFiles/p3.dir/raytracer.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p3.dir/raytracer.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p3/src/p3/raytracer.cpp

p3/CMakeFiles/p3.dir/raytracer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p3.dir/raytracer.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p3/src/p3/raytracer.cpp > CMakeFiles/p3.dir/raytracer.cpp.i

p3/CMakeFiles/p3.dir/raytracer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p3.dir/raytracer.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p3/src/p3/raytracer.cpp -o CMakeFiles/p3.dir/raytracer.cpp.s

p3/CMakeFiles/p3.dir/raytracer.cpp.o.requires:
.PHONY : p3/CMakeFiles/p3.dir/raytracer.cpp.o.requires

p3/CMakeFiles/p3.dir/raytracer.cpp.o.provides: p3/CMakeFiles/p3.dir/raytracer.cpp.o.requires
	$(MAKE) -f p3/CMakeFiles/p3.dir/build.make p3/CMakeFiles/p3.dir/raytracer.cpp.o.provides.build
.PHONY : p3/CMakeFiles/p3.dir/raytracer.cpp.o.provides

p3/CMakeFiles/p3.dir/raytracer.cpp.o.provides.build: p3/CMakeFiles/p3.dir/raytracer.cpp.o

# Object files for target p3
p3_OBJECTS = \
"CMakeFiles/p3.dir/main.cpp.o" \
"CMakeFiles/p3.dir/raytracer.cpp.o"

# External object files for target p3
p3_EXTERNAL_OBJECTS =

p3/p3: p3/CMakeFiles/p3.dir/main.cpp.o
p3/p3: p3/CMakeFiles/p3.dir/raytracer.cpp.o
p3/p3: application/libapplication.a
p3/p3: math/libmath.a
p3/p3: scene/libscene.a
p3/p3: tinyxml/libtinyxml.a
p3/p3: /usr/lib/i386-linux-gnu/libSDLmain.a
p3/p3: /usr/lib/i386-linux-gnu/libSDL.so
p3/p3: /usr/lib/i386-linux-gnu/libpng.so
p3/p3: /usr/lib/i386-linux-gnu/libz.so
p3/p3: /usr/lib/i386-linux-gnu/libGLU.so
p3/p3: /usr/lib/i386-linux-gnu/libGL.so
p3/p3: /usr/lib/i386-linux-gnu/libSM.so
p3/p3: /usr/lib/i386-linux-gnu/libICE.so
p3/p3: /usr/lib/i386-linux-gnu/libX11.so
p3/p3: /usr/lib/i386-linux-gnu/libXext.so
p3/p3: /usr/lib/i386-linux-gnu/libglut.so
p3/p3: /usr/lib/i386-linux-gnu/libXmu.so
p3/p3: /usr/lib/i386-linux-gnu/libXi.so
p3/p3: /usr/lib/i386-linux-gnu/libGLEW.so
p3/p3: p3/CMakeFiles/p3.dir/build.make
p3/p3: p3/CMakeFiles/p3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable p3"
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/p3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
p3/CMakeFiles/p3.dir/build: p3/p3
.PHONY : p3/CMakeFiles/p3.dir/build

p3/CMakeFiles/p3.dir/requires: p3/CMakeFiles/p3.dir/main.cpp.o.requires
p3/CMakeFiles/p3.dir/requires: p3/CMakeFiles/p3.dir/raytracer.cpp.o.requires
.PHONY : p3/CMakeFiles/p3.dir/requires

p3/CMakeFiles/p3.dir/clean:
	cd /home/chelsea/Documents/GraphicsProjects/p3/build/p3 && $(CMAKE_COMMAND) -P CMakeFiles/p3.dir/cmake_clean.cmake
.PHONY : p3/CMakeFiles/p3.dir/clean

p3/CMakeFiles/p3.dir/depend:
	cd /home/chelsea/Documents/GraphicsProjects/p3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chelsea/Documents/GraphicsProjects/p3/src /home/chelsea/Documents/GraphicsProjects/p3/src/p3 /home/chelsea/Documents/GraphicsProjects/p3/build /home/chelsea/Documents/GraphicsProjects/p3/build/p3 /home/chelsea/Documents/GraphicsProjects/p3/build/p3/CMakeFiles/p3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : p3/CMakeFiles/p3.dir/depend

