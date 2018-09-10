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
CMAKE_COMMAND = /opt/clion-2018.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /opt/clion-2018.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/ps02.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ps02.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ps02.dir/flags.make

CMakeFiles/ps02.dir/optical_flow.cpp.o: CMakeFiles/ps02.dir/flags.make
CMakeFiles/ps02.dir/optical_flow.cpp.o: ../optical_flow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ps02.dir/optical_flow.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ps02.dir/optical_flow.cpp.o -c /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/optical_flow.cpp

CMakeFiles/ps02.dir/optical_flow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ps02.dir/optical_flow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/optical_flow.cpp > CMakeFiles/ps02.dir/optical_flow.cpp.i

CMakeFiles/ps02.dir/optical_flow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ps02.dir/optical_flow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/optical_flow.cpp -o CMakeFiles/ps02.dir/optical_flow.cpp.s

CMakeFiles/ps02.dir/optical_flow.cpp.o.requires:

.PHONY : CMakeFiles/ps02.dir/optical_flow.cpp.o.requires

CMakeFiles/ps02.dir/optical_flow.cpp.o.provides: CMakeFiles/ps02.dir/optical_flow.cpp.o.requires
	$(MAKE) -f CMakeFiles/ps02.dir/build.make CMakeFiles/ps02.dir/optical_flow.cpp.o.provides.build
.PHONY : CMakeFiles/ps02.dir/optical_flow.cpp.o.provides

CMakeFiles/ps02.dir/optical_flow.cpp.o.provides.build: CMakeFiles/ps02.dir/optical_flow.cpp.o


# Object files for target ps02
ps02_OBJECTS = \
"CMakeFiles/ps02.dir/optical_flow.cpp.o"

# External object files for target ps02
ps02_EXTERNAL_OBJECTS =

ps02: CMakeFiles/ps02.dir/optical_flow.cpp.o
ps02: CMakeFiles/ps02.dir/build.make
ps02: /usr/local/lib/libopencv_viz.so.3.1.0
ps02: /usr/local/lib/libopencv_videostab.so.3.1.0
ps02: /usr/local/lib/libopencv_superres.so.3.1.0
ps02: /usr/local/lib/libopencv_stitching.so.3.1.0
ps02: /usr/local/lib/libopencv_shape.so.3.1.0
ps02: /usr/local/lib/libopencv_photo.so.3.1.0
ps02: /usr/local/lib/libopencv_objdetect.so.3.1.0
ps02: /usr/local/lib/libopencv_calib3d.so.3.1.0
ps02: /usr/local/lib/libopencv_features2d.so.3.1.0
ps02: /usr/local/lib/libopencv_ml.so.3.1.0
ps02: /usr/local/lib/libopencv_highgui.so.3.1.0
ps02: /usr/local/lib/libopencv_videoio.so.3.1.0
ps02: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
ps02: /usr/local/lib/libopencv_flann.so.3.1.0
ps02: /usr/local/lib/libopencv_video.so.3.1.0
ps02: /usr/local/lib/libopencv_imgproc.so.3.1.0
ps02: /usr/local/lib/libopencv_core.so.3.1.0
ps02: CMakeFiles/ps02.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ps02"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ps02.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ps02.dir/build: ps02

.PHONY : CMakeFiles/ps02.dir/build

CMakeFiles/ps02.dir/requires: CMakeFiles/ps02.dir/optical_flow.cpp.o.requires

.PHONY : CMakeFiles/ps02.dir/requires

CMakeFiles/ps02.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ps02.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ps02.dir/clean

CMakeFiles/ps02.dir/depend:
	cd /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02 /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02 /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug /home/binx/Documents/Self-Learning/vSLAM-HW/hw6/ps02/cmake-build-debug/CMakeFiles/ps02.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ps02.dir/depend

