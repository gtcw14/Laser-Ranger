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
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/grant/ros_workspace/laserRanger

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/grant/ros_workspace/laserRanger/build

# Include any dependencies generated for this target.
include CMakeFiles/distance.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/distance.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/distance.dir/flags.make

CMakeFiles/distance.dir/src/distance.o: CMakeFiles/distance.dir/flags.make
CMakeFiles/distance.dir/src/distance.o: ../src/distance.cpp
CMakeFiles/distance.dir/src/distance.o: ../manifest.xml
CMakeFiles/distance.dir/src/distance.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/distance.dir/src/distance.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/distance.dir/src/distance.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/distance.dir/src/distance.o: /opt/ros/fuerte/share/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/grant/ros_workspace/laserRanger/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/distance.dir/src/distance.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/distance.dir/src/distance.o -c /home/grant/ros_workspace/laserRanger/src/distance.cpp

CMakeFiles/distance.dir/src/distance.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/distance.dir/src/distance.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/grant/ros_workspace/laserRanger/src/distance.cpp > CMakeFiles/distance.dir/src/distance.i

CMakeFiles/distance.dir/src/distance.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/distance.dir/src/distance.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/grant/ros_workspace/laserRanger/src/distance.cpp -o CMakeFiles/distance.dir/src/distance.s

CMakeFiles/distance.dir/src/distance.o.requires:
.PHONY : CMakeFiles/distance.dir/src/distance.o.requires

CMakeFiles/distance.dir/src/distance.o.provides: CMakeFiles/distance.dir/src/distance.o.requires
	$(MAKE) -f CMakeFiles/distance.dir/build.make CMakeFiles/distance.dir/src/distance.o.provides.build
.PHONY : CMakeFiles/distance.dir/src/distance.o.provides

CMakeFiles/distance.dir/src/distance.o.provides.build: CMakeFiles/distance.dir/src/distance.o

# Object files for target distance
distance_OBJECTS = \
"CMakeFiles/distance.dir/src/distance.o"

# External object files for target distance
distance_EXTERNAL_OBJECTS =

../bin/distance: CMakeFiles/distance.dir/src/distance.o
../bin/distance: /opt/ros/fuerte/lib/libopencv_calib3d.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_contrib.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_core.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_features2d.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_flann.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_gpu.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_highgui.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_imgproc.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_legacy.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_ml.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_nonfree.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_objdetect.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_photo.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_stitching.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_ts.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_video.so
../bin/distance: /opt/ros/fuerte/lib/libopencv_videostab.so
../bin/distance: CMakeFiles/distance.dir/build.make
../bin/distance: CMakeFiles/distance.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/distance"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/distance.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/distance.dir/build: ../bin/distance
.PHONY : CMakeFiles/distance.dir/build

CMakeFiles/distance.dir/requires: CMakeFiles/distance.dir/src/distance.o.requires
.PHONY : CMakeFiles/distance.dir/requires

CMakeFiles/distance.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/distance.dir/cmake_clean.cmake
.PHONY : CMakeFiles/distance.dir/clean

CMakeFiles/distance.dir/depend:
	cd /home/grant/ros_workspace/laserRanger/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/grant/ros_workspace/laserRanger /home/grant/ros_workspace/laserRanger /home/grant/ros_workspace/laserRanger/build /home/grant/ros_workspace/laserRanger/build /home/grant/ros_workspace/laserRanger/build/CMakeFiles/distance.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/distance.dir/depend

