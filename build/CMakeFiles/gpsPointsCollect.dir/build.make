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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/fuerte_workspace/sandbox/simple

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/fuerte_workspace/sandbox/simple/build

# Include any dependencies generated for this target.
include CMakeFiles/gpsPointsCollect.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gpsPointsCollect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gpsPointsCollect.dir/flags.make

CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: CMakeFiles/gpsPointsCollect.dir/flags.make
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: ../src/gps_points_collection.cpp
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: ../manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/nav_msgs/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/user/fuerte_workspace/sandbox/simple/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o -c /home/user/fuerte_workspace/sandbox/simple/src/gps_points_collection.cpp

CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/user/fuerte_workspace/sandbox/simple/src/gps_points_collection.cpp > CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.i

CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/user/fuerte_workspace/sandbox/simple/src/gps_points_collection.cpp -o CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.s

CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.requires:
.PHONY : CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.requires

CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.provides: CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.requires
	$(MAKE) -f CMakeFiles/gpsPointsCollect.dir/build.make CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.provides.build
.PHONY : CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.provides

CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.provides.build: CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o

# Object files for target gpsPointsCollect
gpsPointsCollect_OBJECTS = \
"CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o"

# External object files for target gpsPointsCollect
gpsPointsCollect_EXTERNAL_OBJECTS =

../bin/gpsPointsCollect: CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o
../bin/gpsPointsCollect: CMakeFiles/gpsPointsCollect.dir/build.make
../bin/gpsPointsCollect: CMakeFiles/gpsPointsCollect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/gpsPointsCollect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gpsPointsCollect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gpsPointsCollect.dir/build: ../bin/gpsPointsCollect
.PHONY : CMakeFiles/gpsPointsCollect.dir/build

CMakeFiles/gpsPointsCollect.dir/requires: CMakeFiles/gpsPointsCollect.dir/src/gps_points_collection.o.requires
.PHONY : CMakeFiles/gpsPointsCollect.dir/requires

CMakeFiles/gpsPointsCollect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gpsPointsCollect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gpsPointsCollect.dir/clean

CMakeFiles/gpsPointsCollect.dir/depend:
	cd /home/user/fuerte_workspace/sandbox/simple/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/fuerte_workspace/sandbox/simple /home/user/fuerte_workspace/sandbox/simple /home/user/fuerte_workspace/sandbox/simple/build /home/user/fuerte_workspace/sandbox/simple/build /home/user/fuerte_workspace/sandbox/simple/build/CMakeFiles/gpsPointsCollect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gpsPointsCollect.dir/depend

