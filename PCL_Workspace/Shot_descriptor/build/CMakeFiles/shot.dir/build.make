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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/reza/PCL_Workspace/Shot_descriptor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/PCL_Workspace/Shot_descriptor/build

# Include any dependencies generated for this target.
include CMakeFiles/shot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shot.dir/flags.make

CMakeFiles/shot.dir/shot.cpp.o: CMakeFiles/shot.dir/flags.make
CMakeFiles/shot.dir/shot.cpp.o: ../shot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/Shot_descriptor/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/shot.dir/shot.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/shot.dir/shot.cpp.o -c /home/reza/PCL_Workspace/Shot_descriptor/shot.cpp

CMakeFiles/shot.dir/shot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shot.dir/shot.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/Shot_descriptor/shot.cpp > CMakeFiles/shot.dir/shot.cpp.i

CMakeFiles/shot.dir/shot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shot.dir/shot.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/Shot_descriptor/shot.cpp -o CMakeFiles/shot.dir/shot.cpp.s

CMakeFiles/shot.dir/shot.cpp.o.requires:
.PHONY : CMakeFiles/shot.dir/shot.cpp.o.requires

CMakeFiles/shot.dir/shot.cpp.o.provides: CMakeFiles/shot.dir/shot.cpp.o.requires
	$(MAKE) -f CMakeFiles/shot.dir/build.make CMakeFiles/shot.dir/shot.cpp.o.provides.build
.PHONY : CMakeFiles/shot.dir/shot.cpp.o.provides

CMakeFiles/shot.dir/shot.cpp.o.provides.build: CMakeFiles/shot.dir/shot.cpp.o

# Object files for target shot
shot_OBJECTS = \
"CMakeFiles/shot.dir/shot.cpp.o"

# External object files for target shot
shot_EXTERNAL_OBJECTS =

shot: CMakeFiles/shot.dir/shot.cpp.o
shot: CMakeFiles/shot.dir/build.make
shot: /usr/lib/x86_64-linux-gnu/libboost_system.so
shot: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
shot: /usr/lib/x86_64-linux-gnu/libboost_thread.so
shot: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
shot: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
shot: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
shot: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
shot: /usr/lib/x86_64-linux-gnu/libpthread.so
shot: /usr/local/lib/libpcl_common.so
shot: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
shot: /usr/local/lib/libpcl_kdtree.so
shot: /usr/local/lib/libpcl_octree.so
shot: /usr/local/lib/libpcl_search.so
shot: /usr/lib/libOpenNI.so
shot: /usr/lib/libvtkCommon.so.5.8.0
shot: /usr/lib/libvtkRendering.so.5.8.0
shot: /usr/lib/libvtkHybrid.so.5.8.0
shot: /usr/lib/libvtkCharts.so.5.8.0
shot: /usr/local/lib/libpcl_io.so
shot: /usr/local/lib/libpcl_visualization.so
shot: /usr/local/lib/libpcl_sample_consensus.so
shot: /usr/local/lib/libpcl_filters.so
shot: /usr/local/lib/libpcl_features.so
shot: /usr/local/lib/libpcl_keypoints.so
shot: /usr/lib/libqhull.so
shot: /usr/local/lib/libpcl_surface.so
shot: /usr/local/lib/libpcl_registration.so
shot: /usr/local/lib/libpcl_segmentation.so
shot: /usr/local/lib/libpcl_recognition.so
shot: /usr/local/lib/libpcl_people.so
shot: /usr/local/lib/libpcl_outofcore.so
shot: /usr/local/lib/libpcl_tracking.so
shot: /usr/lib/x86_64-linux-gnu/libboost_system.so
shot: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
shot: /usr/lib/x86_64-linux-gnu/libboost_thread.so
shot: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
shot: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
shot: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
shot: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
shot: /usr/lib/x86_64-linux-gnu/libpthread.so
shot: /usr/lib/libqhull.so
shot: /usr/lib/libOpenNI.so
shot: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
shot: /usr/lib/libvtkCommon.so.5.8.0
shot: /usr/lib/libvtkRendering.so.5.8.0
shot: /usr/lib/libvtkHybrid.so.5.8.0
shot: /usr/lib/libvtkCharts.so.5.8.0
shot: /usr/local/lib/libpcl_common.so
shot: /usr/local/lib/libpcl_kdtree.so
shot: /usr/local/lib/libpcl_octree.so
shot: /usr/local/lib/libpcl_search.so
shot: /usr/local/lib/libpcl_io.so
shot: /usr/local/lib/libpcl_visualization.so
shot: /usr/local/lib/libpcl_sample_consensus.so
shot: /usr/local/lib/libpcl_filters.so
shot: /usr/local/lib/libpcl_features.so
shot: /usr/local/lib/libpcl_keypoints.so
shot: /usr/local/lib/libpcl_surface.so
shot: /usr/local/lib/libpcl_registration.so
shot: /usr/local/lib/libpcl_segmentation.so
shot: /usr/local/lib/libpcl_recognition.so
shot: /usr/local/lib/libpcl_people.so
shot: /usr/local/lib/libpcl_outofcore.so
shot: /usr/local/lib/libpcl_tracking.so
shot: /usr/lib/libvtkViews.so.5.8.0
shot: /usr/lib/libvtkInfovis.so.5.8.0
shot: /usr/lib/libvtkWidgets.so.5.8.0
shot: /usr/lib/libvtkHybrid.so.5.8.0
shot: /usr/lib/libvtkParallel.so.5.8.0
shot: /usr/lib/libvtkVolumeRendering.so.5.8.0
shot: /usr/lib/libvtkRendering.so.5.8.0
shot: /usr/lib/libvtkGraphics.so.5.8.0
shot: /usr/lib/libvtkImaging.so.5.8.0
shot: /usr/lib/libvtkIO.so.5.8.0
shot: /usr/lib/libvtkFiltering.so.5.8.0
shot: /usr/lib/libvtkCommon.so.5.8.0
shot: /usr/lib/libvtksys.so.5.8.0
shot: CMakeFiles/shot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable shot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shot.dir/build: shot
.PHONY : CMakeFiles/shot.dir/build

CMakeFiles/shot.dir/requires: CMakeFiles/shot.dir/shot.cpp.o.requires
.PHONY : CMakeFiles/shot.dir/requires

CMakeFiles/shot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shot.dir/clean

CMakeFiles/shot.dir/depend:
	cd /home/reza/PCL_Workspace/Shot_descriptor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/PCL_Workspace/Shot_descriptor /home/reza/PCL_Workspace/Shot_descriptor /home/reza/PCL_Workspace/Shot_descriptor/build /home/reza/PCL_Workspace/Shot_descriptor/build /home/reza/PCL_Workspace/Shot_descriptor/build/CMakeFiles/shot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shot.dir/depend

