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
CMAKE_SOURCE_DIR = /home/reza/PCL_Workspace/read_PCD2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/PCL_Workspace/read_PCD2/build

# Include any dependencies generated for this target.
include CMakeFiles/pcd_read.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcd_read.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcd_read.dir/flags.make

CMakeFiles/pcd_read.dir/pcd_read.cpp.o: CMakeFiles/pcd_read.dir/flags.make
CMakeFiles/pcd_read.dir/pcd_read.cpp.o: ../pcd_read.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/read_PCD2/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pcd_read.dir/pcd_read.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcd_read.dir/pcd_read.cpp.o -c /home/reza/PCL_Workspace/read_PCD2/pcd_read.cpp

CMakeFiles/pcd_read.dir/pcd_read.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcd_read.dir/pcd_read.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/read_PCD2/pcd_read.cpp > CMakeFiles/pcd_read.dir/pcd_read.cpp.i

CMakeFiles/pcd_read.dir/pcd_read.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcd_read.dir/pcd_read.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/read_PCD2/pcd_read.cpp -o CMakeFiles/pcd_read.dir/pcd_read.cpp.s

CMakeFiles/pcd_read.dir/pcd_read.cpp.o.requires:
.PHONY : CMakeFiles/pcd_read.dir/pcd_read.cpp.o.requires

CMakeFiles/pcd_read.dir/pcd_read.cpp.o.provides: CMakeFiles/pcd_read.dir/pcd_read.cpp.o.requires
	$(MAKE) -f CMakeFiles/pcd_read.dir/build.make CMakeFiles/pcd_read.dir/pcd_read.cpp.o.provides.build
.PHONY : CMakeFiles/pcd_read.dir/pcd_read.cpp.o.provides

CMakeFiles/pcd_read.dir/pcd_read.cpp.o.provides.build: CMakeFiles/pcd_read.dir/pcd_read.cpp.o

# Object files for target pcd_read
pcd_read_OBJECTS = \
"CMakeFiles/pcd_read.dir/pcd_read.cpp.o"

# External object files for target pcd_read
pcd_read_EXTERNAL_OBJECTS =

pcd_read: CMakeFiles/pcd_read.dir/pcd_read.cpp.o
pcd_read: CMakeFiles/pcd_read.dir/build.make
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_thread.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pcd_read: /usr/lib/x86_64-linux-gnu/libpthread.so
pcd_read: /usr/local/lib/libpcl_common.so
pcd_read: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
pcd_read: /usr/local/lib/libpcl_kdtree.so
pcd_read: /usr/local/lib/libpcl_octree.so
pcd_read: /usr/local/lib/libpcl_search.so
pcd_read: /usr/lib/libOpenNI.so
pcd_read: /usr/lib/libvtkCommon.so.5.8.0
pcd_read: /usr/lib/libvtkRendering.so.5.8.0
pcd_read: /usr/lib/libvtkHybrid.so.5.8.0
pcd_read: /usr/lib/libvtkCharts.so.5.8.0
pcd_read: /usr/local/lib/libpcl_io.so
pcd_read: /usr/local/lib/libpcl_visualization.so
pcd_read: /usr/local/lib/libpcl_sample_consensus.so
pcd_read: /usr/local/lib/libpcl_filters.so
pcd_read: /usr/local/lib/libpcl_features.so
pcd_read: /usr/local/lib/libpcl_keypoints.so
pcd_read: /usr/lib/libqhull.so
pcd_read: /usr/local/lib/libpcl_surface.so
pcd_read: /usr/local/lib/libpcl_registration.so
pcd_read: /usr/local/lib/libpcl_segmentation.so
pcd_read: /usr/local/lib/libpcl_recognition.so
pcd_read: /usr/local/lib/libpcl_people.so
pcd_read: /usr/local/lib/libpcl_outofcore.so
pcd_read: /usr/local/lib/libpcl_tracking.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_system.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_thread.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
pcd_read: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
pcd_read: /usr/lib/x86_64-linux-gnu/libpthread.so
pcd_read: /usr/lib/libqhull.so
pcd_read: /usr/lib/libOpenNI.so
pcd_read: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
pcd_read: /usr/lib/libvtkCommon.so.5.8.0
pcd_read: /usr/lib/libvtkRendering.so.5.8.0
pcd_read: /usr/lib/libvtkHybrid.so.5.8.0
pcd_read: /usr/lib/libvtkCharts.so.5.8.0
pcd_read: /usr/local/lib/libpcl_common.so
pcd_read: /usr/local/lib/libpcl_kdtree.so
pcd_read: /usr/local/lib/libpcl_octree.so
pcd_read: /usr/local/lib/libpcl_search.so
pcd_read: /usr/local/lib/libpcl_io.so
pcd_read: /usr/local/lib/libpcl_visualization.so
pcd_read: /usr/local/lib/libpcl_sample_consensus.so
pcd_read: /usr/local/lib/libpcl_filters.so
pcd_read: /usr/local/lib/libpcl_features.so
pcd_read: /usr/local/lib/libpcl_keypoints.so
pcd_read: /usr/local/lib/libpcl_surface.so
pcd_read: /usr/local/lib/libpcl_registration.so
pcd_read: /usr/local/lib/libpcl_segmentation.so
pcd_read: /usr/local/lib/libpcl_recognition.so
pcd_read: /usr/local/lib/libpcl_people.so
pcd_read: /usr/local/lib/libpcl_outofcore.so
pcd_read: /usr/local/lib/libpcl_tracking.so
pcd_read: /usr/lib/libvtkViews.so.5.8.0
pcd_read: /usr/lib/libvtkInfovis.so.5.8.0
pcd_read: /usr/lib/libvtkWidgets.so.5.8.0
pcd_read: /usr/lib/libvtkHybrid.so.5.8.0
pcd_read: /usr/lib/libvtkParallel.so.5.8.0
pcd_read: /usr/lib/libvtkVolumeRendering.so.5.8.0
pcd_read: /usr/lib/libvtkRendering.so.5.8.0
pcd_read: /usr/lib/libvtkGraphics.so.5.8.0
pcd_read: /usr/lib/libvtkImaging.so.5.8.0
pcd_read: /usr/lib/libvtkIO.so.5.8.0
pcd_read: /usr/lib/libvtkFiltering.so.5.8.0
pcd_read: /usr/lib/libvtkCommon.so.5.8.0
pcd_read: /usr/lib/libvtksys.so.5.8.0
pcd_read: CMakeFiles/pcd_read.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pcd_read"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcd_read.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcd_read.dir/build: pcd_read
.PHONY : CMakeFiles/pcd_read.dir/build

CMakeFiles/pcd_read.dir/requires: CMakeFiles/pcd_read.dir/pcd_read.cpp.o.requires
.PHONY : CMakeFiles/pcd_read.dir/requires

CMakeFiles/pcd_read.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcd_read.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcd_read.dir/clean

CMakeFiles/pcd_read.dir/depend:
	cd /home/reza/PCL_Workspace/read_PCD2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/PCL_Workspace/read_PCD2 /home/reza/PCL_Workspace/read_PCD2 /home/reza/PCL_Workspace/read_PCD2/build /home/reza/PCL_Workspace/read_PCD2/build /home/reza/PCL_Workspace/read_PCD2/build/CMakeFiles/pcd_read.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcd_read.dir/depend

