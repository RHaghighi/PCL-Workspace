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
CMAKE_SOURCE_DIR = /home/reza/PCL_Workspace/Segmentation_DNBS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/PCL_Workspace/Segmentation_DNBS/build

# Include any dependencies generated for this target.
include CMakeFiles/segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segmentation.dir/flags.make

CMakeFiles/segmentation.dir/segmentation.cpp.o: CMakeFiles/segmentation.dir/flags.make
CMakeFiles/segmentation.dir/segmentation.cpp.o: ../segmentation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/Segmentation_DNBS/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/segmentation.dir/segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/segmentation.dir/segmentation.cpp.o -c /home/reza/PCL_Workspace/Segmentation_DNBS/segmentation.cpp

CMakeFiles/segmentation.dir/segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmentation.dir/segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/Segmentation_DNBS/segmentation.cpp > CMakeFiles/segmentation.dir/segmentation.cpp.i

CMakeFiles/segmentation.dir/segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmentation.dir/segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/Segmentation_DNBS/segmentation.cpp -o CMakeFiles/segmentation.dir/segmentation.cpp.s

CMakeFiles/segmentation.dir/segmentation.cpp.o.requires:
.PHONY : CMakeFiles/segmentation.dir/segmentation.cpp.o.requires

CMakeFiles/segmentation.dir/segmentation.cpp.o.provides: CMakeFiles/segmentation.dir/segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/segmentation.dir/build.make CMakeFiles/segmentation.dir/segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/segmentation.dir/segmentation.cpp.o.provides

CMakeFiles/segmentation.dir/segmentation.cpp.o.provides.build: CMakeFiles/segmentation.dir/segmentation.cpp.o

# Object files for target segmentation
segmentation_OBJECTS = \
"CMakeFiles/segmentation.dir/segmentation.cpp.o"

# External object files for target segmentation
segmentation_EXTERNAL_OBJECTS =

segmentation: CMakeFiles/segmentation.dir/segmentation.cpp.o
segmentation: CMakeFiles/segmentation.dir/build.make
segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
segmentation: /usr/local/lib/libpcl_common.so
segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
segmentation: /usr/local/lib/libpcl_kdtree.so
segmentation: /usr/local/lib/libpcl_octree.so
segmentation: /usr/local/lib/libpcl_search.so
segmentation: /usr/lib/libOpenNI.so
segmentation: /usr/lib/libvtkCommon.so.5.8.0
segmentation: /usr/lib/libvtkRendering.so.5.8.0
segmentation: /usr/lib/libvtkHybrid.so.5.8.0
segmentation: /usr/lib/libvtkCharts.so.5.8.0
segmentation: /usr/local/lib/libpcl_io.so
segmentation: /usr/local/lib/libpcl_visualization.so
segmentation: /usr/local/lib/libpcl_sample_consensus.so
segmentation: /usr/local/lib/libpcl_filters.so
segmentation: /usr/local/lib/libpcl_features.so
segmentation: /usr/local/lib/libpcl_keypoints.so
segmentation: /usr/lib/libqhull.so
segmentation: /usr/local/lib/libpcl_surface.so
segmentation: /usr/local/lib/libpcl_registration.so
segmentation: /usr/local/lib/libpcl_segmentation.so
segmentation: /usr/local/lib/libpcl_recognition.so
segmentation: /usr/local/lib/libpcl_people.so
segmentation: /usr/local/lib/libpcl_outofcore.so
segmentation: /usr/local/lib/libpcl_tracking.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
segmentation: /usr/lib/libqhull.so
segmentation: /usr/lib/libOpenNI.so
segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
segmentation: /usr/lib/libvtkCommon.so.5.8.0
segmentation: /usr/lib/libvtkRendering.so.5.8.0
segmentation: /usr/lib/libvtkHybrid.so.5.8.0
segmentation: /usr/lib/libvtkCharts.so.5.8.0
segmentation: /usr/local/lib/libpcl_common.so
segmentation: /usr/local/lib/libpcl_kdtree.so
segmentation: /usr/local/lib/libpcl_octree.so
segmentation: /usr/local/lib/libpcl_search.so
segmentation: /usr/local/lib/libpcl_io.so
segmentation: /usr/local/lib/libpcl_visualization.so
segmentation: /usr/local/lib/libpcl_sample_consensus.so
segmentation: /usr/local/lib/libpcl_filters.so
segmentation: /usr/local/lib/libpcl_features.so
segmentation: /usr/local/lib/libpcl_keypoints.so
segmentation: /usr/local/lib/libpcl_surface.so
segmentation: /usr/local/lib/libpcl_registration.so
segmentation: /usr/local/lib/libpcl_segmentation.so
segmentation: /usr/local/lib/libpcl_recognition.so
segmentation: /usr/local/lib/libpcl_people.so
segmentation: /usr/local/lib/libpcl_outofcore.so
segmentation: /usr/local/lib/libpcl_tracking.so
segmentation: /usr/lib/libvtkViews.so.5.8.0
segmentation: /usr/lib/libvtkInfovis.so.5.8.0
segmentation: /usr/lib/libvtkWidgets.so.5.8.0
segmentation: /usr/lib/libvtkHybrid.so.5.8.0
segmentation: /usr/lib/libvtkParallel.so.5.8.0
segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
segmentation: /usr/lib/libvtkRendering.so.5.8.0
segmentation: /usr/lib/libvtkGraphics.so.5.8.0
segmentation: /usr/lib/libvtkImaging.so.5.8.0
segmentation: /usr/lib/libvtkIO.so.5.8.0
segmentation: /usr/lib/libvtkFiltering.so.5.8.0
segmentation: /usr/lib/libvtkCommon.so.5.8.0
segmentation: /usr/lib/libvtksys.so.5.8.0
segmentation: CMakeFiles/segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable segmentation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segmentation.dir/build: segmentation
.PHONY : CMakeFiles/segmentation.dir/build

CMakeFiles/segmentation.dir/requires: CMakeFiles/segmentation.dir/segmentation.cpp.o.requires
.PHONY : CMakeFiles/segmentation.dir/requires

CMakeFiles/segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segmentation.dir/clean

CMakeFiles/segmentation.dir/depend:
	cd /home/reza/PCL_Workspace/Segmentation_DNBS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/PCL_Workspace/Segmentation_DNBS /home/reza/PCL_Workspace/Segmentation_DNBS /home/reza/PCL_Workspace/Segmentation_DNBS/build /home/reza/PCL_Workspace/Segmentation_DNBS/build /home/reza/PCL_Workspace/Segmentation_DNBS/build/CMakeFiles/segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segmentation.dir/depend

