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
CMAKE_SOURCE_DIR = /home/reza/PCL_Workspace/Online_Segmentation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/PCL_Workspace/Online_Segmentation/build

# Include any dependencies generated for this target.
include CMakeFiles/Online_Segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Online_Segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Online_Segmentation.dir/flags.make

CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o: CMakeFiles/Online_Segmentation.dir/flags.make
CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o: ../Online_Segmentation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/Online_Segmentation/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o -c /home/reza/PCL_Workspace/Online_Segmentation/Online_Segmentation.cpp

CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/Online_Segmentation/Online_Segmentation.cpp > CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.i

CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/Online_Segmentation/Online_Segmentation.cpp -o CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.s

CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.requires:
.PHONY : CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.requires

CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.provides: CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/Online_Segmentation.dir/build.make CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.provides

CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.provides.build: CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o

# Object files for target Online_Segmentation
Online_Segmentation_OBJECTS = \
"CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o"

# External object files for target Online_Segmentation
Online_Segmentation_EXTERNAL_OBJECTS =

Online_Segmentation: CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o
Online_Segmentation: CMakeFiles/Online_Segmentation.dir/build.make
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
Online_Segmentation: /usr/local/lib/libpcl_common.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
Online_Segmentation: /usr/local/lib/libpcl_kdtree.so
Online_Segmentation: /usr/local/lib/libpcl_octree.so
Online_Segmentation: /usr/local/lib/libpcl_search.so
Online_Segmentation: /usr/lib/libOpenNI.so
Online_Segmentation: /usr/lib/libvtkCommon.so.5.8.0
Online_Segmentation: /usr/lib/libvtkRendering.so.5.8.0
Online_Segmentation: /usr/lib/libvtkHybrid.so.5.8.0
Online_Segmentation: /usr/lib/libvtkCharts.so.5.8.0
Online_Segmentation: /usr/local/lib/libpcl_io.so
Online_Segmentation: /usr/local/lib/libpcl_visualization.so
Online_Segmentation: /usr/local/lib/libpcl_sample_consensus.so
Online_Segmentation: /usr/local/lib/libpcl_filters.so
Online_Segmentation: /usr/local/lib/libpcl_features.so
Online_Segmentation: /usr/local/lib/libpcl_keypoints.so
Online_Segmentation: /usr/lib/libqhull.so
Online_Segmentation: /usr/local/lib/libpcl_surface.so
Online_Segmentation: /usr/local/lib/libpcl_registration.so
Online_Segmentation: /usr/local/lib/libpcl_segmentation.so
Online_Segmentation: /usr/local/lib/libpcl_recognition.so
Online_Segmentation: /usr/local/lib/libpcl_people.so
Online_Segmentation: /usr/local/lib/libpcl_outofcore.so
Online_Segmentation: /usr/local/lib/libpcl_tracking.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
Online_Segmentation: /usr/lib/libqhull.so
Online_Segmentation: /usr/lib/libOpenNI.so
Online_Segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
Online_Segmentation: /usr/lib/libvtkCommon.so.5.8.0
Online_Segmentation: /usr/lib/libvtkRendering.so.5.8.0
Online_Segmentation: /usr/lib/libvtkHybrid.so.5.8.0
Online_Segmentation: /usr/lib/libvtkCharts.so.5.8.0
Online_Segmentation: /usr/local/lib/libpcl_common.so
Online_Segmentation: /usr/local/lib/libpcl_kdtree.so
Online_Segmentation: /usr/local/lib/libpcl_octree.so
Online_Segmentation: /usr/local/lib/libpcl_search.so
Online_Segmentation: /usr/local/lib/libpcl_io.so
Online_Segmentation: /usr/local/lib/libpcl_visualization.so
Online_Segmentation: /usr/local/lib/libpcl_sample_consensus.so
Online_Segmentation: /usr/local/lib/libpcl_filters.so
Online_Segmentation: /usr/local/lib/libpcl_features.so
Online_Segmentation: /usr/local/lib/libpcl_keypoints.so
Online_Segmentation: /usr/local/lib/libpcl_surface.so
Online_Segmentation: /usr/local/lib/libpcl_registration.so
Online_Segmentation: /usr/local/lib/libpcl_segmentation.so
Online_Segmentation: /usr/local/lib/libpcl_recognition.so
Online_Segmentation: /usr/local/lib/libpcl_people.so
Online_Segmentation: /usr/local/lib/libpcl_outofcore.so
Online_Segmentation: /usr/local/lib/libpcl_tracking.so
Online_Segmentation: /usr/lib/libvtkViews.so.5.8.0
Online_Segmentation: /usr/lib/libvtkInfovis.so.5.8.0
Online_Segmentation: /usr/lib/libvtkWidgets.so.5.8.0
Online_Segmentation: /usr/lib/libvtkHybrid.so.5.8.0
Online_Segmentation: /usr/lib/libvtkParallel.so.5.8.0
Online_Segmentation: /usr/lib/libvtkVolumeRendering.so.5.8.0
Online_Segmentation: /usr/lib/libvtkRendering.so.5.8.0
Online_Segmentation: /usr/lib/libvtkGraphics.so.5.8.0
Online_Segmentation: /usr/lib/libvtkImaging.so.5.8.0
Online_Segmentation: /usr/lib/libvtkIO.so.5.8.0
Online_Segmentation: /usr/lib/libvtkFiltering.so.5.8.0
Online_Segmentation: /usr/lib/libvtkCommon.so.5.8.0
Online_Segmentation: /usr/lib/libvtksys.so.5.8.0
Online_Segmentation: CMakeFiles/Online_Segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Online_Segmentation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Online_Segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Online_Segmentation.dir/build: Online_Segmentation
.PHONY : CMakeFiles/Online_Segmentation.dir/build

CMakeFiles/Online_Segmentation.dir/requires: CMakeFiles/Online_Segmentation.dir/Online_Segmentation.cpp.o.requires
.PHONY : CMakeFiles/Online_Segmentation.dir/requires

CMakeFiles/Online_Segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Online_Segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Online_Segmentation.dir/clean

CMakeFiles/Online_Segmentation.dir/depend:
	cd /home/reza/PCL_Workspace/Online_Segmentation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/PCL_Workspace/Online_Segmentation /home/reza/PCL_Workspace/Online_Segmentation /home/reza/PCL_Workspace/Online_Segmentation/build /home/reza/PCL_Workspace/Online_Segmentation/build /home/reza/PCL_Workspace/Online_Segmentation/build/CMakeFiles/Online_Segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Online_Segmentation.dir/depend

