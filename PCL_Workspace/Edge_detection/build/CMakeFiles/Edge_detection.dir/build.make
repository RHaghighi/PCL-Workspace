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
CMAKE_SOURCE_DIR = /home/reza/PCL_Workspace/Edge_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/PCL_Workspace/Edge_detection/build

# Include any dependencies generated for this target.
include CMakeFiles/Edge_detection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Edge_detection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Edge_detection.dir/flags.make

CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o: CMakeFiles/Edge_detection.dir/flags.make
CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o: ../Edge_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/Edge_detection/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o -c /home/reza/PCL_Workspace/Edge_detection/Edge_detection.cpp

CMakeFiles/Edge_detection.dir/Edge_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Edge_detection.dir/Edge_detection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/Edge_detection/Edge_detection.cpp > CMakeFiles/Edge_detection.dir/Edge_detection.cpp.i

CMakeFiles/Edge_detection.dir/Edge_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Edge_detection.dir/Edge_detection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/Edge_detection/Edge_detection.cpp -o CMakeFiles/Edge_detection.dir/Edge_detection.cpp.s

CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.requires:
.PHONY : CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.requires

CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.provides: CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/Edge_detection.dir/build.make CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.provides.build
.PHONY : CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.provides

CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.provides.build: CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o

CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o: CMakeFiles/Edge_detection.dir/flags.make
CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o: ../organized_edge_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/Edge_detection/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o -c /home/reza/PCL_Workspace/Edge_detection/organized_edge_detection.cpp

CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/Edge_detection/organized_edge_detection.cpp > CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.i

CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/Edge_detection/organized_edge_detection.cpp -o CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.s

CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.requires:
.PHONY : CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.requires

CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.provides: CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/Edge_detection.dir/build.make CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.provides.build
.PHONY : CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.provides

CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.provides.build: CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o

# Object files for target Edge_detection
Edge_detection_OBJECTS = \
"CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o" \
"CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o"

# External object files for target Edge_detection
Edge_detection_EXTERNAL_OBJECTS =

Edge_detection: CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o
Edge_detection: CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o
Edge_detection: CMakeFiles/Edge_detection.dir/build.make
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_system.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libpthread.so
Edge_detection: /usr/local/lib/libpcl_common.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
Edge_detection: /usr/local/lib/libpcl_kdtree.so
Edge_detection: /usr/local/lib/libpcl_octree.so
Edge_detection: /usr/local/lib/libpcl_search.so
Edge_detection: /usr/lib/libOpenNI.so
Edge_detection: /usr/lib/libvtkCommon.so.5.8.0
Edge_detection: /usr/lib/libvtkRendering.so.5.8.0
Edge_detection: /usr/lib/libvtkHybrid.so.5.8.0
Edge_detection: /usr/lib/libvtkCharts.so.5.8.0
Edge_detection: /usr/local/lib/libpcl_io.so
Edge_detection: /usr/local/lib/libpcl_visualization.so
Edge_detection: /usr/local/lib/libpcl_sample_consensus.so
Edge_detection: /usr/local/lib/libpcl_filters.so
Edge_detection: /usr/local/lib/libpcl_features.so
Edge_detection: /usr/local/lib/libpcl_keypoints.so
Edge_detection: /usr/lib/libqhull.so
Edge_detection: /usr/local/lib/libpcl_surface.so
Edge_detection: /usr/local/lib/libpcl_registration.so
Edge_detection: /usr/local/lib/libpcl_segmentation.so
Edge_detection: /usr/local/lib/libpcl_recognition.so
Edge_detection: /usr/local/lib/libpcl_people.so
Edge_detection: /usr/local/lib/libpcl_outofcore.so
Edge_detection: /usr/local/lib/libpcl_tracking.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_system.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libpthread.so
Edge_detection: /usr/lib/libqhull.so
Edge_detection: /usr/lib/libOpenNI.so
Edge_detection: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
Edge_detection: /usr/lib/libvtkCommon.so.5.8.0
Edge_detection: /usr/lib/libvtkRendering.so.5.8.0
Edge_detection: /usr/lib/libvtkHybrid.so.5.8.0
Edge_detection: /usr/lib/libvtkCharts.so.5.8.0
Edge_detection: /usr/local/lib/libopencv_viz.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_videostab.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_videoio.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_video.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_superres.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_stitching.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_shape.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_photo.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_objdetect.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_ml.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_imgproc.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_highgui.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_flann.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_features2d.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudev.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudawarping.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudastereo.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudaoptflow.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudaobjdetect.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudalegacy.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudaimgproc.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudafilters.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudafeatures2d.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudacodec.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudabgsegm.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudaarithm.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_core.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_calib3d.so.3.1.0
Edge_detection: /usr/local/lib/libpcl_common.so
Edge_detection: /usr/local/lib/libpcl_kdtree.so
Edge_detection: /usr/local/lib/libpcl_octree.so
Edge_detection: /usr/local/lib/libpcl_search.so
Edge_detection: /usr/local/lib/libpcl_io.so
Edge_detection: /usr/local/lib/libpcl_visualization.so
Edge_detection: /usr/local/lib/libpcl_sample_consensus.so
Edge_detection: /usr/local/lib/libpcl_filters.so
Edge_detection: /usr/local/lib/libpcl_features.so
Edge_detection: /usr/local/lib/libpcl_keypoints.so
Edge_detection: /usr/local/lib/libpcl_surface.so
Edge_detection: /usr/local/lib/libpcl_registration.so
Edge_detection: /usr/local/lib/libpcl_segmentation.so
Edge_detection: /usr/local/lib/libpcl_recognition.so
Edge_detection: /usr/local/lib/libpcl_people.so
Edge_detection: /usr/local/lib/libpcl_outofcore.so
Edge_detection: /usr/local/lib/libpcl_tracking.so
Edge_detection: /usr/lib/libvtkCharts.so.5.8.0
Edge_detection: /usr/lib/libvtkViews.so.5.8.0
Edge_detection: /usr/lib/libvtkInfovis.so.5.8.0
Edge_detection: /usr/lib/libvtkWidgets.so.5.8.0
Edge_detection: /usr/lib/libvtkHybrid.so.5.8.0
Edge_detection: /usr/lib/libvtkParallel.so.5.8.0
Edge_detection: /usr/lib/libvtkVolumeRendering.so.5.8.0
Edge_detection: /usr/lib/libvtkRendering.so.5.8.0
Edge_detection: /usr/lib/libvtkGraphics.so.5.8.0
Edge_detection: /usr/lib/libvtkImaging.so.5.8.0
Edge_detection: /usr/lib/libvtkIO.so.5.8.0
Edge_detection: /usr/lib/libvtkFiltering.so.5.8.0
Edge_detection: /usr/lib/libvtkCommon.so.5.8.0
Edge_detection: /usr/lib/libvtksys.so.5.8.0
Edge_detection: /usr/local/lib/libopencv_cudawarping.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_objdetect.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudafilters.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudaarithm.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_features2d.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_ml.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_highgui.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_videoio.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_flann.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_video.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_imgproc.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_core.so.3.1.0
Edge_detection: /usr/local/lib/libopencv_cudev.so.3.1.0
Edge_detection: CMakeFiles/Edge_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Edge_detection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Edge_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Edge_detection.dir/build: Edge_detection
.PHONY : CMakeFiles/Edge_detection.dir/build

CMakeFiles/Edge_detection.dir/requires: CMakeFiles/Edge_detection.dir/Edge_detection.cpp.o.requires
CMakeFiles/Edge_detection.dir/requires: CMakeFiles/Edge_detection.dir/organized_edge_detection.cpp.o.requires
.PHONY : CMakeFiles/Edge_detection.dir/requires

CMakeFiles/Edge_detection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Edge_detection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Edge_detection.dir/clean

CMakeFiles/Edge_detection.dir/depend:
	cd /home/reza/PCL_Workspace/Edge_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/PCL_Workspace/Edge_detection /home/reza/PCL_Workspace/Edge_detection /home/reza/PCL_Workspace/Edge_detection/build /home/reza/PCL_Workspace/Edge_detection/build /home/reza/PCL_Workspace/Edge_detection/build/CMakeFiles/Edge_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Edge_detection.dir/depend

