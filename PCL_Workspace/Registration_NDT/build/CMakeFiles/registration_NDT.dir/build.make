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
CMAKE_SOURCE_DIR = /home/reza/PCL_Workspace/Registration_NDT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reza/PCL_Workspace/Registration_NDT/build

# Include any dependencies generated for this target.
include CMakeFiles/registration_NDT.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/registration_NDT.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/registration_NDT.dir/flags.make

CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o: CMakeFiles/registration_NDT.dir/flags.make
CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o: ../registration_NDT.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reza/PCL_Workspace/Registration_NDT/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o -c /home/reza/PCL_Workspace/Registration_NDT/registration_NDT.cpp

CMakeFiles/registration_NDT.dir/registration_NDT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/registration_NDT.dir/registration_NDT.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reza/PCL_Workspace/Registration_NDT/registration_NDT.cpp > CMakeFiles/registration_NDT.dir/registration_NDT.cpp.i

CMakeFiles/registration_NDT.dir/registration_NDT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/registration_NDT.dir/registration_NDT.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reza/PCL_Workspace/Registration_NDT/registration_NDT.cpp -o CMakeFiles/registration_NDT.dir/registration_NDT.cpp.s

CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.requires:
.PHONY : CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.requires

CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.provides: CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.requires
	$(MAKE) -f CMakeFiles/registration_NDT.dir/build.make CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.provides.build
.PHONY : CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.provides

CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.provides.build: CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o

# Object files for target registration_NDT
registration_NDT_OBJECTS = \
"CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o"

# External object files for target registration_NDT
registration_NDT_EXTERNAL_OBJECTS =

registration_NDT: CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o
registration_NDT: CMakeFiles/registration_NDT.dir/build.make
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_system.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_thread.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libpthread.so
registration_NDT: /usr/local/lib/libpcl_common.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
registration_NDT: /usr/local/lib/libpcl_kdtree.so
registration_NDT: /usr/local/lib/libpcl_octree.so
registration_NDT: /usr/local/lib/libpcl_search.so
registration_NDT: /usr/lib/libOpenNI.so
registration_NDT: /usr/lib/libvtkCommon.so.5.8.0
registration_NDT: /usr/lib/libvtkRendering.so.5.8.0
registration_NDT: /usr/lib/libvtkHybrid.so.5.8.0
registration_NDT: /usr/lib/libvtkCharts.so.5.8.0
registration_NDT: /usr/local/lib/libpcl_io.so
registration_NDT: /usr/local/lib/libpcl_visualization.so
registration_NDT: /usr/local/lib/libpcl_sample_consensus.so
registration_NDT: /usr/local/lib/libpcl_filters.so
registration_NDT: /usr/local/lib/libpcl_features.so
registration_NDT: /usr/local/lib/libpcl_keypoints.so
registration_NDT: /usr/lib/libqhull.so
registration_NDT: /usr/local/lib/libpcl_surface.so
registration_NDT: /usr/local/lib/libpcl_registration.so
registration_NDT: /usr/local/lib/libpcl_segmentation.so
registration_NDT: /usr/local/lib/libpcl_recognition.so
registration_NDT: /usr/local/lib/libpcl_people.so
registration_NDT: /usr/local/lib/libpcl_outofcore.so
registration_NDT: /usr/local/lib/libpcl_tracking.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_system.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_thread.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_mpi.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libpthread.so
registration_NDT: /usr/lib/libqhull.so
registration_NDT: /usr/lib/libOpenNI.so
registration_NDT: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
registration_NDT: /usr/lib/libvtkCommon.so.5.8.0
registration_NDT: /usr/lib/libvtkRendering.so.5.8.0
registration_NDT: /usr/lib/libvtkHybrid.so.5.8.0
registration_NDT: /usr/lib/libvtkCharts.so.5.8.0
registration_NDT: /usr/local/lib/libpcl_common.so
registration_NDT: /usr/local/lib/libpcl_kdtree.so
registration_NDT: /usr/local/lib/libpcl_octree.so
registration_NDT: /usr/local/lib/libpcl_search.so
registration_NDT: /usr/local/lib/libpcl_io.so
registration_NDT: /usr/local/lib/libpcl_visualization.so
registration_NDT: /usr/local/lib/libpcl_sample_consensus.so
registration_NDT: /usr/local/lib/libpcl_filters.so
registration_NDT: /usr/local/lib/libpcl_features.so
registration_NDT: /usr/local/lib/libpcl_keypoints.so
registration_NDT: /usr/local/lib/libpcl_surface.so
registration_NDT: /usr/local/lib/libpcl_registration.so
registration_NDT: /usr/local/lib/libpcl_segmentation.so
registration_NDT: /usr/local/lib/libpcl_recognition.so
registration_NDT: /usr/local/lib/libpcl_people.so
registration_NDT: /usr/local/lib/libpcl_outofcore.so
registration_NDT: /usr/local/lib/libpcl_tracking.so
registration_NDT: /usr/lib/libvtkViews.so.5.8.0
registration_NDT: /usr/lib/libvtkInfovis.so.5.8.0
registration_NDT: /usr/lib/libvtkWidgets.so.5.8.0
registration_NDT: /usr/lib/libvtkHybrid.so.5.8.0
registration_NDT: /usr/lib/libvtkParallel.so.5.8.0
registration_NDT: /usr/lib/libvtkVolumeRendering.so.5.8.0
registration_NDT: /usr/lib/libvtkRendering.so.5.8.0
registration_NDT: /usr/lib/libvtkGraphics.so.5.8.0
registration_NDT: /usr/lib/libvtkImaging.so.5.8.0
registration_NDT: /usr/lib/libvtkIO.so.5.8.0
registration_NDT: /usr/lib/libvtkFiltering.so.5.8.0
registration_NDT: /usr/lib/libvtkCommon.so.5.8.0
registration_NDT: /usr/lib/libvtksys.so.5.8.0
registration_NDT: CMakeFiles/registration_NDT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable registration_NDT"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/registration_NDT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/registration_NDT.dir/build: registration_NDT
.PHONY : CMakeFiles/registration_NDT.dir/build

CMakeFiles/registration_NDT.dir/requires: CMakeFiles/registration_NDT.dir/registration_NDT.cpp.o.requires
.PHONY : CMakeFiles/registration_NDT.dir/requires

CMakeFiles/registration_NDT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/registration_NDT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/registration_NDT.dir/clean

CMakeFiles/registration_NDT.dir/depend:
	cd /home/reza/PCL_Workspace/Registration_NDT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reza/PCL_Workspace/Registration_NDT /home/reza/PCL_Workspace/Registration_NDT /home/reza/PCL_Workspace/Registration_NDT/build /home/reza/PCL_Workspace/Registration_NDT/build /home/reza/PCL_Workspace/Registration_NDT/build/CMakeFiles/registration_NDT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/registration_NDT.dir/depend

