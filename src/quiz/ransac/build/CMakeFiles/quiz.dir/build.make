# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nivesh/Desktop/pcltest/src/quiz/ransac

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nivesh/Desktop/pcltest/src/quiz/ransac/build

# Include any dependencies generated for this target.
include CMakeFiles/quiz.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quiz.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quiz.dir/flags.make

CMakeFiles/quiz.dir/ransac2d.cpp.o: CMakeFiles/quiz.dir/flags.make
CMakeFiles/quiz.dir/ransac2d.cpp.o: ../ransac2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nivesh/Desktop/pcltest/src/quiz/ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quiz.dir/ransac2d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quiz.dir/ransac2d.cpp.o -c /home/nivesh/Desktop/pcltest/src/quiz/ransac/ransac2d.cpp

CMakeFiles/quiz.dir/ransac2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quiz.dir/ransac2d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nivesh/Desktop/pcltest/src/quiz/ransac/ransac2d.cpp > CMakeFiles/quiz.dir/ransac2d.cpp.i

CMakeFiles/quiz.dir/ransac2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quiz.dir/ransac2d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nivesh/Desktop/pcltest/src/quiz/ransac/ransac2d.cpp -o CMakeFiles/quiz.dir/ransac2d.cpp.s

CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.o: CMakeFiles/quiz.dir/flags.make
CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.o: /home/nivesh/Desktop/pcltest/src/render/render.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nivesh/Desktop/pcltest/src/quiz/ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.o -c /home/nivesh/Desktop/pcltest/src/render/render.cpp

CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nivesh/Desktop/pcltest/src/render/render.cpp > CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.i

CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nivesh/Desktop/pcltest/src/render/render.cpp -o CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.s

# Object files for target quiz
quiz_OBJECTS = \
"CMakeFiles/quiz.dir/ransac2d.cpp.o" \
"CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.o"

# External object files for target quiz
quiz_EXTERNAL_OBJECTS =

quiz: CMakeFiles/quiz.dir/ransac2d.cpp.o
quiz: CMakeFiles/quiz.dir/home/nivesh/Desktop/pcltest/src/render/render.cpp.o
quiz: CMakeFiles/quiz.dir/build.make
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_surface.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_keypoints.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_tracking.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_recognition.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_stereo.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_outofcore.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_people.so
quiz: /opt/boost_1_77_0/boost_install_dir/lib/libboost_system.so
quiz: /opt/boost_1_77_0/boost_install_dir/lib/libboost_filesystem.so
quiz: /opt/boost_1_77_0/boost_install_dir/lib/libboost_date_time.so
quiz: /opt/boost_1_77_0/boost_install_dir/lib/libboost_iostreams.so
quiz: /opt/boost_1_77_0/boost_install_dir/lib/libboost_regex.so
quiz: /usr/lib/x86_64-linux-gnu/libqhull_r.so
quiz: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libfreetype.so
quiz: /usr/lib/x86_64-linux-gnu/libz.so
quiz: /usr/lib/x86_64-linux-gnu/libjpeg.so
quiz: /usr/lib/x86_64-linux-gnu/libpng.so
quiz: /usr/lib/x86_64-linux-gnu/libtiff.so
quiz: /usr/lib/x86_64-linux-gnu/libexpat.so
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_registration.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_segmentation.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_features.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_filters.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_sample_consensus.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_ml.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_visualization.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_search.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_kdtree.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_io.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_octree.so
quiz: /opt/pcl/pcl_install_dir/lib/libpcl_common.so
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libfreetype.so
quiz: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
quiz: /usr/lib/x86_64-linux-gnu/libz.so
quiz: /usr/lib/x86_64-linux-gnu/libGLEW.so
quiz: /usr/lib/x86_64-linux-gnu/libSM.so
quiz: /usr/lib/x86_64-linux-gnu/libICE.so
quiz: /usr/lib/x86_64-linux-gnu/libX11.so
quiz: /usr/lib/x86_64-linux-gnu/libXext.so
quiz: /usr/lib/x86_64-linux-gnu/libXt.so
quiz: CMakeFiles/quiz.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nivesh/Desktop/pcltest/src/quiz/ransac/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable quiz"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quiz.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quiz.dir/build: quiz

.PHONY : CMakeFiles/quiz.dir/build

CMakeFiles/quiz.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quiz.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quiz.dir/clean

CMakeFiles/quiz.dir/depend:
	cd /home/nivesh/Desktop/pcltest/src/quiz/ransac/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nivesh/Desktop/pcltest/src/quiz/ransac /home/nivesh/Desktop/pcltest/src/quiz/ransac /home/nivesh/Desktop/pcltest/src/quiz/ransac/build /home/nivesh/Desktop/pcltest/src/quiz/ransac/build /home/nivesh/Desktop/pcltest/src/quiz/ransac/build/CMakeFiles/quiz.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quiz.dir/depend

