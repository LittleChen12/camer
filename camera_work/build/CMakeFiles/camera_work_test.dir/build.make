# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhu/camera_work

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhu/camera_work/build

# Include any dependencies generated for this target.
include CMakeFiles/camera_work_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/camera_work_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_work_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_work_test.dir/flags.make

CMakeFiles/camera_work_test.dir/main.cpp.o: CMakeFiles/camera_work_test.dir/flags.make
CMakeFiles/camera_work_test.dir/main.cpp.o: ../main.cpp
CMakeFiles/camera_work_test.dir/main.cpp.o: CMakeFiles/camera_work_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhu/camera_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera_work_test.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/camera_work_test.dir/main.cpp.o -MF CMakeFiles/camera_work_test.dir/main.cpp.o.d -o CMakeFiles/camera_work_test.dir/main.cpp.o -c /home/zhu/camera_work/main.cpp

CMakeFiles/camera_work_test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_work_test.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhu/camera_work/main.cpp > CMakeFiles/camera_work_test.dir/main.cpp.i

CMakeFiles/camera_work_test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_work_test.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhu/camera_work/main.cpp -o CMakeFiles/camera_work_test.dir/main.cpp.s

CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o: CMakeFiles/camera_work_test.dir/flags.make
CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o: ../src/CCalibration.cpp
CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o: CMakeFiles/camera_work_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhu/camera_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o -MF CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o.d -o CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o -c /home/zhu/camera_work/src/CCalibration.cpp

CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhu/camera_work/src/CCalibration.cpp > CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.i

CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhu/camera_work/src/CCalibration.cpp -o CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.s

# Object files for target camera_work_test
camera_work_test_OBJECTS = \
"CMakeFiles/camera_work_test.dir/main.cpp.o" \
"CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o"

# External object files for target camera_work_test
camera_work_test_EXTERNAL_OBJECTS =

camera_work_test: CMakeFiles/camera_work_test.dir/main.cpp.o
camera_work_test: CMakeFiles/camera_work_test.dir/src/CCalibration.cpp.o
camera_work_test: CMakeFiles/camera_work_test.dir/build.make
camera_work_test: /usr/local/lib/libopencv_gapi.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_highgui.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_ml.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_objdetect.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_photo.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_stitching.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_video.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_videoio.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_dnn.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_calib3d.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_features2d.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_flann.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_imgproc.so.4.10.0
camera_work_test: /usr/local/lib/libopencv_core.so.4.10.0
camera_work_test: CMakeFiles/camera_work_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhu/camera_work/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable camera_work_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_work_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_work_test.dir/build: camera_work_test
.PHONY : CMakeFiles/camera_work_test.dir/build

CMakeFiles/camera_work_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_work_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_work_test.dir/clean

CMakeFiles/camera_work_test.dir/depend:
	cd /home/zhu/camera_work/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhu/camera_work /home/zhu/camera_work /home/zhu/camera_work/build /home/zhu/camera_work/build /home/zhu/camera_work/build/CMakeFiles/camera_work_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_work_test.dir/depend
