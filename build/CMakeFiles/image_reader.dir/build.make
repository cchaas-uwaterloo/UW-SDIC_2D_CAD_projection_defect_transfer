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
CMAKE_SOURCE_DIR = /home/cameron/cam_cad_proj

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cameron/cam_cad_proj/build

# Include any dependencies generated for this target.
include CMakeFiles/image_reader.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/image_reader.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/image_reader.dir/flags.make

CMakeFiles/image_reader.dir/src/imageReader.cc.o: CMakeFiles/image_reader.dir/flags.make
CMakeFiles/image_reader.dir/src/imageReader.cc.o: ../src/imageReader.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cameron/cam_cad_proj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/image_reader.dir/src/imageReader.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image_reader.dir/src/imageReader.cc.o -c /home/cameron/cam_cad_proj/src/imageReader.cc

CMakeFiles/image_reader.dir/src/imageReader.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_reader.dir/src/imageReader.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cameron/cam_cad_proj/src/imageReader.cc > CMakeFiles/image_reader.dir/src/imageReader.cc.i

CMakeFiles/image_reader.dir/src/imageReader.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_reader.dir/src/imageReader.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cameron/cam_cad_proj/src/imageReader.cc -o CMakeFiles/image_reader.dir/src/imageReader.cc.s

# Object files for target image_reader
image_reader_OBJECTS = \
"CMakeFiles/image_reader.dir/src/imageReader.cc.o"

# External object files for target image_reader
image_reader_EXTERNAL_OBJECTS =

libimage_reader.a: CMakeFiles/image_reader.dir/src/imageReader.cc.o
libimage_reader.a: CMakeFiles/image_reader.dir/build.make
libimage_reader.a: CMakeFiles/image_reader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cameron/cam_cad_proj/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libimage_reader.a"
	$(CMAKE_COMMAND) -P CMakeFiles/image_reader.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image_reader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/image_reader.dir/build: libimage_reader.a

.PHONY : CMakeFiles/image_reader.dir/build

CMakeFiles/image_reader.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/image_reader.dir/cmake_clean.cmake
.PHONY : CMakeFiles/image_reader.dir/clean

CMakeFiles/image_reader.dir/depend:
	cd /home/cameron/cam_cad_proj/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cameron/cam_cad_proj /home/cameron/cam_cad_proj /home/cameron/cam_cad_proj/build /home/cameron/cam_cad_proj/build /home/cameron/cam_cad_proj/build/CMakeFiles/image_reader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/image_reader.dir/depend

