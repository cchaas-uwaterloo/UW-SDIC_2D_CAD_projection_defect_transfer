cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)

# set project name
project(Cam_cad_proj VERSION 1.0)

set(beam_DIR "~/catkin_ws/build/libbeam") 

# find packages
find_package(PCL 1.3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(beam REQUIRED)


add_library(image_reader src/imageReader.cc visualizer src/visualizer.cc)


include_directories(includes ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
link_directories(includes ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# add executable
add_executable(cam_cad_proj src/main.cc)

target_link_libraries(cam_cad_proj image_reader ${PCL_LIBRARIES} ${CERES_LIBRARIES} beam::beam)

install(TARGETS cam_cad_proj DESTINATION bin)



