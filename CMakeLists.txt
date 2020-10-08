cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)

# set project name
project(Cam_cad_proj VERSION 1.0)

# find packages
find_package(PCL 1.3 REQUIRED Ceres REQUIRED)


add_library(image_reader src/imageReader.cc)


include_directories(includes ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
link_directories(includes ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# add executable
add_executable(cam_cad_proj src/main.cc)

target_link_libraries(cam_cad_proj image_reader ${PCL_LIBRARIES} ${CERES_LIBRARIES}))

install(TARGETS cam_cad_proj DESTINATION bin)



