cmake_minimum_required(VERSION 3.13)
project(beam_2DCAD_projection VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(build_depends
  roscpp
  cv_bridge
  pcl_ros
  geometry_msgs
)

find_package(beam REQUIRED matching utils)

find_package(catkin REQUIRED
  COMPONENTS
    ${build_depends}
)

find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
)

# Core libraries
add_library(image_reader src/imageReader.cpp)
  
add_library(visualizer src/visualizer.cpp)
  
add_library(solver src/solver.cpp)

add_library(utils src/util.cpp)

target_link_libraries(visualizer 
  beam::matching
  beam::filtering
  ${PCl_LIBRARIES}
)

target_link_libraries(pointcloud_display
  beam::matching
  ${PCl_LIBRARIES}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCl_INCLUDE_DIRS}
)


link_directories(${PROJECT_NAME}
  include
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

##################################

# add main executable
add_executable(${PROJECT_NAME} src/main.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  beam::matching 
  beam::filtering
  image_reader 
  visualizer 
  utils
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark hpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
