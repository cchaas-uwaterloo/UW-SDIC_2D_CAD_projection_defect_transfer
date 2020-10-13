cmake_minimum_required(VERSION 3.13)
project(cam_CAD_proj VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(build_depends
  roscpp
  cv_bridge
  pcl_ros
  geometry_msgs
)

find_package(beam REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    ${build_depends}
    image_transport
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

# Core library
add_library(${PROJECT_NAME}
  src/main.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    
)
target_link_libraries(${PROJECT_NAME}
  beam::beam
  ${catkin_LIBRARIES}
)

##################################


# add the main executable
add_executable(main src/main.cc)
add_dependencies(main ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES} ${PROJECT_NAME})

# add the solver
add_executable(solver src/solver.cc)
add_dependencies(solver ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(solver ${catkin_LIBRARIES} ${PROJECT_NAME})


# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
