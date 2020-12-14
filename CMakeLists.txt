cmake_minimum_required(VERSION 3.13)
project(beam_2DCAD_projection VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

set(build_depends
  roscpp
  cv_bridge
  pcl_ros
  geometry_msgs
)

find_package(beam REQUIRED matching utils calibration)

find_package(catkin REQUIRED
  COMPONENTS
    ${build_depends}
)

find_package(Ceres REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${build_depends}
)

# Core libraries
add_library(image_buffer STATIC src/ImageBuffer.cpp)
  
add_library(visualizer STATIC src/visualizer.cpp)
  
add_library(solver STATIC src/Solver.cpp)

add_library(utils STATIC src/util.cpp)

target_link_libraries(image_buffer
  ${OpenCV_LIBS}
)

target_link_libraries(visualizer 
  beam::matching
  beam::filtering
  ${PCl_LIBRARIES}
)

target_link_libraries(utils
  beam::calibration
)

target_link_libraries(solver
   beam::calibration
   beam::optimization
   visualizer
   utils
   ${PCl_LIBRARIES}
   ${CERES_LIBRARIES}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCl_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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
  image_buffer 
  visualizer 
  utils
  solver
)

# add test executables
add_executable(back_project_test tests/src/back_project_test.cpp)
add_dependencies(back_project_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(back_project_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

add_executable(scale_test tests/src/cad_image_scale_test.cpp)
add_dependencies(scale_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(scale_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

add_executable(convergence_test tests/src/convergence_check_test.cpp)
add_dependencies(convergence_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(convergence_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

add_executable(ladybug_images_test tests/src/ladybug_images_test.cpp)
add_dependencies(ladybug_images_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ladybug_images_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

add_executable(load_pose_test tests/src/load_pose_test.cpp)
add_dependencies(load_pose_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(load_pose_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

add_executable(simulation_test tests/src/simulation_test.cpp)
add_dependencies(simulation_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(simulation_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

# Add heuristic test executables
add_executable(init_iterations_test tests/src/heuristics/ceres_iterations_test.cpp)
add_dependencies(init_iterations_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(init_iterations_test
  ${catkin_LIBRARIES} 
  ${PCl_LIBRARIES}
  image_buffer 
  visualizer 
  utils
  solver
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark h header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
