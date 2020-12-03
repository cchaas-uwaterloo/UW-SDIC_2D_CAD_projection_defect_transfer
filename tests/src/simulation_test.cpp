#include <stdio.h>
#include <cstdint>
#include <iostream>
#include "imageReader.hpp"
#include "visualizer.hpp"
#include "Solver.hpp"
#include "util.hpp"
#include <X11/Xlib.h> 
#include <Eigen/Dense>
#include <Eigen/Geometry>

bool single_image_test ();

void full_array_test ();

int main () {
    

    
    return 0;
}

bool single_image_test () {
    printf("Started... \n");
    
    cam_cad::ImageReader imageReader;
    cam_cad::Util mainUtility;
    std::shared_ptr<cam_cad::Util> solverUtility (new cam_cad::Util);
    std::shared_ptr<cam_cad::Visualizer> solverVisualizer (new cam_cad::Visualizer ("solution visualizer"));
    std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);

    //image and CAD data input block//

    bool read_success_camera = false, read_success_CAD = false; 

    std::string camera_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/src/P210_north.json";
    std::string CAD_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/src/P210_north_crackmap.json";
    std::cout << camera_file_location << std::endl;
    std::cout << CAD_file_location << std::endl;

    read_success_camera = imageReader.readPoints(camera_file_location, &input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");

    read_success_CAD = imageReader.readPoints(CAD_file_location, &input_points_CAD);

    if (read_success_CAD) printf("CAD data read success\n");

    //*******************************//

    //input cloud operations*********//

    imageReader.densifyPoints(&input_points_camera, 10);
    imageReader.densifyPoints(&input_points_CAD, 10);

    //imageReader.scalePoints(&input_points_CAD, 0.01);

    printf("points scaled \n");

    imageReader.populateCloud(&input_points_camera, input_cloud_camera, 0);
    imageReader.populateCloud(&input_points_CAD, input_cloud_CAD, 0);
}