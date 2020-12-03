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
    
    
int main () {

    cam_cad::ImageReader imageReader;
    cam_cad::Util mainUtility;
    std::vector<cam_cad::point> input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);

    bool read_success_CAD = false; 

    std::string CAD_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/sim_CAD.json";
    std::cout << CAD_file_location << std::endl;

    read_success_CAD = imageReader.readPoints(CAD_file_location, &input_points_CAD);

    if (read_success_CAD) printf("CAD data read success\n");

    imageReader.densifyPoints(&input_points_CAD, 10);

    imageReader.populateCloud(&input_points_CAD, input_cloud_CAD, 0);

    // CAD dimensions
    double max_x_dimension = 4, max_y_dimension = 3.39; 

    float x_scale = 0, y_scale = 0;

    mainUtility.GetCloudScale(input_cloud_CAD, max_x_dimension, max_y_dimension, x_scale, y_scale);

    std::cout << "the cloud x scale: " << x_scale << std::endl;
    std::cout << "the cloud y scale: " << y_scale << std::endl;


    return 0;
}  
    
