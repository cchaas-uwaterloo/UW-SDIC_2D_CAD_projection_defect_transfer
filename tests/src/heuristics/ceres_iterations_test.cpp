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

// Alternate Radtan intrinsics 
/*
    -0.2294924671994032,
    0.18008566892263364,
    -0.0005326294604360527,
    -0.0004378797791316729
*/



void testOne (Eigen::Matrix4d perfect_init_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD_);

cam_cad::ImageReader imageReader;
cam_cad::Util mainUtility;
std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);


int main () {

    printf("Started... \n");

    //image and CAD data input block//

    bool read_success_camera = false, read_success_CAD = false; 

    std::string camera_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/-1.000000_-1.000000.json";
    std::string CAD_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/sim_CAD.json";
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

    printf("clouds populated \n");

    
    mainUtility.originCloudxy(input_cloud_CAD);

    //Perfect inits for each test pose********//
    // Perfect init # 1 (-1,-1)
    Eigen::Matrix4d perfect_init_one;
    perfect_init_one <<     0.999717,   0.00255823,    0.0236701,    -0.843579,
                        -0.000201718,     0.995085,   -0.0990278,     -1.32106,
                            -0.0238071,    0.0989949,     0.994803,      9.63384,
                            0,            0,            0,            1;



    return 0;
}

void testOne (Eigen::Matrix4d perfect_init_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD_) {
    
    //check initial pose with default solution settings 
    std::string config_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/SolutionParameters.json";

    cam_cad::Solver solver(solverVisualizer, solverUtility, config_file_location);

    solver.LoadInitialPose("/home/cameron/wkrpt300_images/testing/poses/-1.000000_-1.000000.json", 
                           "/home/cameron/wkrpt300_images/testing/poses/struct_world.json");

    solver.TransformPose("/home/cameron/wkrpt300_images/testing/poses/camera_robot.json");

    bool convergence = solver.SolveOptimization(input_cloud_CAD, input_cloud_camera);

    if (convergence) {
        printf("\n\n\n\nIt's converged you fucking beutician.\n");
        Eigen::Matrix4d T_CS_final = solver.GetTransform();
        printf("The converged structure -> camera transform is: \n");
        std::string sep = "\n----------------------------------------\n";
        std::cout << T_CS_final << sep;

    } 
    else printf ("It failed. Just like you. Figure it the fuck out.\n");


    //*******************************//
}

