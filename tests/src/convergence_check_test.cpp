#include <stdio.h>
#include <cstdint>
#include <iostream>
#include "ImageBuffer.h"
#include "visualizer.h"
#include "Solver.h"
#include "util.h"
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


int main () {

    printf("Started... \n");
    
    cam_cad::ImageBuffer ImageBuffer;
    cam_cad::Util mainUtility;
    std::shared_ptr<cam_cad::Util> solverUtility (new cam_cad::Util);
    cam_cad::Visualizer vis1("visualizer");
    std::shared_ptr<cam_cad::Visualizer> solverVisualizer (new cam_cad::Visualizer ("solution visualizer"));
    std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);

    //image and CAD data input block//

    bool read_success_camera = false, read_success_CAD = false; 

    std::string camera_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/-3.000000_0.000000.json";
    std::string CAD_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/sim_CAD.json";
    std::cout << camera_file_location << std::endl;
    std::cout << CAD_file_location << std::endl;

    read_success_camera = ImageBuffer.readPoints(camera_file_location, &input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");

    read_success_CAD = ImageBuffer.readPoints(CAD_file_location, &input_points_CAD);

    if (read_success_CAD) printf("CAD data read success\n");

    //*******************************//

    //input cloud operations*********//

    ImageBuffer.densifyPoints(&input_points_camera, 10);
    ImageBuffer.densifyPoints(&input_points_CAD, 10);

    //ImageBuffer.scalePoints(&input_points_CAD, 0.01);

    printf("points scaled \n");

    ImageBuffer.populateCloud(&input_points_camera, input_cloud_camera, 0);
    ImageBuffer.populateCloud(&input_points_CAD, input_cloud_CAD, 0);

    printf("clouds populated \n");

    
    mainUtility.originCloudxy(input_cloud_CAD);


    //Solver Block*******************//

    std::string config_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/SolutionParameters.json";

    cam_cad::Solver solver(solverVisualizer, solverUtility, config_file_location);

    solver.LoadInitialPose("/home/cameron/wkrpt300_images/testing/poses/-3.000000_0.000000.json", 
                           "/home/cameron/wkrpt300_images/testing/poses/struct_world.json");

    // robot -> camera 
    solver.TransformPose("/home/cameron/wkrpt300_images/testing/poses/camera_robot.json");

    bool convergence = solver.SolveOptimization(input_cloud_CAD, input_cloud_camera);

    if (convergence) {
        printf("\n\n\n\nIt's converged.\n");
        Eigen::Matrix4d T_CS_final = solver.GetTransform();
        printf("The converged structure -> camera transform is: \n");
        std::string sep = "\n----------------------------------------\n";
        std::cout << T_CS_final << sep;

    } 
    else printf ("It failed. Just like you. Figure it out.\n");


    //*******************************//

    char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    vis1.endVis();

    printf("exiting program \n");


    return 0;
}

