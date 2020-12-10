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


pcl::PointCloud<pcl::PointXYZ>::Ptr createTestCrackPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_);

int main () {

    printf("Started... \n");
    
    cam_cad::ImageReader imageReader;
    cam_cad::Util mainUtility;
    std::shared_ptr<cam_cad::Util> solverUtility (new cam_cad::Util);
    std::shared_ptr<cam_cad::Visualizer> solverVisualizer (new cam_cad::Visualizer ("solution visualizer"));
    std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);

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


    //Solver Block*******************//

    std::string config_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/SolutionParameters.json";

    cam_cad::Solver solver(solverVisualizer, solverUtility, config_file_location);

    solver.LoadInitialPose("/home/cameron/wkrpt300_images/testing/poses/-1.000000_-1.000000.json", 
                           "/home/cameron/wkrpt300_images/testing/poses/struct_world.json");

    // robot -> camera 
    solver.TransformPose("/home/cameron/wkrpt300_images/testing/poses/camera_robot.json");

    bool convergence = solver.SolveOptimization(input_cloud_CAD, input_cloud_camera);

    Eigen::Matrix4d T_CS_final;

    if (convergence) {
        printf("\n\n\n\nIt's converged.\n");
        T_CS_final = solver.GetTransform();
        printf("The converged structure -> camera transform is: \n");
        std::string sep = "\n----------------------------------------\n";
        std::cout << T_CS_final << sep;

    } 
    else printf ("It failed.\n");


    //*******************************//

    //Back Projection****************//
    cam_cad::Visualizer vis1("back projection visualizer");

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_CAD_cloud = mainUtility.TransformCloud(input_cloud_CAD, T_CS_final);

    pcl::ModelCoefficients::Ptr CAD_plane = mainUtility.GetCloudPlane(transformed_CAD_cloud);

    std::cout << "Model coefficients: " << CAD_plane->values[0] << " " 
                                        << CAD_plane->values[1] << " "
                                        << CAD_plane->values[2] << " " 
                                        << CAD_plane->values[3] << std::endl;

    char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    vis1.endVis();

    printf("exiting program \n");


    return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createTestCrackPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr crack_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // get center of structure cloud
    uint16_t num_points = input_cloud_camera_->size();

    // determine central x and y values
    float avg_x = 0, avg_y = 0;
    for (uint16_t point_index = 0; point_index < num_points; point_index ++) {
        if (input_cloud_camera_->at(point_index).x > avg_x) avg_x = input_cloud_camera_->at(point_index).x; 
        if (input_cloud_camera_->at(point_index).y > avg_y) avg_y = input_cloud_camera_->at(point_index).y; 
    }

    avg_x /= 2;
    avg_y /= 2;

    // build cross at center of structure cloud

    for (uint16_t u = avg_x-20; u < avg_x+20; u ++) {
        pcl::PointXYZ to_add (u,avg_y,0);
        crack_cloud->push_back(to_add);
    }
    for (uint16_t v = avg_y-20; v < avg_y+20; v ++) {
        pcl::PointXYZ to_add (avg_x,v,0);
        crack_cloud->push_back(to_add);
    }

    return crack_cloud;

}

