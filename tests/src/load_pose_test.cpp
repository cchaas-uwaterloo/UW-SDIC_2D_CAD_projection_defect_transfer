#include <stdio.h>
#include <cstdint>
#include <iostream>
#include "ImageBuffer.hpp"
#include "visualizer.hpp"
#include "Solver.hpp"
#include "util.hpp"
#include <X11/Xlib.h> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdio.h>

void LoadInitialPose (std::string file_name_);

std::shared_ptr<cam_cad::Util> util (new cam_cad::Util());
std::vector<double> results; //stores the incremental results of the ceres solution

    
    
int main () {

    cam_cad::ImageBuffer ImageBuffer;
    cam_cad::Visualizer vis1("visualizer");

    std::vector<cam_cad::point> input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);

    bool read_success_CAD = false; 

    std::string CAD_file_location = "/home/cameron/wkrpt300_images/testing/labelled_images/sim_CAD.json";
    std::cout << CAD_file_location << std::endl;

    read_success_CAD = ImageBuffer.readPoints(CAD_file_location, &input_points_CAD);

    if (read_success_CAD) printf("CAD data read success\n");

    ImageBuffer.densifyPoints(&input_points_CAD, 10);

    ImageBuffer.populateCloud(&input_points_CAD, input_cloud_CAD, 0);

    util->originCloudxy(input_cloud_CAD);

    // CAD dimensions
    
    double max_x_dimension = 4, max_y_dimension = 3.39; 

    float x_scale = 0, y_scale = 0;

    util->GetCloudScale(input_cloud_CAD, max_x_dimension, max_y_dimension, x_scale, y_scale);

    util->ScaleCloud(input_cloud_CAD, x_scale, y_scale);

    Eigen::Matrix4d T_CR = Eigen::Matrix4d::Identity(); // robot to camera transform
    Eigen::Matrix4d T_RW = Eigen::Matrix4d::Identity(); // world to robot transform
    Eigen::Matrix4d T_WS = Eigen::Matrix4d::Identity(); // structure to world transform 
    Eigen::Matrix4d T_CS; 

    util->TransformPose ("/home/cameron/wkrpt300_images/testing/poses/camera_robot.json", T_CR);
    util->LoadInitialPose ("/home/cameron/wkrpt300_images/testing/poses/-1.000000_-1.000000.json", T_RW);
    util->LoadInitialPose ("/home/cameron/wkrpt300_images/testing/poses/struct_world.json", T_WS, true);

    T_CS = T_CR * T_RW * T_WS; 

    util->TransformCloudUpdate(input_cloud_CAD, T_CS);

    // Sanity check the transformed cloud projection:
    util->ReadCameraModel("/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/Radtan_test.json");
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud = util->ProjectCloud(input_cloud_CAD);

    // scale up the structure cloud for visualization
    util->ScaleCloud(input_cloud_CAD, 1/x_scale);


    vis1.startVis(100);

    vis1.displayClouds(input_cloud_CAD, projected_cloud, "input_cloud_CAD", "projected_cloud");

        char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    vis1.endVis();

    printf("exiting program \n");

    return 0;
}  


    