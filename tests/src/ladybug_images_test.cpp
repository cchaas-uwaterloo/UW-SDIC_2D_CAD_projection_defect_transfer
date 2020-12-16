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


/**
 * @brief Program to test the camera pose estimation solver on a distorted ladybug camera image
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
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);

    //image and CAD data input block//

    bool read_success_camera = false, read_success_CAD = false; 

    std::string camera_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/src/P210_north.json";
    std::string CAD_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/src/P210_north_crackmap.json";
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

    //*******************************//

    //Unit tests*********************//

    //mainUtility.getCorrespondences(correspondences, input_cloud_CAD, input_cloud_camera, 1000);
    
    mainUtility.originCloudxy(input_cloud_CAD);
    //mainUtility.rotateCCWxy(input_cloud_CAD);
    //mainUtility.rotateCCWxy(input_cloud_CAD);
    //mainUtility.rotateCCWxy(input_cloud_CAD);
    
    /************ Test Transformation **********/
    /*
    Eigen::Matrix4d T_TEST = Eigen::Matrix4d::Identity(); 
    T_TEST(2,3) = 200; 

    printf("matrix created \n");

    mainUtility.ReadCameraModel();

    printf("read camera information \n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud;

    transformed_cloud = mainUtility.TransformCloud(input_cloud_CAD, T_TEST);

    printf("transformed cloud \n");
    printf("updating\n");

    projected_cloud = mainUtility.ProjectCloud(transformed_cloud);

    printf("projected cloud \n");
    
    // TEST_ determine cloud correspondences
    //mainUtility.getCorrespondences(correspondences, input_cloud_CAD, input_cloud_camera, 1000);

    vis1.startVis();

    printf("started visualizer \n");

    //vis1.displayClouds(input_cloud_camera, input_cloud_CAD, "camera_cloud", "CAD_cloud");
    //vis1.displayClouds(input_cloud_camera, input_cloud_CAD, correspondences, "camera_cloud", "CAD_cloud");
    vis1.displayClouds(input_cloud_camera, transformed_cloud, projected_cloud, correspondences, "camera_cloud", "transformed_cloud", "projected_cloud");
    //vis1.displayClouds(input_cloud_camera,"camera_cloud");
    //vis1.displayClouds(projected_cloud, "Projected cloud");
    
    */
    //*******************************//

    //Solver Block*******************//

    std::string config_file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/SolutionParameters.json";

    cam_cad::Solver solver(solverVisualizer, solverUtility, config_file_location);

    solver.SolveOptimization(input_cloud_CAD, input_cloud_camera);


    //*******************************//

    char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    vis1.endVis();

    printf("exiting program \n");


    return 0;
}

