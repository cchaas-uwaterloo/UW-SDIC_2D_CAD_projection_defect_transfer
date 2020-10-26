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

    printf("Started... \n");
    
    cam_cad::ImageReader imageReader;
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

    read_success_camera = imageReader.readPoints("/home/nick/projects/beam_robotics/beam_2DCAD_projection/src/P210_north.json", &input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");


    read_success_CAD = imageReader.readPoints("/home/nick/projects/beam_robotics/beam_2DCAD_projection/src/P210_north_crackmap.json", &input_points_CAD);

    if (read_success_camera) printf("CAD data read success\n");

    //*******************************//

    //input cloud operations*********//

    imageReader.densifyPoints(&input_points_camera, 10);
    imageReader.densifyPoints(&input_points_CAD, 10);

    //imageReader.scalePoints(&input_points_CAD, 10);

    printf("points scaled \n");

    imageReader.populateCloud(&input_points_camera, input_cloud_camera, 0);
    imageReader.populateCloud(&input_points_CAD, input_cloud_CAD, 0);
    //imageReader.populateCloud(&input_points_CAD, input_cloud_CAD, 2000);

    printf("clouds populated \n");

    //*******************************//

    //Unit tests*********************//

    mainUtility.getCorrespondences(correspondences, input_cloud_CAD, input_cloud_camera, 1000);
    
    //mainUtility.originCloudxy(input_cloud_CAD);
    //mainUtility.rotateCCWxy(input_cloud_CAD);
    //mainUtility.rotateCCWxy(input_cloud_CAD);
    //mainUtility.rotateCCWxy(input_cloud_CAD);
    
    /************ Test Transformation **********/
    Eigen::Matrix4d T_TEST = Eigen::Matrix4d::Identity(); 
    T_TEST(2,3) = 2000; 

    printf("matrix created \n");

    mainUtility.ReadCameraModel();
    mainUtility.SetLadyBugCamera(6);

    printf("read camera information \n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud;

    transformed_cloud = mainUtility.TransformCloud(input_cloud_CAD, T_TEST);
    projected_cloud = mainUtility.ProjectCloud(transformed_cloud);

    // TEST_ determine cloud correspondences
    //mainUtility.getCorrespondences(correspondences, input_cloud_CAD, input_cloud_camera, 1000);

    //vis1.startVis();

    //printf("started visualizer \n");

    //vis1.displayClouds(input_cloud_camera, input_cloud_CAD, "camera_cloud", "CAD_cloud");
    //vis1.displayClouds(input_cloud_camera, input_cloud_CAD, correspondences, "camera_cloud", "CAD_cloud");
    //vis1.displayClouds(input_cloud_camera, transformed_cloud, projected_cloud, correspondences, "camera_cloud", "transformed_cloud", "projected_cloud");
    //vis1.displayClouds(input_cloud_camera,"camera_cloud");
    //vis1.displayClouds(input_cloud_CAD, "CAD_cloud");

    //*******************************//

    //Solver Block*******************//

    solverUtility->ReadCameraModel();

    cam_cad::Solver solver(solverVisualizer, solverUtility);

    solver.SolveOptimization(input_cloud_CAD, input_cloud_camera);


    //*******************************//

    char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    //vis1.endVis();

    printf("exiting program \n");


    return 0;
}





