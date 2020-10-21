#include <stdio.h>
#include <cstdint>
#include <iostream>
#include "imageReader.hpp"
#include "visualizer.hpp"
#include "solver.hpp"
#include "util.hpp"
#include <X11/Xlib.h> 


int main () {

    printf("Started... \n");
    
    cam_cad::ImageReader imageReader;
    cam_cad::Util mainUtility;
    cam_cad::Visualizer vis1("visualizer");
    std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences);

    bool read_success_camera = false, read_success_CAD = false; 

    read_success_camera = imageReader.readPoints("/home/sdic/projects/beam_robotics/beam_2DCAD_projection/src/P210_north.json", &input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");


    read_success_CAD = imageReader.readPoints("/home/sdic/projects/beam_robotics/beam_2DCAD_projection/src/P210_north_crackmap.json", &input_points_CAD);

    if (read_success_camera) printf("CAD data read success\n");

    

    imageReader.densifyPoints(&input_points_camera, 10);
    imageReader.densifyPoints(&input_points_CAD, 10);

    //imageReader.scalePoints(&input_points_CAD, 10);

    printf("points scaled \n");

    imageReader.populateCloud(&input_points_camera, input_cloud_camera, 0);
    imageReader.populateCloud(&input_points_CAD, input_cloud_CAD, 0);
    //imageReader.populateCloud(&input_points_CAD, input_cloud_CAD, 2000);
    
    //imageReader.originCloudxy(input_cloud_CAD);
    imageReader.rotateCWxy(input_cloud_CAD);
    imageReader.rotateCWxy(input_cloud_CAD);
    imageReader.rotateCWxy(input_cloud_CAD);

    // TEST_ determine cloud correspondences
    mainUtility.getCorrespondences(correspondences, input_cloud_CAD, input_cloud_camera, 1000);

    vis1.startVis();
    //vis1.displayClouds(input_cloud_camera, input_cloud_CAD, "camera_cloud", "CAD_cloud");
    vis1.displayClouds(input_cloud_camera, input_cloud_CAD, correspondences, "camera_cloud", "CAD_cloud");
    //vis1.displayClouds(input_cloud_camera,"camera_cloud");
    //vis1.displayClouds(input_cloud_CAD, "CAD_cloud");

    char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    vis1.endVis();

    printf("exiting program \n");

    /*
    //TEST_ generate correspondences
    boost::shared_ptr<pcl::Correspondences> correspondences =
            boost::make_shared<pcl::Correspondences>();
    mainUtility.getCorrespondences(correspondences, input_cloud_CAD, input_cloud_camera, 50);

    //Display initial correspondences
    vis1.displayCameraPlane(input_cloud_camera, input_cloud_CAD, correspondences);
    */


    return 0;
}





