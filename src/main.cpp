#include <stdio.h>
#include <cstdint>
#include "imageReader.hpp"
#include "visualizer.hpp"
#include "solver.hpp"
#include "util.hpp"


int main () {

    cam_cad::ImageReader imageReader;
    cam_cad::Util mainUtility;
    cam_cad::Visualizer vis1("camera vis");
    //cam_cad::Visualizer vis2("CAD vis");
    std::vector<cam_cad::point>* input_points_camera, * input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera, input_cloud_CAD;
    bool read_success_camera = false, read_success_CAD = false; 

    read_success_camera = imageReader.readPoints("/home/sdic/projects/beam_robotics/beam_2DCAD_projection/src/P210_north.json", input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");

    read_success_CAD = imageReader.readPoints("/home/sdic/projects/beam_robotics/beam_2DCAD_projection/src/P210_north_crackmap.json", input_points_CAD);

    if (read_success_camera) printf("CAD data read success\n");

    imageReader.densifyPoints(input_points_camera, 20);

    imageReader.scalePoints(input_points_CAD);

    imageReader.populateCloud(input_points_camera, input_cloud_camera);
    imageReader.populateCloud(input_points_CAD, input_cloud_CAD);

    //vis1.displayCloud(input_cloud_camera);
    //vis2.displayCloud(input_cloud_CAD);


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





