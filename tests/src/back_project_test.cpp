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
 * @brief program to test the back projection of crack points onto 
 * the surface of the CAD clound once the solution converges. 
 * The re-transformation back to the xy plane of the CAD cloud 
 * and the crack points and the output to the CAD
 * drawing is also tested. 
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr createTestCrackPoints
    (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_);

int main () {

    printf("Started... \n");
    
    cam_cad::ImageBuffer ImageBuffer;
    cam_cad::Util mainUtility;
    std::shared_ptr<cam_cad::Util> solverUtility (new cam_cad::Util);
    std::shared_ptr<cam_cad::Visualizer> solverVisualizer 
        (new cam_cad::Visualizer ("solution visualizer"));
    std::vector<cam_cad::point> input_points_camera, input_points_CAD; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera 
        (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_CAD 
        (new pcl::PointCloud<pcl::PointXYZ>);

    //image and CAD data input block//

    bool read_success_camera = false, read_success_CAD = false; 

    std::string camera_file_location = 
        "/home/cameron/wkrpt300_images/testing/labelled_images/-1.000000_-1.000000.json";
    std::string CAD_file_location = 
        "/home/cameron/wkrpt300_images/testing/labelled_images/sim_CAD.json";
    std::cout << camera_file_location << std::endl;
    std::cout << CAD_file_location << std::endl;

    read_success_camera = ImageBuffer.readPoints(camera_file_location, &input_points_camera); 

    if (read_success_camera) printf("camera data read success\n");

    read_success_CAD = ImageBuffer.readPoints(CAD_file_location, &input_points_CAD);

    if (read_success_CAD) printf("CAD data read success\n");

    //*******************************//

    //input cloud operations*********//

    ImageBuffer.densifyPoints(&input_points_camera, 2);
    ImageBuffer.densifyPoints(&input_points_CAD, 2);

    printf("points scaled \n");

    ImageBuffer.populateCloud(&input_points_camera, input_cloud_camera, 0);
    ImageBuffer.populateCloud(&input_points_CAD, input_cloud_CAD, 0);

    printf("clouds populated \n");

    
    mainUtility.originCloudxy(input_cloud_CAD);


    //Solver Block*******************//

    std::string config_file_location = 
        "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/SolutionParameters.json";

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

    mainUtility.ReadCameraModel("/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/Radtan_test.json");

    mainUtility.ScaleCloud(input_cloud_CAD, 0.01);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_CAD_cloud = 
        mainUtility.TransformCloud(input_cloud_CAD, T_CS_final);

    pcl::ModelCoefficients::Ptr CAD_plane = 
        mainUtility.GetCloudPlane(transformed_CAD_cloud);

    std::cout << "Model coefficients: " << CAD_plane->values[0] << " " 
                                        << CAD_plane->values[1] << " "
                                        << CAD_plane->values[2] << " " 
                                        << CAD_plane->values[3] << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr crack_points = 
        createTestCrackPoints(input_cloud_camera);

    printf("created test crack points \n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr back_projected_crack_points = 
        mainUtility.BackProject(crack_points, transformed_CAD_cloud, CAD_plane);

    printf("completed back projection \n");

    vis1.startVis(1);

    vis1.displayClouds(transformed_CAD_cloud, back_projected_crack_points, 
        "structure_cloud", "crack_cloud");

    char end = ' ';

    while (end != 'r') {
        cin >> end; 
    }

    vis1.endVis();

    //Re-center CAD cloud with crack points*************//
    cam_cad::Visualizer vis2("back projection visualizer");

    Eigen::Matrix4d T_SC = T_CS_final.inverse();

    pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_crack_points = 
        mainUtility.TransformCloud(back_projected_crack_points, T_SC);

    mainUtility.TransformCloudUpdate(transformed_CAD_cloud, T_SC);

    mainUtility.ScaleCloud(transformed_CAD_cloud, 100);
    mainUtility.ScaleCloud(CAD_crack_points, 100);

    mainUtility.OffsetCloudxy(transformed_CAD_cloud);
    mainUtility.OffsetCloudxy(CAD_crack_points);

    vis2.startVis(10);

    vis2.displayClouds(CAD_crack_points, transformed_CAD_cloud, 
        "crack points", "CAD_cloud");

    char end1 = ' ';

    while (end1 != 'r') {
        cin >> end1; 
    }

    //Write crack data to image******************//
    std::vector<cam_cad::point> output_points_CAD;
    ImageBuffer.flattenCloud(CAD_crack_points, &output_points_CAD);

    bool write_success = 
        ImageBuffer.writeToImage(&output_points_CAD, "/home/cameron/wkrpt300_images/sim_CAD.jpg",
                                 "/home/cameron/wkrpt300_images/sim_CAD_annotated.jpg");

    printf("exiting program \n");

    return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
    createTestCrackPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_camera_) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr crack_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // get center of structure cloud
    uint16_t num_points = input_cloud_camera_->size();

    // determine central x and y values
    float max_x = 0, max_y = 0, min_x = 2048, min_y = 2048;
    for (uint16_t point_index = 0; point_index < num_points; point_index ++) {
        if (input_cloud_camera_->at(point_index).x > max_x) max_x = 
            input_cloud_camera_->at(point_index).x; 
        if (input_cloud_camera_->at(point_index).y > max_y) max_y = 
            input_cloud_camera_->at(point_index).y; 

        if (input_cloud_camera_->at(point_index).x < min_x) min_x = 
            input_cloud_camera_->at(point_index).x; 
        if (input_cloud_camera_->at(point_index).y < min_y) min_y = 
            input_cloud_camera_->at(point_index).y;
    }

    float center_x = min_x + (max_x-min_x)/2;
    float center_y = min_y + (max_y-min_y)/2;

    // build cross at center of structure cloud
    
    for (uint16_t u = center_x-150; u < center_x+150; u ++) {
        pcl::PointXYZ to_add (u,center_y,0);
        crack_cloud->push_back(to_add);
    }
    for (uint16_t v = center_y-150; v < center_y+150; v ++) {
        pcl::PointXYZ to_add (center_x,v,0);
        crack_cloud->push_back(to_add);
    }

    return crack_cloud;

}

