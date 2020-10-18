// Class to display clouds and 2D point maps 

#ifndef CAMCAD_VISUALIZER_HPP
#define CAMCAD_VISUALIZER_HPP

#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/correspondence_estimation.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include "imageReader.hpp"
#include "beam_matching/pointcloud_display.hpp"
#include <thread>
#include <mutex>
#include <chrono>

namespace cam_cad { 

class Visualizer{
public: 
    Visualizer(const std::string name_); 
    ~Visualizer(); 

    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::string id_);

    // display camera and projected points in 2D without correspondences
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                            std::string id_image_,
                            std::string id_projected_);

    // display camera and projected points in 2D with correspondences
    //NOTE_ take correspondences from projected points to image points
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                            pcl::CorrespondencesConstPtr corrs_,
                            std::string id_image_,
                            std::string id_projected_);

    //starts the visualizer without any point clouds in the vis_thread by calling the spin method 
    void startVis(); 
    void endVis();

private: 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> point_cloud_display;
    boost::shared_ptr<std::thread> vis_thread;
    //mutex for the point_cloud_display object, held by the main thread when updating the visualization params
    std::mutex mtx;

    //vis thread method in which the visualizer spins
    void spin();

};


}

#endif