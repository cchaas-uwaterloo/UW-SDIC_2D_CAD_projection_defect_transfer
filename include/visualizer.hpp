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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/make_shared.hpp>
#include <atomic>

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

    // display three clouds with no correspondences
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_,
                       std::string id1_,
                       std::string id2_,
                       std::string id3_);

    // display four clouds with no correspondences
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4_,
                       std::string id1_,
                       std::string id2_,
                       std::string id3_,
                       std::string id4_);

    // display camera and projected points in 2D with correspondences
    //NOTE_ take correspondences from projected points to image points
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                            pcl::CorrespondencesConstPtr corrs_,
                            std::string id_image_,
                            std::string id_projected_);

    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                            pcl::CorrespondencesConstPtr corrs_,
                            std::string id_image_,
                            std::string id_CAD_,
                            std::string id_projected_);

    //starts the visualizer without any point clouds in the vis_thread by calling the spin method 
    void startVis(uint16_t coord_size = 100); 
    void endVis();

private: 
    pcl::visualization::PCLVisualizer::Ptr point_cloud_display;
    std::thread vis_thread;
    //mutex for the point_cloud_display object, held by the main thread when updating the visualization params
    std::mutex mtx;

    std::string display_name;

    std::atomic_flag continueFlag = ATOMIC_FLAG_INIT;
    bool display1_called, display2_called, display3_called, display4_called, display5_called, display6_called;

    //vis thread method in which the visualizer spins
    void spin();

};


}

#endif