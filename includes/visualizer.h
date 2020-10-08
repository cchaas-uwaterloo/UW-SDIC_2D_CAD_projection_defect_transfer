// Class to display clouds and 2D point maps 

#pragma once

#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include "imageReader.h"

namespace cam_cad { 

class Visualizer{
public: 
    Visualizer(); 
    ~Visualizer() = default; 

    pcl::visualization::PCLVisualizer::Ptr displayCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_);
    void displayCameraPlane(const std::vector<point> &points_); 
    void displayCameraPlane(const std::vector<point> &image_points_, const std::vector<point> &projected_points_);

};


}