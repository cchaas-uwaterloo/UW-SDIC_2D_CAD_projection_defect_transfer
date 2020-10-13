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

enum {
    1 : 
}

class Visualizer{
public: 
    Visualizer(std::String name_); 
    ~Visualizer(); 

    void displayCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_);
    void displayCameraPlane(const std::vector<point> &points_); 
    void displayCameraPlane(const std::vector<point> &image_points_, const std::vector<point> &projected_points_);

private: 
    uint16_t num_clouds;
    beam_matching::PointCloudDisplay point_cloud_display;
};


}