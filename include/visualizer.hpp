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
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include "imageReader.hpp"
#include <beam_matching/pointcloud_display.hpp>

namespace cam_cad { 

class Visualizer{
public: 
    Visualizer(const std::string &name_); 
    ~Visualizer(); 

    void displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void displayCameraPlane(const std::vector<point> &points_); 
    void displayCameraPlane(const std::vector<point> &image_points_, const std::vector<point> &projected_points_);

private: 
    uint16_t num_clouds;
    beam_matching::PointCloudDisplay* point_cloud_display;
};


}

#endif