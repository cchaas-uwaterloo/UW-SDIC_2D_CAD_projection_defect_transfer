// Class to display clouds and 2D point maps 

#ifndef CAMCAD_UTIL_HPP
#define CAMCAD_UTIL_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/registration/correspondence_estimation.h>

namespace cam_cad { 

class Util{
public: 
    Util(); 
    ~Util() = default; 

    void getCorrespondences(pcl::CorrespondencesPtr corrs_, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr source_coud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_,
                            uint16_t max_dist_);
    void originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void rotateCCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

};


}

#endif