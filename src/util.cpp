#include "util.hpp"

namespace cam_cad {

Util::Util() {}; 

void Util::getCorrespondences(pcl::CorrespondencesPtr corrs_, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr source_coud_,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_,
                              uint16_t max_dist_) {

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
    corr_est.setInputSource(source_coud_);
    corr_est.setInputTarget(target_cloud_);
    corr_est.determineCorrespondences(*corrs_,max_dist_);

}

void Util::originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
    
    uint16_t num_points = cloud_->size();

    // determine min x and y values
    uint32_t min_x = 100000, min_y = 100000;
    for (uint16_t point_index = 0; point_index < num_points; point_index ++) {
        if (cloud_->at(point_index).x < min_x) min_x = cloud_->at(point_index).x;
        if (cloud_->at(point_index).y < min_y) min_y = cloud_->at(point_index).y;
        std::cout << "minx: " << min_x << std::endl;
        std::cout << "miny: " << min_y << std::endl;
    }

    // shift all points back to abutt origin
    for (uint16_t point_index = 0; point_index < num_points; point_index ++) {
        cloud_->at(point_index).x -= min_x;
        cloud_->at(point_index).y -= min_y;
    }

}

void Util::rotateCCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
    // determine max x,y values
    uint32_t max_x = 0, max_y = 0;
    for (uint16_t point_index = 0; point_index < cloud_->size(); point_index ++) {
        if (cloud_->at(point_index).x > max_x) max_x = cloud_->at(point_index).x;
        if (cloud_->at(point_index).y > max_y) max_y = cloud_->at(point_index).y;
    }

    uint32_t min_x = 100000, min_y = 100000;
    for (uint16_t point_index = 0; point_index < cloud_->size(); point_index ++) {
        if (cloud_->at(point_index).x < min_x) min_x = cloud_->at(point_index).x;
        if (cloud_->at(point_index).y < min_y) min_y = cloud_->at(point_index).y;
    }
    
    for (uint16_t index = 0; index < cloud_->size(); index ++) {
        cloud_->at(index).x = max_x - cloud_->at(index).x + min_x;
        uint16_t x_tmp = cloud_->at(index).x;
        cloud_->at(index).x = cloud_->at(index).y;
        cloud_->at(index).y = x_tmp; 
    }
}

}
