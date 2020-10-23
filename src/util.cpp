#include "util.hpp"

namespace cam_cad {

Util::Util() {
    camera_type = "ladybug";
}; 

void Util::getCorrespondences(pcl::CorrespondencesPtr corrs_, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr source_coud_,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_,
                              uint16_t max_dist_) {

    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
    corr_est.setInputSource(source_coud_);
    corr_est.setInputTarget(target_cloud_);
    corr_est.determineCorrespondences(*corrs_,max_dist_);

}

void Util::CorrEst (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                        Eigen::Matrix4d T_CW,
                        pcl::CorrespondencesPtr corrs_) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // transform the CAD cloud points to the camera frame
    this->TransformCloud(CAD_cloud_,T_CW,trans_cloud);

    // project the transformed points to the camera plane
    this->ProjectCloud(trans_cloud, proj_cloud);

    // get correspondences
    this->getCorrespondences(corrs_, proj_cloud, camera_cloud_, 1000);
}

void Util::TransformCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d T_CW,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud_) {
    
    for(uint16_t i=0; i < cloud_->size(); i++) {
        Eigen::Vector4d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z, 1);
        Eigen::Vector4d point_transformed = T_CW*point; 
        pcl::PointXYZ pcl_point_transformed (point_transformed(0), point_transformed(1), point_transformed(2));
        trans_cloud_->push_back(pcl_point_transformed);
    }

}

void Util::ProjectCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud_) {
    
    for(uint16_t i=0; i < cloud_->size(); i++) {
        Eigen::Vector3d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z);
        opt<Eigen::Vector2d> pixel_projected;
        if (camera_type == "base") pixel_projected = camera_model->ProjectPointPrecise(point);
        else if (camera_type == "ladybug") pixel_projected = camera_model_ladybug->ProjectPointPrecise(point);
        pcl::PointXYZ proj_point (pixel_projected.value()(0), pixel_projected.value()(1), 0);
        proj_cloud_->push_back(proj_point);
    }

}

void Util::ReadCameraModel () {
    if (camera_type == "ladybug") {
        std::string file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/ladybug.conf";
        std::cout << file_location << std::endl;
        camera_model = beam_calibration::CameraModel::Create(file_location);
    }
    else if (camera_type == "base") {
        std::string camera_model_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/CamFactorIntrinsics.json";
        camera_model = beam_calibration::CameraModel::Create(camera_model_location);
    }
    

}

void Util::SetLadyBugCamera (uint8_t num_camera_) {
    camera_model_ladybug->SetCameraID(num_camera_);
    camera_type = "ladybug";
}

//TEST_ function
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

//TEST_ function
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
