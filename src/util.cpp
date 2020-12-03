#include "util.hpp"

namespace cam_cad {

Util::Util(std::string camera_type_) {
    camera_type = camera_type_;
}

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
                        Eigen::Matrix4d &T_CW,
                        pcl::CorrespondencesPtr corrs_) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // transform the CAD cloud points to the camera frame
    trans_cloud = this->TransformCloud(CAD_cloud_,T_CW);

    // project the transformed points to the camera plane
    proj_cloud = this->ProjectCloud(trans_cloud);

    // get correspondences
    this->getCorrespondences(corrs_, proj_cloud, camera_cloud_, 1000);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Util::TransformCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d &T_CW) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    for(uint16_t i=0; i < cloud_->size(); i++) {
        Eigen::Vector4d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z, 1);
        Eigen::Vector4d point_transformed = T_CW*point; 
        pcl::PointXYZ pcl_point_transformed (point_transformed(0), point_transformed(1), point_transformed(2));
        trans_cloud->push_back(pcl_point_transformed);
    }

    return trans_cloud;

}

void Util::TransformCloudUpdate (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d &T_CW) {
    
    for(uint16_t i=0; i < cloud_->size(); i++) {
        Eigen::Vector4d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z, 1);
        Eigen::Vector4d point_transformed = T_CW*point; 
        pcl::PointXYZ pcl_point_transformed (point_transformed(0), point_transformed(1), point_transformed(2));
        cloud_->at(i) = pcl_point_transformed;
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Util::ProjectCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    for(uint16_t i=0; i < cloud_->size(); i++) {
        Eigen::Vector3d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z);
        std::optional<Eigen::Vector2d> pixel_projected;
        pixel_projected = camera_model->ProjectPointPrecise(point);
        if (pixel_projected.has_value()) {
            pcl::PointXYZ proj_point (pixel_projected.value()(0), pixel_projected.value()(1), 0);
            proj_cloud->push_back(proj_point);
        }
        
    }

    printf("projection size: %zu \n", proj_cloud->size());

    return proj_cloud;

}

Eigen::Matrix4d Util::QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose_) {
    Eigen::Quaternion<double> quaternion{pose_[0], pose_[1], pose_[2], pose_[3]};
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = quaternion.toRotationMatrix();
    T(0, 3) = pose_[4];
    T(1, 3) = pose_[5];
    T(2, 3) = pose_[6];
    return T;
}

std::shared_ptr<beam_calibration::CameraModel> Util::GetCameraModel () {
    return camera_model;
}

void Util::ReadCameraModel (std::string intrinsics_file_path_) {

    camera_model = beam_calibration::CameraModel::Create (intrinsics_file_path_); 
    
}

void Util::SetCameraID (uint8_t cam_ID_){
    //camera_model->SetCameraID(cam_ID_);
    
}

Eigen::Matrix4d Util::PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
  Eigen::Vector3d r_perturb = perturbations.block(0, 0, 3, 1);
  Eigen::Vector3d t_perturb = perturbations.block(3, 0, 3, 1);
  Eigen::Matrix3d R_in = T_in.block(0, 0, 3, 3);
  Eigen::Matrix3d R_out = LieAlgebraToR(r_perturb) * R_in;
  Eigen::Matrix4d T_out;
  T_out.setIdentity();
  T_out.block(0, 3, 3, 1) = T_in.block(0, 3, 3, 1) + t_perturb;
  T_out.block(0, 0, 3, 3) = R_out;
  return T_out;
}

Eigen::Matrix4d Util::PerturbTransformDegM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
  Eigen::VectorXd perturbations_rad(perturbations);
  perturbations_rad[0] = DegToRad(perturbations_rad[0]);
  perturbations_rad[1] = DegToRad(perturbations_rad[1]);
  perturbations_rad[2] = DegToRad(perturbations_rad[2]);
  return PerturbTransformRadM(T_in, perturbations_rad);
}

void Util::originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
    
    uint16_t num_points = cloud_->size();

    // determine average x and y values
    uint32_t avg_x = 0, avg_y = 0;
    for (uint16_t point_index = 0; point_index < num_points; point_index ++) {
        avg_x += cloud_->at(point_index).x; 
        avg_y += cloud_->at(point_index).y; 
    }

    avg_x /= num_points;
    avg_y /= num_points;

    // shift all points back to abutt origin
    for (uint16_t point_index = 0; point_index < num_points; point_index ++) {
        cloud_->at(point_index).x -= avg_x;
        cloud_->at(point_index).y -= avg_y;
    }

}

Eigen::MatrixXd Util::RoundMatrix(const Eigen::MatrixXd& M, const int& precision) {
  Eigen::MatrixXd Mround(M.rows(), M.cols());
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      Mround(i, j) = std::round(M(i, j) * std::pow(10, precision)) /
                     std::pow(10, precision);
    }
  }
  return Mround;
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

void Util::GetCloudScale(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, const double max_x_dim_, const double max_y_dim_, float& x_scale_, float& y_scale_) {
    
    // get max cloud dimensions in x and y
    uint16_t max_x = 0, max_y = 0;
    uint16_t min_x = cloud_->at(0).x, min_y = cloud_->at(0).y;
    for(uint16_t point_index = 0; point_index < cloud_->size(); point_index++) {
        if (cloud_->at(point_index).x > max_x) max_x = cloud_->at(point_index).x;
        if (cloud_->at(point_index).y > max_y) max_y = cloud_->at(point_index).y;
        if (cloud_->at(point_index).x < min_x) min_x = cloud_->at(point_index).x;
        if (cloud_->at(point_index).y < min_y) min_y = cloud_->at(point_index).y;
    }

    // CAD unit/pixel 
    x_scale_ = max_x_dim_/(max_x-min_x);
    y_scale_ = max_y_dim_/(max_y-min_y);

}

void Util::ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, float scale_) {
    for (uint16_t i = 0; i < cloud_->size(); i++) {
        cloud_->at(i).x *= scale_;
        cloud_->at(i).y *= scale_;
        cloud_->at(i).z *= scale_;
    }
}

// Private functions

Eigen::Matrix3d Util::LieAlgebraToR(const Eigen::Vector3d& eps) {
  return SkewTransform(eps).exp();
}

Eigen::Matrix3d Util::SkewTransform(const Eigen::Vector3d& V) {
  Eigen::Matrix3d M;
  M(0, 0) = 0;
  M(0, 1) = -V(2, 0);
  M(0, 2) = V(1, 0);
  M(1, 0) = V(2, 0);
  M(1, 1) = 0;
  M(1, 2) = -V(0, 0);
  M(2, 0) = -V(1, 0);
  M(2, 1) = V(0, 0);
  M(2, 2) = 0;
  return M;
}

double Util::DegToRad(double d) {
  return d * (M_PI / 180);
}

} // namespace cam_cad
