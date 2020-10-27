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
                        Eigen::Matrix4d &T_CW,
                        pcl::CorrespondencesPtr corrs_) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud;

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
        printf("Point to project %f %f %f \n", point.x(), point.y(), point.z());
        std::optional<Eigen::Vector2i> pixel_projected;
        if (camera_type == "base") pixel_projected = camera_model->ProjectPoint(point);
        else if (camera_type == "ladybug") pixel_projected = camera_model->ProjectPoint(point);

        if (pixel_projected.has_value()) {
            pcl::PointXYZ proj_point (pixel_projected.value()(0), pixel_projected.value()(1), 0);
            proj_cloud->push_back(proj_point);
            printf ("pixel project success");
        }
        
    }

    printf ("Points successfully projected to image plane %f \n", proj_cloud->size());

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

void Util::ReadCameraModel () {
    if (camera_type == "ladybug") {
        std::string file_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/ladybug.conf";
        std::cout << file_location << std::endl;
        camera_model = beam_calibration::CameraModel::Create (file_location);
    }

    printf("image width: %d \n", camera_model->GetWidth());
    printf("image height: %d \n", camera_model->GetHeight());
    
}

void Util::SetLadyBugCamera (uint8_t num_camera_) {
    //DEBUG_ test old camera model
    
    //camera_model->SetFrameID(num_camera_);
    //camera_type = "ladybug";
     
    
}

//TEST_ function 
void Util::projectPointsTest (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
    // check if point is behind image plane

    LadybugError lb_error_;
    Eigen::VectorXd intrinsics = camera_model->GetIntrinsics();

    LadybugContext lb_context_;

    lb_error_ = ladybugCreateContext(&lb_context_);

    if (lb_error_ != LADYBUG_OK) {
        printf("Ladybug threw an error \n");
    }

    double focal_length_;
    double cx_;
    double cy_;

    focal_length_ = intrinsics[1];
    cx_ = intrinsics[2];
    cy_ = intrinsics[3];

    printf ("Project points test: focal length = %f \n", focal_length_);
    printf ("Project points test: cx = %f \n", cx_);
    printf ("Project points test: cy = %f \n", cy_);

    for (uint16_t i = 0; i < cloud_->size(); i++) {
        Eigen::Vector3d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z);
        if (point[2] > 0) {
            Eigen::Vector2d coords;
            Eigen::Vector3d x_proj, X_flip;
            Eigen::Matrix3d K;
            K << focal_length_, 0, cx_, 0, focal_length_, cy_, 0, 0, 1;
            // flip the coordinate system to be consistent with opencv convention shown
            // here:
            // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
            X_flip[0] = -point[1]; // x = -y
            X_flip[1] = point[0];  // y = x
            X_flip[2] = point[2];  // z = z
            // project
            x_proj = K * X_flip;
            // normalize
            coords[0] = x_proj[0] / x_proj[2];
            coords[1] = x_proj[1] / x_proj[2];
            Eigen::Vector2d pixel_out = {0, 0};
            lb_error_ = ladybugUnrectifyPixel(lb_context_, 1, coords[0], coords[1],
                                                &pixel_out[0], &pixel_out[1]);
            
            printf("The normalized projected x pixel is: %f \n", coords[0]);
            printf("The rectified x pixel is: %f \n", pixel_out[0]);

        }

    }

}

//TEST_ function
void Util::addZeroPoint (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
    pcl::PointXYZ to_add (0, 0, 0);
    pcl::PointXYZ to_addd (0, 5, 0);
    cloud_->push_back(to_add);
    cloud_->push_back(to_addd);
}

//TEST_ function
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
