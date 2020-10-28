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

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // transform the CAD cloud points to the camera frame
    trans_cloud = this->TransformCloud(CAD_cloud_,T_CW);

    // project the transformed points to the camera plane
    //proj_cloud = this->ProjectCloud(trans_cloud);
    proj_cloud = this->projectPointsTest(trans_cloud, "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/ladybug.conf");

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
        std::optional<Eigen::Vector2i> pixel_projected;
        if (camera_type == "base") pixel_projected = camera_model->ProjectPoint(point);
        else if (camera_type == "ladybug") pixel_projected = camera_model->ProjectPoint(point);

        if (pixel_projected.has_value()) {
            pcl::PointXYZ proj_point (pixel_projected.value()(0), pixel_projected.value()(1), 0);
            proj_cloud->push_back(proj_point);
            printf ("pixel project success");
        }
        
    }

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
pcl::PointCloud<pcl::PointXYZ>::Ptr Util::projectPointsTest (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, const std::string& file_path) {
    // check if point is behind image plane

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud_test (new pcl::PointCloud<pcl::PointXYZ>);

    LadybugError lb_error_;
    Eigen::VectorXd intrinsics = camera_model->GetIntrinsics();

    unsigned int cam_id_ = 0;
    const unsigned int LB_FULL_WIDTH_ = 2048;
    const unsigned int LB_FULL_HEIGHT_ = 2464;

    double focal_length_;
    double cx_;
    double cy_;

    focal_length_ = intrinsics[1];
    cx_ = intrinsics[2];
    cy_ = intrinsics[3];

    LadybugContext lb_context_;

    lb_error_ = ladybugCreateContext(&lb_context_);

    lb_error_ = ladybugLoadConfig(lb_context_, file_path.c_str());

    lb_error_ = ladybugConfigureOutputImages(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES);
  
    lb_error_ =
      ladybugSetOffScreenImageSize(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES,
                                   LB_FULL_HEIGHT_, LB_FULL_WIDTH_);

    lb_error_ =
      ladybugGetCameraUnitFocalLength(lb_context_, cam_id_, &focal_length_);

    lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cx_, &cy_);

    if (lb_error_ != LADYBUG_OK) {
        printf("Ladybug threw an error \n");
    }

    for (uint16_t i = 0; i < cloud_->size(); i++) {
        Eigen::Vector3d point (cloud_->at(i).x, cloud_->at(i).y, cloud_->at(i).z);
        pcl::PointXYZ p_projected;
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

            if (lb_error_ != LADYBUG_OK) {
                printf("Ladybug threw an error: %s \n", ladybugErrorToString(lb_error_));
            }
            else {
                p_projected.x = pixel_out[0];
                p_projected.y = pixel_out[1];
                p_projected.z = 0;
                proj_cloud_test->push_back(p_projected);
            }
            
        }

    }

    return proj_cloud_test;

}

std::optional<Eigen::Vector2i> Util::projectPointTest (Eigen::Vector3d point_, std::shared_ptr<beam_calibration::CameraModel> camera_model_) {
    
    LadybugError lb_error_;
    Eigen::VectorXd intrinsics = camera_model_->GetIntrinsics();

    printf("got camera model intrinsics \n");


    unsigned int cam_id_ = 0;
    const unsigned int LB_FULL_WIDTH_ = 2048;
    const unsigned int LB_FULL_HEIGHT_ = 2464;

    double focal_length_;
    double cx_;
    double cy_;

    focal_length_ = intrinsics[1];
    cx_ = intrinsics[2];
    cy_ = intrinsics[3];

    LadybugContext lb_context_;

    lb_error_ = ladybugCreateContext(&lb_context_);

    lb_error_ = ladybugLoadConfig(lb_context_, "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/ladybug.conf");

    lb_error_ = ladybugConfigureOutputImages(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES);
  
    lb_error_ =
      ladybugSetOffScreenImageSize(lb_context_, LADYBUG_ALL_RECTIFIED_IMAGES,
                                   LB_FULL_HEIGHT_, LB_FULL_WIDTH_);

    lb_error_ =
      ladybugGetCameraUnitFocalLength(lb_context_, cam_id_, &focal_length_);

    lb_error_ = ladybugGetCameraUnitImageCenter(lb_context_, cam_id_, &cx_, &cy_);

    if (lb_error_ != LADYBUG_OK) {
        printf("Ladybug threw an error \n");
    }

    printf("setup camera intrinsics \n");

    if (point_[2] > 0) {
        Eigen::Vector2d coords;
        Eigen::Vector3d x_proj, X_flip;
        Eigen::Matrix3d K;
        K << focal_length_, 0, cx_, 0, focal_length_, cy_, 0, 0, 1;

        X_flip[0] = -point_[1]; // x = -y
        X_flip[1] = point_[0];  // y = x
        X_flip[2] = point_[2];  // z = z
        // project
        x_proj = K * X_flip;
        // normalize
        coords[0] = x_proj[0] / x_proj[2];
        coords[1] = x_proj[1] / x_proj[2];
        Eigen::Vector2d pixel_out = {0, 0};
        lb_error_ = ladybugUnrectifyPixel(lb_context_, 1, coords[0], coords[1],
                                            &pixel_out[0], &pixel_out[1]);

        Eigen::Vector2i pixel_round_out = {0,0};

        pixel_round_out[0] = std::round(pixel_out[0]);
        pixel_round_out[1] = std::round(pixel_out[1]);

        printf("finished point project \n");

        return pixel_round_out;    
    }

    return {};

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
