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
#include "beam_calibration/CameraModel.h"
#include <beam_calibration/Ladybug.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdio.h>
#include <optional>

namespace cam_cad { 

#define NUM_CAMERAS 6

class Util{
public: 
    Util(std::string camera_type_ = "ladybug"); 
    ~Util() = default; 

    void getCorrespondences(pcl::CorrespondencesPtr corrs_, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr source_coud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_,
                            uint16_t max_dist_);
    
    //get correspondence estimates based on camera pos
    void CorrEst (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                        Eigen::Matrix4d &T_CW,
                        pcl::CorrespondencesPtr corrs_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d &T_CW);

    void TransformCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d &T_CW);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    Eigen::Matrix4d QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose_);

    void ReadCameraModel ();

    void SetCameraID (uint8_t cam_ID_);
    
    Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations);

    Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations);


    //TEST_ functions

    std::shared_ptr<beam_calibration::CameraModel> GetCameraModel();

    Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, const int& precision);

    void originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void rotateCCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

private: 

    Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps);

    Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V);

    double DegToRad(double d);

    std::shared_ptr<beam_calibration::CameraModel> camera_model;
    //std::shared_ptr<beam_calibration::Ladybug> camera_model;

    std::string camera_type;


};


}

#endif