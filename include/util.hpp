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
    Util(); 
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
    void SetLadyBugCamera (uint8_t num_camera_);

    std::shared_ptr<beam_calibration::CameraModel> GetCameraModel();

    void addZeroPoint (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void projectPointsTest (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    void originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void rotateCCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

private: 

    std::shared_ptr<beam_calibration::CameraModel> camera_model;

    std::string camera_type;


};


}

#endif