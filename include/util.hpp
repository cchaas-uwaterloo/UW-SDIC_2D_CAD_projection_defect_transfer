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
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <beam_calibration/CameraModel.h>
#include <beam_calibration/Ladybug.h>
#include <beam_calibration/Radtan.h>
#include <beam_calibration/DoubleSphere.h>
#include <beam_calibration/KannalaBrandt.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdio.h>
#include <optional>
#include <nlohmann/json.hpp>

namespace cam_cad { 

#define NUM_CAMERAS 6

class Util{
public: 
    Util(); 
    ~Util() = default; 

    void getCorrespondences(pcl::CorrespondencesPtr corrs_, 
                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_coud_,
                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud_,
                            uint16_t max_dist_);
    
    //get correspondence estimates based on camera pos
    void CorrEst (pcl::PointCloud<pcl::PointXYZ>::ConstPtr CAD_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_,
                        Eigen::Matrix4d &T_CW,
                        pcl::CorrespondencesPtr corrs_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, Eigen::Matrix4d &T_CW);

    void TransformCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d &T_CW);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    Eigen::Matrix4d QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose_);

    std::vector<double> TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T);

    std::shared_ptr<beam_calibration::CameraModel> GetCameraModel();

    void ReadCameraModel (std::string intrinsics_file_path_);

    void SetCameraID (uint8_t cam_ID_);
    
    Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations);

    Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations);


    Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, const int& precision);

    void originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    // used to restore cloud after calling originCloudxy 
    // need to call after originCloudxy with the same utility
    void OffsetCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    void rotateCCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    void GetCloudScale(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, const double max_x_dim_, const double max_y_dim_, float& x_scale_, float& y_scale_);

    void ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, float scale_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, float scale_);

    void ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, float x_scale_, float y_scale_);

    // TODO remove this function, can just use the generalised transformed pose function 
    void LoadInitialPose (std::string file_name_, Eigen::Matrix4d &T_, bool structure_ = false);

    // updates the initial pose by an additional transformation
    // for example, can be used if the initial pose is given for the robot base to transform into the camera frame
    // the given relatve pose is in the world coordinate set and the returned transformation is in the camera coordinate set
    void TransformPose (std::string file_name_, Eigen::Matrix4d &T_, bool inverted_ = false);

    // TODO remove this function, can just use the generalised transform pose function
    // can update the robot -> camera transformation file to include the necesary rotations or just create a new one
    void RemapWorldtoCameraCoords (const double (&world_transform)[6], double (&camera_transform)[6]);

    //return value: [0] = a , [1] = b, [2] = c, [3] = d
    pcl::ModelCoefficients::Ptr GetCloudPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr BackProject(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud_, 
                                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cad_cloud_, 
                                                    pcl::ModelCoefficients::ConstPtr target_plane_);

private: 

    Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps);

    Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V);

    double DegToRad(double d);

    std::shared_ptr<beam_calibration::CameraModel> camera_model;

    double image_offset_x_, image_offset_y_; 
    bool center_image_called_;

};


}

#endif