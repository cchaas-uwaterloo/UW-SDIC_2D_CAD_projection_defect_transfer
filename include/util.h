#pragma once 

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

/**
 * @brief General utility class
 * Note: this has been implemented as a class in order to store a local 
 * camera model as well as internally keep track of offsets that have been
 * applied to clouds
 */
class Util{
public: 

  /**
   * @brief Constructor 
   */
    Util(); 

  /**
   * @brief Default destructor
   */
    ~Util() = default; 

  /**
   * @brief Method to get nearest-neighbor correspondences between a source and target cloud 
   * multiple source points can correspond to one target point
   * @param corrs_ correspondences
   * @param source_cloud_ source cloud to link from 
   * @param target_cloud_ target cloud to link to 
   * @param max_dist_ maximum distance to form a correspondence 
   */
    void getCorrespondences(pcl::CorrespondencesPtr corrs_, 
                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_coud_,
                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud_,
                            uint16_t max_dist_);
    
  /**
   * @brief Method to get correspondences between a CAD cloud projection and an image 
   * cloud given transformation matrix
   * @param CAD_cloud_ CAD structure cloud (centered, at correct scale, untransformed)
   * @param camera_cloud_ camera image label cloud
   * @param T_ transformation matrix to apply to CAD cloud before projecting (usually T_CS)
   * @param corrs_ nearest-neighbor correspondences between the CAD cloud projection and the camera image cloud
   * @param offset_type_ type of offset to use for correspondence generation (options: "center", "centroid", "none") 
   */
    void CorrEst (pcl::PointCloud<pcl::PointXYZ>::ConstPtr CAD_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_,
                        Eigen::Matrix4d &T_,
                        pcl::CorrespondencesPtr corrs_,
                        std::string offset_type_);

  /**
   * @brief Method to apply a transform to a point cloud
   * @param cloud_ original point cloud
   * @param T_ transformation matrix 
   * @return transformed point cloud
   */
    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, Eigen::Matrix4d &T_);

  /**
   * @brief Method to apply a transform to a point cloud by updating the original cloud
   * @param cloud_ point cloud to transform
   * @param T_ transformation matrix 
   * @todo since Transform cloud has been changed to take a constant cloud pointer, can just change
   * this to overload TransformCloud
   */
    void TransformCloudUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Matrix4d &T_CW);

  /**
   * @brief Method to use camera model to project a point cloud into the xy plane
   * @param cloud_ point cloud to project
   * @return projected planar cloud in the xy plane
   */
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

  /**
   * @brief Method to convert a vector of quaternions and translations to a transformation matrix
   * @param pose_ vector of quaternions and translations (quaternions followed by translations)
   * @return transformation matrix
   */
    Eigen::Matrix4d QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose_);

  /**
   * @brief Method to convert a transformation matrix to a vector of quaternions and translations
   * @param T_ transformation matrix
   * @return quaternions and translations
   */
    std::vector<double> TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T_);

  /**
   * @brief Accessor method to retrieve camera model 
   * @return camera model
   */
    std::shared_ptr<beam_calibration::CameraModel> GetCameraModel();

  /**
   * @brief Method to read the camera model used by the utility object from a config file
   * @param intrinsics_file_path_ absolute path to the camera configuration file
   */
    void ReadCameraModel (std::string intrinsics_file_path_);

  /**
   * @brief Setter method to set the camera ID used by the camera model
   * this is currently only applicable to the ladybug camera model 
   * @param cam_ID_ ID of the camera intrinsics set to use
   */
    void SetCameraID (uint8_t cam_ID_);
    
  /**
   * @brief Method to apply perturbations to a transform in radians
   * @param T_in_ unperturbed transform
   * @param perturbations_ perturbations (euler angles and translations) to apply
   * @return perturbed transform 
   */
    Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in_,
                                     const Eigen::VectorXd& perturbations_);

  /**
   * @brief Method to apply perturbations to a transform in degrees
   * @param T_in_ unperturbed transform
   * @param perturbations_ perturbations (euler angles and translations) to apply
   * @return perturbed transform 
   */
    Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T_in_,
                                     const Eigen::VectorXd& perturbations_);


  /**
   * @brief Method to round all matrix components to a specified decimal precision
   * @param M_ matrix to round
   * @param precision_ decimal precision
   * @return rounded matrix
   */
    Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, const int& precision);

  /**
   * @brief Method to center a cloud on the origin in the xy plane
   * cloud is centered based on its maximum dimensions in x and y 
   * @param cloud_ cloud to be centered
   */
    void originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

  /**
   * @brief Method to restore a cloud after it has been centered in x and y
   * this can only be used after having called originCloudxy with the same
   * utility object 
   * @param cloud_ cloud to be offset
   */
    void OffsetCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

  /**
   * @brief Method to rotate a point cloud counter clockwise about the z axis by 90 degrees
   * @param cloud_ cloud to be rotated
   */
    void rotateCCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

  /**
   * @brief Method to get the scale in x and y of a cloud with respect to the original structure dimensions
   * @param cloud_ scaled point cloud 
   * @param max_x_dim_ maximum x dimension of the real structure (likely from CAD drawing)
   * @param max_y_dim_ maximum y dimension of the real structure (likely from CAD drawing)
   * @param x_scale_ scale in x direction (CAD unit/pixel)
   * @param y_scale_ scale in y direction (CAD unit/pixel) 
   */
    void GetCloudScale(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, const double max_x_dim_, const double max_y_dim_, float& x_scale_, float& y_scale_);

  /**
   * @brief Method to scale a cloud (in xyz)
   * @param cloud_ cloud to scale 
   * @param scale_ scale to apply (updated cloud = original cloud * scale)
   */
    void ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, float scale_);

  /**
   * @brief Method to scale a cloud (in xyz)
   * @param cloud_ cloud to scale 
   * @param scale_ scale to apply (updated cloud = original cloud * scale)
   * @return scaled cloud
   */
    pcl::PointCloud<pcl::PointXYZ>::Ptr ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_, float scale_);

  /**
   * @brief Method to scale a cloud in x and y with different scales in each dimension
   * @param cloud_ cloud to scale 
   * @param x_scale_ scale to apply in x (updated cloud = original cloud * scale)
   * @param y_scale_ scale to apply in y (updated cloud = original cloud * scale)
   * @todo not sure if we really need this one
   */
    void ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, float x_scale_, float y_scale_);

  /**
   * @brief Method to load initial poses of the structure or robot 
   * @param file_name_ absolute path to json file with initial pose 
   * @param T_ transformation matrix to which the read pose is applied 
   * @param inverted_ optional paramter, if set to true, the inverse of the 
   * transform read in from the file will be applied to T_
   * @todo not sure if we really need this one, can just use the TransformPose,
   * they are pretty much identical
   */
    void LoadInitialPose (std::string file_name_, Eigen::Matrix4d &T_, bool inverted_ = false);

  /**
   * @brief Method to load initial transformation and apply it to the given matrix 
   * @param file_name_ absolute path to json file with initial pose 
   * @param T_ transformation matrix to which the read pose is applied 
   * @param inverted_ optional paramter, if set to true, the inverse of the 
   * transform read in from the file will be applied to T_
   */
    void TransformPose (std::string file_name_, Eigen::Matrix4d &T_, bool inverted_ = false);

  /**
   * @brief Method to perform rotations to convert from world -> camera coordinate system 
   * @param world_transform transformation matrix in the world coordinate system 
   * (z+ axis up, x+ forward, y+ left )
   * @param camera_transform transformation matrix in the camera coordinate system
   * (z+ axis forward, x+ right, y+ down)
   */
    void RemapWorldtoCameraCoords (const double (&world_transform_)[6], double (&camera_transform_)[6]);

  /**
   * @brief Method to get the plane that best fits a cloud 
   * @param cloud_ point cloud
   * @return pcl model coefficients object, planar equation coefficients are given in form: 
   * [0] = a , [1] = b, [2] = c, [3] = d 
   */
    pcl::ModelCoefficients::Ptr GetCloudPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_);

  /**
   * @brief Method to get the plane that best fits a cloud 
   * @param cloud_ point cloud
   * @return pcl model coefficients object, planar equation coefficients are given in form: 
   * [0] = a , [1] = b, [2] = c, [3] = d 
   */
    pcl::PointCloud<pcl::PointXYZ>::Ptr BackProject(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud_, 
                                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cad_cloud_, 
                                                    pcl::ModelCoefficients::ConstPtr target_plane_);

private: 

    pcl::PointXYZ GetCloudCentroid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_); 

    pcl::PointXYZ GetCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_);

    void OffsetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, Eigen::Vector3d offset_);

    Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps);

    Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V);

    double DegToRad(double d);

    std::shared_ptr<beam_calibration::CameraModel> camera_model;

    double image_offset_x_, image_offset_y_; 
    bool center_image_called_;

};


}