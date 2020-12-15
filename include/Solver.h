#pragma once 

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <beam_calibration/CameraModel.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "util.h"
#include "visualizer.h"
#include <stdio.h>
#include "beam_optimization/CamPoseReprojectionCost.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

namespace cam_cad { 

using AlignVec2d = Eigen::aligned_allocator<Eigen::Vector2d>;

/**
 * @brief Class to solve camera pose estimation problem 
 */
class Solver{
public: 

  /**
   * @brief Constructor 
   * @param vis_ visualizer object that will be used by the solver to display the solution if visualization is enabled
   * @param util_ utility object that will be used by the solver 
   * @param config_file_name_ absolute path to the solution configuration json file 
   */
    Solver(std::shared_ptr<Visualizer> vis_, std::shared_ptr<Util> util_, std::string config_file_name_); 

  /**
   * @brief Default destructor 
   */
    ~Solver() = default; 


  /**
   * @brief Method for estimating the camera pose for an image by solving the projection optimization problem
   * @param CAD_cloud_ 3D point cloud generated from the CAD drawing - used to represent a planar surface of the structure
   * @param camera_cloud_ 3D point cloud generated from the camera image
   */
    bool SolveOptimization (pcl::PointCloud<pcl::PointXYZ>::ConstPtr CAD_cloud_, 
                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_);

  /**
   * @brief Accessor method to retrieve the structure - camera transformation matrix 
   * @return stucture - camera transformation matrix (T_CS)
   */
    Eigen::Matrix4d GetTransform();

  /**
   * @brief Method to load the initial camera pose (T_CS) from an existing transform 
   * @param T_ initial structure - camera transformation matrix
   */
    void LoadInitialPose (Eigen::Matrix4d &T_);

  /**
   * @brief Method to load the initial camera pose (T_CS) from a json file 
   * @param file_name_ absolute path to the json file with the initial transform
   */
    void LoadInitialPose (std::string file_name_);

  /**
   * @brief Method to load initial T_CW (world to camera transform) and T_WS (structure to world transform) at the same time from json files
   * @param file_name_robot_ absolute path to json file with the world to camera transform (T_CW)
   * @param file_name_struct_ absolute path to json file with the structure to world transform (T_WS)
   */
    void LoadInitialPose (std::string file_name_robot_, std::string file_name_struct_);

    // used to apply an additional transform to the initial pose 
    // for example, when the initial pose is given in terms of the robot base, here the base -> camera transform can be applied

   /**
    * @brief Method to apply additional transform to pose currently held by this solver object
    * @param file_name_ absolute path to json file with the transform to apply
    * @param inverted_ if set to true, will apply the inverse of the transform given by the file, default is false 
    * @todo overload this function to take existing transform (i.e. Eigen::Matrix4d)
    */
    void TransformPose (std::string file_name_, bool inverted_ = false);

   /**
    * @brief Setter method to set the maximum numer of minimizer iterations for the Ceres solver
    * @param max_iter_ maximum number of iterations for the Ceres solver 
    */
    void SetMaxMinimizerIterations (uint16_t max_iter_);

   /**
    * @brief Accessor method to retrieve the inital pixel error in the projection before the solution was run 
    * error is calculated as the average euclidean distance (in pixels) between the projected points and their 
    * nearest-neighbor correspondences in the image cloud
    */
    double GetInitialPixelError ();

   /**
    * @brief Accessor method to retrieve the number of iterations required for the overal solution to converge
    */
    int GetSolutionIterations ();

private:
    
   /**
    * @brief Method for building the Ceres problem by adding the residual blocks
    * @param problem Ceres problem object
    * @param corrs_ nearest-neighbor correspondences between the CAD cloud projection and the camera cloud
    * @param camera_model_ camera model
    * @param camera_cloud_ target image point cloud 
    * @param cad_cloud_ CAD cloud (un-transformed, centered in x and y, correct scale)
    */
    void BuildCeresProblem (std::shared_ptr<ceres::Problem>& problem, pcl::CorrespondencesPtr corrs_, 
                        const std::shared_ptr<beam_calibration::CameraModel> camera_model_,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cad_cloud_);

   /**
    * @brief Method setting ceres solver options, these are mostly based on parameters read in from the 
    * SolutionParameters.json file or set by public setters
    */
    std::shared_ptr<ceres::Problem> SetupCeresOptions ();

   /**
    * @brief Method to call the ceres solver on the individual ceres problem
    * @param problem ceres problem object
    * @param output_results used to toggle ceres terminal output on and off
    */
    void SolveCeresProblem (const std::shared_ptr<ceres::Problem>& problem, bool output_results);

   /**
    * @brief Method to check the overall problem for convergence by checking the average error in pixels between 
    * the projected points and the image points
    * @param query_cloud_ projected cloud 
    * @param match_cloud_ image cloud 
    * @param corrs_ nearest-neighbor correspondences between the CAD cloud projection and the camera cloud
    * @param pixel_threshold_ threshold for convergence condition 
    */
    bool CheckPixelConvergence(pcl::PointCloud<pcl::PointXYZ>::ConstPtr query_cloud_, pcl::PointCloud<pcl::PointXYZ>::ConstPtr match_cloud_, 
                          pcl::CorrespondencesPtr corrs_, uint16_t pixel_threshold_);

   /**
    * @brief Method to read the solution parameters from the SolutionParameters.json file
    * the initial pose and cad scale set in the file are defaults and can be overwritten 
    * by calling the dedicated setters
    * @param file_name_ absolute path to the solution parameters json file 
    */
    void ReadSolutionParams(std::string file_name_);

   /**
    * @brief Method to save the initial pixel error before the solution for reference
    * @param query_cloud_ projected cloud 
    * @param match_cloud_ image cloud
    * @param corrs_ nearest-neighbor correspondences between the CAD cloud projection and the camera cloud
    */
    void SetInitialPixelError(pcl::PointCloud<pcl::PointXYZ>::ConstPtr query_cloud_, pcl::PointCloud<pcl::PointXYZ>::ConstPtr match_cloud_, 
                          pcl::CorrespondencesPtr corrs_);

    Eigen::Matrix4d T_CS; //structure -> camera transformatin matrix

    std::shared_ptr<Visualizer> vis;
    std::shared_ptr<Util> util;

    ceres::Solver::Options ceres_solver_options_;
    std::unique_ptr<ceres::LossFunction> loss_function_;
    std::unique_ptr<ceres::LocalParameterization> se3_parameterization_;
    bool output_results_{true};

    // Solution parameters
    uint32_t max_solution_iterations_, max_ceres_iterations_; 
    std::string cam_intrinsics_file_, convergence_type_;
    bool minimizer_progress_to_stdout_, transform_progress_to_stdout_, visualize_; 
    uint32_t max_solver_time_in_seconds_;
    double function_tolerance_, gradient_tolerance_, parameter_tolerance_, cloud_scale_, convergence_limit_;

    double initial_projection_error_;

    uint8_t solution_iterations_;

    std::vector<double> results; // stores the incremental results of the ceres solution

    std::shared_ptr<beam_calibration::CameraModel> camera_model;

};


}
