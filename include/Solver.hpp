// Class to display clouds and 2D point maps 

#ifndef CAMCAD_SOLVER_HPP
#define CAMCAD_SOLVER_HPP

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <beam_calibration/CameraModel.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "imageReader.hpp"
#include "util.hpp"
#include "visualizer.hpp"
#include <stdio.h>
#include "beam_optimization/CamPoseReprojectionCost.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

namespace cam_cad { 

using AlignVec2d = Eigen::aligned_allocator<Eigen::Vector2d>;

class Solver{
public: 
    Solver(std::shared_ptr<Visualizer> vis_, std::shared_ptr<Util> util_, std::string config_file_name_); 
    ~Solver() = default; 

    bool SolveOptimization (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_);

    Eigen::Matrix4d GetTransform();

    //load the initial T_CW to use when solving from a pose json file 
    void LoadInitialPose (std::string file_name_);

    //load initial T_CW (world to camera transform) and T_WS (structure to world transform) 
    //in this case the form of the solution is the T_CS (structure to camera transform)
    void LoadInitialPose (std::string file_name_robot_, std::string file_name_struct_);

private:
    
    //initialize the problem with the residual blocks for each projected point 
    void BuildCeresProblem (std::shared_ptr<ceres::Problem>& problem, pcl::CorrespondencesPtr corrs_, 
                        const std::shared_ptr<beam_calibration::CameraModel> camera_model_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_);

    //initialize the ceres solver options for the problem
    std::shared_ptr<ceres::Problem> SetupCeresOptions ();

    //set solution options and iterate through ceres solution
    void SolveCeresProblem (const std::shared_ptr<ceres::Problem>& problem, bool output_results);

    //check convergence by determining the error between the projection and 
    bool CheckConvergence(pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud_, pcl::PointCloud<pcl::PointXYZ>::Ptr match_cloud_, 
                          pcl::CorrespondencesPtr corrs_, uint16_t pixel_threshold_);

    //Read solution parameters from the json configuration file, 
    //Note that the initial pose and cad scale set here are config defaults, they can be overwritten by calling the 
    //dedicated setters
    void ReadSolutionParams(std::string file_name_);

    Eigen::Matrix4d T_CW; //world -> camera transformatin matrix
    Eigen::Matrix4d T_CW_prev; //previous iteration's world -> camera transformation matrix

    std::shared_ptr<Visualizer> vis;
    std::shared_ptr<Util> util;

    ceres::Solver::Options ceres_solver_options_;
    std::unique_ptr<ceres::LossFunction> loss_function_;
    std::unique_ptr<ceres::LocalParameterization> se3_parameterization_;
    bool output_results_{true};

    // Solution parameters
    uint32_t max_solution_iterations_, max_ceres_iterations_; 
    std::string cam_intrinsics_file_;

    bool minimizer_progress_to_stdout_; 
    uint32_t max_solver_time_in_seconds_;
    double function_tolerance_, gradient_tolerance_, parameter_tolerance_, cloud_scale_;

    std::vector<double> results; //stores the incremental results of the ceres solution

    std::shared_ptr<beam_calibration::CameraModel> camera_model;

};


}

#endif