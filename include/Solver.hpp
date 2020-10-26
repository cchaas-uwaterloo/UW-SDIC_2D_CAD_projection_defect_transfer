// Class to display clouds and 2D point maps 

#ifndef CAMCAD_SOLVER_HPP
#define CAMCAD_SOLVER_HPP

#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <beam_calibration/CameraModel.h>
#include <eigen.h>
#include "imageReader.hpp"
#include "util.hpp"

namespace cam_cad { 

using AlignVec2d = Eigen::aligned_allocator<Eigen::Vector2d>;

class Solver{
public: 
    Solver(Util* util_); 
    ~Solver() = default; 

    bool SolveOptimization (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_, 
                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_);

    std::shared_ptr<ceres::Problem> GetTransform();

private:
    
    //initialize the problem with the residual blocks for each projected point 
    void BuildCeresProblem (std::shared_ptr<ceres::Problem>& problem, pcl::CorrespondencesPtr corrs_, 
                        const std::shared_ptr<beam_calibration::CameraModel> camera_model_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_);

    //initialize the ceres solver options for the problem
    std::shared_ptr<ceres::Problem> SetupCeresOptions (std::string location_);

    //load the initial T_CW to use when solving, default is identity transformation with 2000 z offset
    void LoadInitialPose (std::string location_);

    //set solution options and iterate through ceres solution
    void SolveCeresProblem (const std::shared_ptr<ceres::Problem>& problem, bool output_results);

    //get the camera model data 
    void ReadCameraModel (std::string location_);

    //check the transform against the previous iteration, if each dimmension is unchanged within a certain tolerance, end the optimization
    bool CheckConvergence();

    Eigen::Matrix4d T_CW; //world -> camera transformatin matrix
    Eigen::Matrix4d T_CW_prev; //previous iteration's world -> camera transformation matrix
    Util* util; //local utility object

ceres::Solver::Options ceres_solver_options_;
std::unique_ptr<ceres::LossFunction> loss_function_;
std::unique_ptr<ceres::LocalParameterization> se3_parameterization_;
bool output_results_{true};
uint8_t max_solution_iterations;
std::vector<double> results; //stores the incremental results of the ceres solution

std::shared_ptr<beam_calibration::Ladybug> camera_model_ladybug;

std::shared_ptr<beam_calibration::CameraModel> camera_model;


};


}

#endif