#include "Solver.hpp"

namespace cam_cad {

Solver::Solver(Util* util) {

    util = util_;

}; 

bool Solver::SolveOptimization (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_, 
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_) {

    bool has_converged = false;
    uint8_t iterations = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // correspondence object tells the cost function which points to compare
    pcl::CorrespondencesPtr proj_corrs (new pcl::Correspondences); 

     // initialize problem 
    std::shared_ptr<ceres::Problem> problem = SetupCeresOptions();

    LoadInitialPose("placeholder");

    // loop problem until it has converged 
    while (!has_converged && iterations < max_solution_iterations) {

        //set previous iteration transform value
        T_CW_prev = T_CW;
        
        // transform, project, and get correspondences
        util->CorrEst(CAD_cloud_, camera_cloud_, T_CW, proj_corrs);

        // update the position of the transformed cloud based on the upated transformation matrix 
        // this is the starting point for the ceres solution for this iteration 
        util->TransformCloud(trans_cloud);

        BuildCeresProblem(problem,proj_corrs,camera_model, camera_cloud_, trans_cloud);

        SolveCeresProblem(problem, true);

        T_CW = vicon_calibration::utils::QuaternionAndTranslationToTransformMatrix(results);

        CheckConvergence();
    }

    if (has_converged) return true;
    else return false;
}

std::shared_ptr<ceres::Problem> Solver::SetupCeresOptions (std::string location_) {
    // set ceres solver params
    ceres_solver_options_.minimizer_progress_to_stdout = false;
    ceres_solver_options_.max_num_iterations = 50;
    ceres_solver_options_.max_solver_time_in_seconds = 1e6;
    ceres_solver_options_.function_tolerance = 1e-8;
    ceres_solver_options_.gradient_tolerance = 1e-10;
    ceres_solver_options_.parameter_tolerance = 1e-8;
    ceres_solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
    ceres_solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;

    // set ceres problem options
    ceres::Problem::Options ceres_problem_options;

    // if we want to manage our own data for these, we can set these flags:
    ceres_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres_problem_options.local_parameterization_ownership =
        ceres::DO_NOT_TAKE_OWNERSHIP;

    std::shared_ptr<ceres::Problem> problem =
        std::make_shared<ceres::Problem>(ceres_problem_options);

    loss_function_ =
        std::unique_ptr<ceres::LossFunction>(new ceres::HuberLoss(1.0));

    std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
        new ceres::QuaternionParameterization());
    std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
        new ceres::IdentityParameterization(3));
    se3_parameterization_ = std::unique_ptr<ceres::LocalParameterization>(
        new ceres::ProductParameterization(quat_parameterization.release(),
                                            identity_parameterization.release()));

    return problem;
}

void Solver::LoadInitialPose (std::string location_) {
    Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity();
    T_CW(2,3) = 2000; 

    Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
    Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
    results{q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};

}

void Solver::BuildCeresProblem(std::shared_ptr<ceres::Problem>& problem, pcl::CorrespondencesPtr corrs_,
                          const std::shared_ptr<beam_calibration::CameraModel> camera_model_,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_) {

    // ----------------------------------------------

    problem1->AddParameterBlock(&(results[0]), 7,
                                se3_parameterization_.get());

    for (int i = 0; i < corrs_.size(); i++) {
        //pixel 
        Eigen::Vector2d pixel (camera_cloud_->at(corrs_->at(i).index_match).x,camera_cloud_->at(corrs_->at(i).index_match).y)

        //P_STRUCT
        Eigen::Vector3d P_STRUCT (camera_cloud_->at(corrs_->at(i).query_match).x,
                                  camera_cloud_->at(corrs_->at(i).query_match).y,
                                  camera_cloud_->at(corrs_->at(i).query_match).z)

        // add residuals
        std::unique_ptr<ceres::CostFunction> cost_function1(
            CeresCameraCostFunction::Create(pixels[i], P_STRUCT,
                                            camera_model));

        problem1->AddResidualBlock(cost_function1.release(), loss_function_.get(),
                                    &(results[0]));
        
    }
}

void Solver::SolveCeresProblem(const std::shared_ptr<ceres::Problem>& problem, bool output_results) {
    ceres::Solver::Summary ceres_summary;
    ceres::Solve(ceres_solver_options_, problem.get(), &ceres_summary);
    if (output_results) {
        LOG_INFO("Done.");
        LOG_INFO("Outputting ceres summary:");
        std::string report = ceres_summary.FullReport();
        std::cout << report << "\n";
    }
}

}