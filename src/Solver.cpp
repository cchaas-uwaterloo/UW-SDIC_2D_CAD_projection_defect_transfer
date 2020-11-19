#include "Solver.hpp"

namespace cam_cad {

Solver::Solver(std::shared_ptr<Visualizer> vis_, std::shared_ptr<Util> util_) {
    vis = vis_;
    util = util_;
    camera_model = util->GetCameraModel();
}; 

bool Solver::SolveOptimization (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_) {

    bool has_converged = false;
    uint8_t iterations = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    printf("In solver\n");

    // correspondence object tells the cost function which points to compare
    pcl::CorrespondencesPtr proj_corrs (new pcl::Correspondences); 

    LoadInitialPose("placeholder");

    vis->startVis();

    // transform, project, and get correspondences
    util->CorrEst(CAD_cloud_, camera_cloud_, T_CW, proj_corrs);

    printf("initial pose: \n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CW << sep;

    // update the position of the transformed cloud based on the upated transformation matrix 
    // this is the starting point for the ceres solution for this iteration 
    trans_cloud = util->TransformCloud(CAD_cloud_, T_CW);

    // project cloud for visualizer
    proj_cloud = util->ProjectCloud(trans_cloud);

    printf("ready to start optimization \n");

    // loop problem until it has converged 
    while (!has_converged && iterations < max_solution_iterations_) {

        //set previous iteration transform value
        T_CW_prev = T_CW;

        // initialize problem 
        std::shared_ptr<ceres::Problem> problem = SetupCeresOptions("placeholder");

        printf("Solver iteration %u \n", iterations);

        vis->displayClouds(camera_cloud_, trans_cloud, proj_cloud, proj_corrs, "camera_cloud", "transformed_cloud", "projected_cloud");

        char end = ' ';

        while (end != 'n') {
            cin >> end; 
        }

        BuildCeresProblem(problem, proj_corrs, camera_model, camera_cloud_, CAD_cloud_);

        printf("Solving with %zu correspondences \n", proj_corrs->size());

        SolveCeresProblem(problem, true);

        T_CW = util->QuaternionAndTranslationToTransformMatrix(results);

        std::string sep = "\n----------------------------------------\n";
        std::cout << T_CW << sep;

        // transform, project, and get correspondences
        util->CorrEst(CAD_cloud_, camera_cloud_, T_CW, proj_corrs);

        // update the position of the transformed cloud based on the upated transformation matrix 
        // this is the starting point for the ceres solution for this iteration 
        trans_cloud = util->TransformCloud(CAD_cloud_, T_CW);

        // project cloud for visualizer
        proj_cloud = util->ProjectCloud(trans_cloud);

        iterations ++;

        //has_converged = CheckConvergence(proj_cloud, camera_cloud_, proj_corrs, 20);
    }

    vis->endVis();
    if (has_converged) return true;
    else return false;
}

Eigen::Matrix4d Solver::GetTransform() {
    return T_CW;
}

std::shared_ptr<ceres::Problem> Solver::SetupCeresOptions (std::string location_) {
    // set ceres solver params
    ceres_solver_options_.minimizer_progress_to_stdout = true;
    ceres_solver_options_.max_num_iterations = max_ceres_iterations_;
    ceres_solver_options_.max_solver_time_in_seconds = max_solver_time_in_seconds_;
    ceres_solver_options_.function_tolerance = function_tolerance_;
    ceres_solver_options_.gradient_tolerance = gradient_tolerance_;
    ceres_solver_options_.parameter_tolerance = parameter_tolerance_;
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

    loss_function_ = NULL;

    std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
        new ceres::QuaternionParameterization());
    std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
        new ceres::IdentityParameterization(3));
    se3_parameterization_ = std::unique_ptr<ceres::LocalParameterization>(
        new ceres::ProductParameterization(quat_parameterization.release(),
                                            identity_parameterization.release()));

    return problem;
}

void ScaleCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, float scale_) {
    for (uint16_t i = 0; i < cloud_->size(); i++) {
        cloud_->at(i) *= scale_;
    }
}

//TODO -> read in initial pose, or list of initial poses from json file
void Solver::LoadInitialPose (std::string location_) {
    T_CW = Eigen::Matrix4d::Identity();
    Eigen::VectorXd perturbation(6, 1);
    perturbation << initial_alpha_, initial_beta_, initial_gamma_, initial_x_, initial_y_, initial_z_;  //this should come from a configuration file
    T_CW = util->PerturbTransformDegM(T_CW, perturbation); 
    Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
    Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
    results = {q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};

    printf("loaded initial pose\n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CW << sep;

}

void Solver::BuildCeresProblem(std::shared_ptr<ceres::Problem>& problem, pcl::CorrespondencesPtr corrs_,
                          const std::shared_ptr<beam_calibration::CameraModel> camera_model_,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_) {

    problem->AddParameterBlock(&(results[0]), 7,
                                se3_parameterization_.get());

    printf("added parameter block \n");

    for (int i = 0; i < corrs_->size(); i++) {
        //pixel 
        Eigen::Vector2d pixel (camera_cloud_->at(corrs_->at(i).index_match).x,camera_cloud_->at(corrs_->at(i).index_match).y);

        //P_STRUCT
        Eigen::Vector3d P_STRUCT (cad_cloud_->at(corrs_->at(i).index_query).x,
                                  cad_cloud_->at(corrs_->at(i).index_query).y,
                                  cad_cloud_->at(corrs_->at(i).index_query).z);

        // add residuals
        std::unique_ptr<ceres::CostFunction> cost_function(
        CeresReprojectionCostFunction::Create(pixel, P_STRUCT,
                                            camera_model));

        problem->AddResidualBlock(cost_function.release(), loss_function_.get(),
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

bool Solver::ReadSolutionParams(std::string file_name_) {
  // load file
  nlohmann::json J;
  std::ifstream file(file_name_);
  file >> J;

  /*
  if (!nlohmann::json::accept(file))
  {
    printf("Bad JSON input \n");
    return false;
  }
  */

  max_solution_iterations_ = J["max_solution_iterations"];
  max_ceres_iterations_ = J["max_ceres_iterations"];
  initial_alpha_ = J["initial_alpha"];
  initial_beta_ = J["initial_beta"];
  initial_gamma_ = J["initial_gamma"];
  initial_x_ = J["initial_x"];
  initial_y_ = J["initial_y"];
  initial_z_ = J["initial_z"];
  minimizer_progress_to_stdout_ = J["minimizer_progress_to_stdout"];
  max_solver_time_in_seconds_ = J["max_solver_time_in_seconds"];
  function_tolerance_ = J["function_tolerance"];
  gradient_tolerance_ = J["gradient_tolerance"];
  parameter_tolerance_ = J["parameter_tolerance"];

  return true;

}

}