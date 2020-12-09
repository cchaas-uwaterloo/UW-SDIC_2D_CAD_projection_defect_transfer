#include "Solver.hpp"

namespace cam_cad {

Solver::Solver(std::shared_ptr<Visualizer> vis_, std::shared_ptr<Util> util_, std::string config_file_name_) {
    vis = vis_;
    util = util_;

    ReadSolutionParams(config_file_name_);

    util->ReadCameraModel(cam_intrinsics_file_);
    camera_model = util->GetCameraModel();
}; 

bool Solver::SolveOptimization (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_) {

    bool has_converged = false;
    uint8_t iterations = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // correspondence object tells the cost function which points to compare
    pcl::CorrespondencesPtr proj_corrs (new pcl::Correspondences); 

    util->ScaleCloud(CAD_cloud_,cloud_scale_);

    if (visualize_)
        vis->startVis();

    // transform, project, and get correspondences
    util->CorrEst(CAD_cloud_, camera_cloud_, T_CS, proj_corrs);

    printf("initial pose: \n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CS << sep;

    // transformed cloud is only for the visualizer, the actual ceres solution takes just teh original CAD cloud and the iterative results 
    trans_cloud = util->TransformCloud(CAD_cloud_, T_CS);

    // project cloud for visualizer
    proj_cloud = util->ProjectCloud(trans_cloud);

    // blow up the transformed cloud for visualization
    util->ScaleCloud(trans_cloud,(1/cloud_scale_));

    // set initial error before optimizing
    SetInitialPixelError(proj_cloud, camera_cloud_, proj_corrs);

    printf("ready to start optimization \n");

    // loop problem until it has converged 
    while (!has_converged && iterations < max_solution_iterations_) {

        iterations ++;

        // initialize problem 
        std::shared_ptr<ceres::Problem> problem = SetupCeresOptions();

        printf("Solver iteration %u \n", iterations);

        if (visualize_)
        {
            vis->displayClouds(camera_cloud_, trans_cloud, proj_cloud, proj_corrs, "camera_cloud", "transformed_cloud", "projected_cloud");

            char end = ' ';

            while (end != 'n' && end != 'r') {
                cin >> end; 
            }

            if (end == 'r') return false;
        }

        BuildCeresProblem(problem, proj_corrs, camera_model, camera_cloud_, CAD_cloud_);

        printf("Solving with %zu correspondences \n", proj_corrs->size());

        SolveCeresProblem(problem, true);

        T_CS = util->QuaternionAndTranslationToTransformMatrix(results);

        std::string sep = "\n----------------------------------------\n";
        std::cout << T_CS << sep;

        // transform, project, and get correspondences
        util->CorrEst(CAD_cloud_, camera_cloud_, T_CS, proj_corrs);

        // update the position of the transformed cloud based on the upated transformation matrix for visualization
        trans_cloud = util->TransformCloud(CAD_cloud_, T_CS);

        // project cloud for visualizer
        proj_cloud = util->ProjectCloud(trans_cloud);

        // blow up the transformed CAD cloud for visualization
        util->ScaleCloud(trans_cloud,(1/cloud_scale_));

        if (convergence_type_ == "pixel")
            has_converged = CheckPixelConvergence(proj_cloud, camera_cloud_, proj_corrs, convergence_limit_);

    }

    if (visualize_)
        vis->endVis();
    if (has_converged) return true;
    else return false;
}

Eigen::Matrix4d Solver::GetTransform() {
    return T_CS;
}

std::shared_ptr<ceres::Problem> Solver::SetupCeresOptions () {
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

void Solver::LoadInitialPose (Eigen::Matrix4d &T_) {
    T_CS = T_;

    Eigen::Matrix3d R1 = T_CS.block(0, 0, 3, 3);
    Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
    results = {q1.w(), q1.x(), q1.y(), q1.z(), T_CS(0, 3), T_CS(1, 3), T_CS(2, 3)};

    printf("loaded initial pose\n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CS << sep;

}

void Solver::LoadInitialPose (std::string file_name_) {

    // load file
    nlohmann::json J;
    std::ifstream file(file_name_);
    file >> J;

    int32_t initial_alpha, initial_beta, initial_gamma, initial_x, initial_y, initial_z;

    initial_alpha = J["pose"][0];
    initial_beta = J["pose"][1];
    initial_gamma = J["pose"][2];
    initial_x = J["pose"][3];
    initial_y = J["pose"][4];
    initial_z = J["pose"][5];

    T_CS = Eigen::Matrix4d::Identity();
    Eigen::VectorXd perturbation(6, 1);
    perturbation << initial_alpha, 0, 0, 0, 0, 0; 
    T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
    perturbation << 0, initial_beta, 0, 0, 0, 0;
    T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
    perturbation << 0, 0, initial_gamma, 0, 0, 0;
    T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
    perturbation << 0, 0, 0, initial_x, initial_y, initial_z;
    T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
    Eigen::Matrix3d R1 = T_CS.block(0, 0, 3, 3);
    Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
    results = {q1.w(), q1.x(), q1.y(), q1.z(), T_CS(0, 3), T_CS(1, 3), T_CS(2, 3)};

    printf("loaded initial pose\n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CS << sep;

}

void Solver::LoadInitialPose (std::string file_name_robot_, std::string file_name_struct_) {
    Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity(); //world to camera transform
    Eigen::Matrix4d T_WS = Eigen::Matrix4d::Identity(); //structure to world transform 

    util->LoadInitialPose (file_name_robot_, T_CW);
    util->LoadInitialPose (file_name_struct_, T_WS, true);

    T_CS = T_CW * T_WS; 

    Eigen::Matrix3d R1 = T_CS.block(0, 0, 3, 3);
    Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
    results = {q1.w(), q1.x(), q1.y(), q1.z(), T_CS(0, 3), T_CS(1, 3), T_CS(2, 3)};

    printf("loaded initial pose\n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CS << sep;
}

void Solver::TransformPose (std::string file_name_, bool inverted_) {
    
    util->TransformPose(file_name_, T_CS, inverted_);

    Eigen::Matrix3d R1 = T_CS.block(0, 0, 3, 3);
    Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
    results = {q1.w(), q1.x(), q1.y(), q1.z(), T_CS(0, 3), T_CS(1, 3), T_CS(2, 3)};

    printf("updated initial pose\n");
    std::string sep = "\n----------------------------------------\n";
    std::cout << T_CS << sep;

}

void Solver::SetMaxMinimizerIterations (uint16_t max_iter_) {
    max_ceres_iterations_ = max_iter_;
}

double Solver::GetInitialPixelError () {
    return initial_projection_error_;
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

bool Solver::CheckPixelConvergence(pcl::PointCloud<pcl::PointXYZ>::ConstPtr query_cloud_, pcl::PointCloud<pcl::PointXYZ>::ConstPtr match_cloud_, 
                          pcl::CorrespondencesPtr corrs_, uint16_t pixel_threshold_) {

  float pixel_error = 0;
    
  for (uint16_t i = 0; i < corrs_->size(); i++) {

    uint16_t proj_point_index = corrs_->at(i).index_query;
    uint16_t cam_point_index = corrs_->at(i).index_match;

    float error_x = query_cloud_->at(proj_point_index).x - match_cloud_->at(cam_point_index).x;
    float error_y = query_cloud_->at(proj_point_index).y - match_cloud_->at(cam_point_index).y;

    pixel_error += sqrt(pow(error_x,2) + pow(error_y,2));

  }

  // average pixel error
  pixel_error /= corrs_->size();

  printf ("the average pixel error is: %f\n", pixel_error);

  if (pixel_error <= pixel_threshold_)
    return true;

  return false;

}

void Solver::ReadSolutionParams(std::string file_name_) {
  // load file
  nlohmann::json J;
  std::ifstream file(file_name_);
  file >> J;

  int32_t initial_alpha, initial_beta, initial_gamma, initial_x, initial_y, initial_z;

  // read solution parameters from configuration file
  max_solution_iterations_ = J["max_solution_iterations"];
  max_ceres_iterations_ = J["max_ceres_iterations"];
  convergence_limit_ = J["convergence_limit"];
  initial_alpha = J["initial_alpha"];
  initial_beta = J["initial_beta"];
  initial_gamma = J["initial_gamma"];
  initial_x = J["initial_x"];
  initial_y = J["initial_y"];
  initial_z = J["initial_z"];
  cloud_scale_ = J["cloud_scale"];
  minimizer_progress_to_stdout_ = J["minimizer_progress_to_stdout"];
  max_solver_time_in_seconds_ = J["max_solver_time_in_seconds"];
  function_tolerance_ = J["function_tolerance"];
  gradient_tolerance_ = J["gradient_tolerance"];
  parameter_tolerance_ = J["parameter_tolerance"];
  cam_intrinsics_file_ = J["camera_intrinsics"];
  visualize_ = J["visualize"];
  convergence_type_ = J["convergence_type"];


  // Load default initial pose
  // default rotations are applied successively around x,y,z
  T_CS = Eigen::Matrix4d::Identity();
  Eigen::VectorXd perturbation(6, 1);
  perturbation << initial_alpha, 0, 0, 0, 0, 0; 
  T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
  perturbation << 0, initial_beta, 0, 0, 0, 0;
  T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
  perturbation << 0, 0, initial_gamma, 0, 0, 0;
  T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
  perturbation << 0, 0, 0, initial_x, initial_y, initial_z;
  T_CS = util->PerturbTransformDegM(T_CS, perturbation); 
  Eigen::Matrix3d R1 = T_CS.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
  results = {q1.w(), q1.x(), q1.y(), q1.z(), T_CS(0, 3), T_CS(1, 3), T_CS(2, 3)};

}

void Solver::SetInitialPixelError(pcl::PointCloud<pcl::PointXYZ>::ConstPtr query_cloud_, pcl::PointCloud<pcl::PointXYZ>::ConstPtr match_cloud_, 
                          pcl::CorrespondencesPtr corrs_) {
    double pixel_error = 0;
    
    for (uint16_t i = 0; i < corrs_->size(); i++) {

        uint16_t proj_point_index = corrs_->at(i).index_query;
        uint16_t cam_point_index = corrs_->at(i).index_match;

        double error_x = query_cloud_->at(proj_point_index).x - match_cloud_->at(cam_point_index).x;
        double error_y = query_cloud_->at(proj_point_index).y - match_cloud_->at(cam_point_index).y;

        pixel_error += sqrt(pow(error_x,2) + pow(error_y,2));

    }

    // average pixel error
    pixel_error /= corrs_->size();

    initial_projection_error_ = pixel_error;

}

}