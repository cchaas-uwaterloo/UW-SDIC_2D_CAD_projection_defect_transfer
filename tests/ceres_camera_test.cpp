#define CATCH_CONFIG_MAIN

#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include "util.hpp"
#include "imageReader.hpp"
#include "visualizer.hpp"
#include <beam_calibration/CameraModel.h>
#include <cmath>
#include <math.h>

using AlignVec2d = Eigen::aligned_allocator<Eigen::Vector2d>;

ceres::Solver::Options ceres_solver_options_;
std::unique_ptr<ceres::LossFunction> loss_function_;
std::unique_ptr<ceres::LocalParameterization> se3_parameterization_;
bool output_results_{true};

cam_cad::Util utility;

/********************************************COST FUNCTION********************************/

struct CameraProjectionFunctor {
  CameraProjectionFunctor(
      const std::shared_ptr<beam_calibration::CameraModel>& camera_model)
      : camera_model_(camera_model) {}

  bool operator()(const double* P, double* pixel) const {
    Eigen::Vector3d P_CAMERA_eig{P[0], P[1], P[2]};
    std::optional<Eigen::Vector2d> pixel_projected =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig);
    if (!pixel_projected.has_value()) {
      printf("failed projection \n"); 
      pixel[0] = 5000.00; 
      pixel[1] = 5000.00;
      return false; 
    }
    //printf("projection succeeded: \n");
    pixel[0] = pixel_projected.value()[0];
    pixel[1] = pixel_projected.value()[1];
    return true;
  }

  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
};

struct CeresCameraCostFunction {
  CeresCameraCostFunction(
      Eigen::Vector2d pixel_detected, Eigen::Vector3d P_STRUCT,
      std::shared_ptr<beam_calibration::CameraModel> camera_model)
      : pixel_detected_(pixel_detected),
        P_STRUCT_(P_STRUCT),
        camera_model_(camera_model) {
        compute_projection.reset(new ceres::CostFunctionToFunctor<2, 3>(
          new ceres::NumericDiffCostFunction<CameraProjectionFunctor,
                                           ceres::CENTRAL, 2, 3>(
            new CameraProjectionFunctor(camera_model_))));
  }

  template <typename T>
  bool operator()(const T* const T_CR, T* residuals) const {
    T P_STRUCT[3];
    P_STRUCT[0] = P_STRUCT_.cast<T>()[0];
    P_STRUCT[1] = P_STRUCT_.cast<T>()[1];
    P_STRUCT[2] = P_STRUCT_.cast<T>()[2];

    // rotate and translate point
    T P_CAMERA[3];
    ceres::QuaternionRotatePoint(T_CR, P_STRUCT, P_CAMERA);
    P_CAMERA[0] += T_CR[4];
    P_CAMERA[1] += T_CR[5];
    P_CAMERA[2] += T_CR[6];

    const T* P_CAMERA_const = &(P_CAMERA[0]);

    T pixel_projected[2];
    (*compute_projection)(P_CAMERA_const, &(pixel_projected[0]));

    residuals[0] = pixel_detected_.cast<T>()[0] - pixel_projected[0];
    residuals[1] = pixel_detected_.cast<T>()[1] - pixel_projected[1];

    printf("residuals: %f , %f  \n", residuals[0], residuals[1]);

    return true;
    
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      const Eigen::Vector2d pixel_detected, const Eigen::Vector3d P_STRUCT,
      const std::shared_ptr<beam_calibration::CameraModel> camera_model) {
    return (new ceres::AutoDiffCostFunction<CeresCameraCostFunction, 2, 7>(
        new CeresCameraCostFunction(pixel_detected, P_STRUCT,
                                    camera_model)));
  }

  Eigen::Vector2d pixel_detected_;
  Eigen::Vector3d P_STRUCT_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;
};

/*****************************************************************************************************/

std::shared_ptr<ceres::Problem> SetupCeresProblem() {
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

void SolveProblem(const std::shared_ptr<ceres::Problem>& problem,
                  bool output_results) {
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_solver_options_, problem.get(), &ceres_summary);
  if (output_results) {
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = ceres_summary.FullReport();
    std::cout << report << "\n";
  }
}

TEST_CASE("Test camera optimization") {
  // create keypoints
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      points;
  double max_distance_x = 2, max_distance_y = 2, max_distance_z = 4;
  for (int i = 0; i < 80; i++) {
    double x = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_x;
    double y = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_y;
    double z = ((double)std::rand() / (RAND_MAX)-0) * 1 * max_distance_z;
    Eigen::Vector4d point(x, y, z, 1);
    points.push_back(point);
  }

  // Create intrinsics
  std::string camera_model_location = "/home/cameron/projects/beam_robotics/beam_2DCAD_projection/config/ladybug.conf";
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(camera_model_location);

  // Create initial transform
  Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity();
  T_CW(2,3) = 600; 

  // create perturbed initial
  Eigen::Matrix4d T_CW_pert = T_CW; 
  T_CW_pert(2,3) += 100; 

  //rotate 45 deg around z
  T_CW_pert (0,0) = std::cos(M_PI/4);
  T_CW_pert (0,1) = -std::sin(M_PI/4);
  T_CW_pert (1,0) = std::sin(M_PI/4);
  T_CW_pert (1,1) = std::cos(M_PI/4);

  

  // create projected (detected) points - no noise
  std::vector<Eigen::Vector2d, AlignVec2d> pixels(points.size());
  std::vector<bool> pixels_valid(points.size());
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector4d point_transformed = T_CW * points[i];
    opt<Eigen::Vector2d> pixel = camera_model->ProjectPointPrecise(point_transformed.hnormalized());
    if (pixel.has_value()) {
      pixels_valid[i] = true;
      pixels[i] = pixel.value();
    } else {
      pixels_valid[i] = false;
    }
  }

  // create values to optimize
  // ----------------------------------------------
  // THIS OPTION CAUSES A SEG FAULT FOR SOME REASON
  // std::vector<double> results_perfect_init =
  //     vicon_calibration::utils::TransformMatrixToQuaternionAndTranslation(T_CV);
  // std::vector<double> results_perturbed_init =
  //     vicon_calibration::utils::TransformMatrixToQuaternionAndTranslation(
  //         T_CV_pert);
  // ----------------------------------------------
  Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
  std::vector<double> results_perfect_init{
      q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};
  Eigen::Matrix3d R2 = T_CW_pert.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q2 = Eigen::Quaternion<double>(R2);
  std::vector<double> results_perturbed_init{
      q2.w(),          q2.x(),          q2.y(),         q2.z(),
      T_CW_pert(0, 3), T_CW_pert(1, 3), T_CW_pert(2, 3)};
  // ----------------------------------------------

  // build problems
  std::shared_ptr<ceres::Problem> problem1 = SetupCeresProblem();
  std::shared_ptr<ceres::Problem> problem2 = SetupCeresProblem();

  problem1->AddParameterBlock(&(results_perfect_init[0]), 7,
                              se3_parameterization_.get());
  problem2->AddParameterBlock(&(results_perturbed_init[0]), 7,
                              se3_parameterization_.get());

  for (int i = 0; i < points.size(); i++) {
    if (pixels_valid[i]) {
      Eigen::Vector3d P_VICONBASE = (T_CW * points[i]).hnormalized();

      // add residuals for perfect init
      std::unique_ptr<ceres::CostFunction> cost_function1(
          CeresCameraCostFunction::Create(pixels[i], P_VICONBASE,
                                          camera_model));

      problem1->AddResidualBlock(cost_function1.release(), loss_function_.get(),
                                 &(results_perfect_init[0]));

      // add residuals for perturbed init
      std::unique_ptr<ceres::CostFunction> cost_function2(
          CeresCameraCostFunction::Create(pixels[i], P_VICONBASE,
                                          camera_model));
      problem2->AddResidualBlock(cost_function2.release(), loss_function_.get(),
                                 &(results_perturbed_init[0]));

      // Check that the inputs are correct:
      double P_C[3];
      ceres::QuaternionRotatePoint(&(results_perfect_init[0]),
                                   P_VICONBASE.data(), P_C);
      Eigen::Vector3d point_transformed(P_C[0] + results_perfect_init[4],
                                        P_C[1] + results_perfect_init[5],
                                        P_C[2] + results_perfect_init[6]);
      opt<Eigen::Vector2d> pixels_projected =
          camera_model->ProjectPointPrecise(point_transformed);
      REQUIRE(pixels_projected.value().isApprox(pixels[i], 1e-5));
    }
  }

  LOG_INFO("TESTING WITH PERFECT INITIALIZATION");
  SolveProblem(problem1, output_results_);
  Eigen::Matrix4d T_CV_opt1 =
      utility.QuaternionAndTranslationToTransformMatrix(
          results_perfect_init);
  
  LOG_INFO("TESTING WITH PERTURBED INITIALIZATION");
  SolveProblem(problem2, output_results_);
  Eigen::Matrix4d T_CV_opt2 =
      utility.QuaternionAndTranslationToTransformMatrix(
          results_perturbed_init);
  
  REQUIRE(utility.RoundMatrix(T_CW, 5) ==
          utility.RoundMatrix(T_CV_opt1, 5));
  REQUIRE(utility.RoundMatrix(T_CW, 5) ==
          utility.RoundMatrix(T_CV_opt2, 5));
}