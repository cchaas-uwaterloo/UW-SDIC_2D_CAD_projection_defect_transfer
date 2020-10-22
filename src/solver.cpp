#include "solver.hpp"

namespace cam_cad {

Solver::Solver() {}; 

bool Solver::solveOptimization (pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_, 
                                pcl::PointCloud<pcl::PointXYZ>::ConstPtr camera_cloud_) {

    Util utility; 

    bool converged = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // correspondence object tells the cost function which points to compare
    pcl::CorrespondencesPtr proj_corrs (new pcl::Correspondences); 

    // get initial projection estimate based on initial camera pos to start
    // NOTE_ may want to put this all in a loop to be able to try multiple initial poses if none converge
    corrEst(CAD_cloud_, proj_cloud, camera_model, proj_corrs); 

    //initialize the ceres problem
    ceres::Problem problem; 

    // call build to add cost functions and residual blocks
    buildProblem(&problem);

    // initialize solve options
    initSolveOptions(&problem);

    // solve the problem 
    solveProblem(&problem); 

    return true;
}

void Solver::corrEst(pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud_,
                            const std::shared_ptr<beam_calibration::CameraModel> camera_model_, 
                            pcl::CorrespondencesPtr corrs_,
                            Util* util_) {

     (new pcl::PointCloud<pcl::PointXYZ>); 
    Eigen::Vector2d pixel_projected;

    //project all points to the image plane
    //Note_ in all cases will revert to pcl::cloud for storage as it is the most flexible container for point data
    for (uint16_t index = 0; index < CAD_cloud_->size(); index ++) {
        Eigen::Vector3d cad_point (CAD_cloud_->at(index).x, CAD_cloud_->at(index).y, CAD_cloud_->at(index).z);
        pixel_projected = camera_model_->ProjectPointPrecise(cad_point);

        pcl::PointXYZ proj_point (pixel_projected.x(), pixel_projected.y(), 0);
        proj_cloud_->push_back(proj_point);
    }

    util_->getCorrespondences(corrs_, proj_cloud_, camera_cloud_, 1000);

}

void Solver::setPos(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, uint32_t z_dist_, uint8_t ccw_rotations_, Util* util_) {
    
    //set z distance
    for (uint16_t index = 0; index < cloud_->size(); index++) {
        cloud_->at(index).z = z_dist_;
    }

    // rotate cloud about its center as many times as requested
    for (uint16_t index = 0; index < ccw_rotations_; index ++) {
        util_->rotateCCWxy(cloud_);
    }

}

void Solver::buildProblem(ceres::Problem* problem, pcl::CorrespondencesPtr corrs_,
                          const std::shared_ptr<beam_calibration::CameraModel> camera_model_,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud_) {

    Eigen::Vector2d camera_pixel; 
    Eigen::Vector3d cad_point;

    for (int index = 0; index < corrs_->size(); index++) {
        ceres::CostFunction* cost_function;

        uint16_t cad_index = corrs_->at(index).index_query;
        uint16_t camera_index = corrs_->at(index).index_match;

        //this should work since the projected cloud was built in the same order the CAD cloud was traversed
        cad_point.x() = cad_cloud_->at(cad_index).x;
        cad_point.y() = cad_cloud_->at(cad_index).y;
        cad_point.z() = 0;

        camera_pixel.x() = camera_cloud_->at(camera_index).x;
        camera_pixel.y() = camera_cloud_->at(camera_index).y;

        // Each Residual block takes a point and a camera as input and
        // outputs a 2 dimensional residual.
        cost_function = CeresCameraCostFunction::Create(camera_pixel, cad_point, camera_model_);

        Eigen::Vector2d pixel_detected, Eigen::Vector3d P_CADCLOUD,
            std::shared_ptr<beam_calibration::CameraModel> camera_model

        // TODO_ understand the difference in the arguements passed here and in in cost function constructor
        double* camera =
            cameras + camera_block_size * bal_problem->camera_index()[i];
        double* point = points + point_block_size * bal_problem->point_index()[i];
        problem->AddResidualBlock(cost_function, loss_function, camera, point);
    }
}

}
