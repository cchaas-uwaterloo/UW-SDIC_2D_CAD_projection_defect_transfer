#include "solver.h"

namespace cam_cad {

Solver::Solver() {}; 

bool Solver::solveOptimization (pcl::PointCloud<pcl::PointXYZ>::ConstPtr CAD_cloud_, std::vector<point> &camera_points_) {
    //define cost funtion

    
    bool converged = false;
    while (!converged) {
        ceres::Problem prob; 
        //ceres::ResidualBlock()
    }

    return true;
}

}
