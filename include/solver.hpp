// Class to display clouds and 2D point maps 

#ifndef CAMCAD_SOLVER_HPP
#define CAMCAD_SOLVER_HPP

#include <ceres/ceres.h>
#include "imageReader.hpp"

namespace cam_cad { 

class Solver{
public: 
    Solver(); 
    ~Solver() = default; 

    bool solveOptimization (pcl::PointCloud<pcl::PointXYZ>::ConstPtr CAD_cloud_, std::vector<point> &camera_points_);

};


}

#endif