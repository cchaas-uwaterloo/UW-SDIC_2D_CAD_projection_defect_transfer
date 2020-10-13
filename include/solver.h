// Class to display clouds and 2D point maps 

#pragma once

#include <ceres/ceres.h>
#include "imageReader.h"

namespace cam_cad { 

class Solver{
public: 
    Solver(); 
    ~Solver() = default; 

    bool solveOptimization (pcl::PointCloud<pcl::PointXYZ>::ConstPtr CAD_cloud_, std::vector<point> &camera_points_);

};


}