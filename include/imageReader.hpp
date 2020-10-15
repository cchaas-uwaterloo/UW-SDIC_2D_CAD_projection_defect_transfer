// Class to read in image data from JSON files 

#ifndef CAMCAD_IMAGEREADER_HPP
#define CAMCAD_IMAGEREADER_HPP


#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>

namespace cam_cad { 

struct point { 
    float x; 
    float y;

    point () {
        x = 0; 
        y = 0; 
    }
    
    point (float x_, float y_) {
        x = x_;
        y = y_;
    }
};

class ImageReader { 
public: 
    ImageReader (); 
    ~ImageReader () = default;

    bool readPoints (std::string filename_, std::vector<point>* points_); 
    void scalePoints (std::vector<point>* points_);
    void densifyPoints (std::vector<point>* points_, uint8_t density_index_);
    void populateCloud (std::vector<point>* points_, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);

    //uint16_t return3 (); 

private: 
    //consider moving these and making them arguments to the relevant member functions
    const float CADX_SCALE = 10.0, CADY_SCALE = 10.0;    

};

}

#endif
