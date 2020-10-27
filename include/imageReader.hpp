// Class to read in image data from JSON files 

#ifndef CAMCAD_IMAGEREADER_HPP
#define CAMCAD_IMAGEREADER_HPP


#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>

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
    void scalePoints (std::vector<point>* points_, float scale_);
    void densifyPoints (std::vector<point>* points_, uint8_t density_index_);
    void populateCloud (std::vector<point>* points_, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, uint16_t init_z_pos_);

    //TODO_ Move these to utilities or solver 
    void originCloudxy (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);
    void rotateCWxy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_);


};

}

#endif
