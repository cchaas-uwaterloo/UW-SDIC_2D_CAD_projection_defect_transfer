// Class to read in image data from JSON files 

#ifndef CAMCAD_IMAGEBUFFER_HPP
#define CAMCAD_IMAGEBUFFER_HPP


#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
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

class ImageBuffer { 
public: 
    ImageBuffer (); 
    ~ImageBuffer () = default;

    // input functions
    bool readPoints (std::string filename_, std::vector<point>* points_); 
    void scalePoints (std::vector<point>* points_, float scale_);
    void densifyPoints (std::vector<point>* points_, uint8_t density_index_);
    void populateCloud (std::vector<point>* points_, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, uint16_t init_z_pos_);

    // output functions 

    // should pass empty points vector (otherwise cloud points will be appended to the existing point set)
    void flattenCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::vector<point>* points_);

    // src_file_name_ : existing file (unannotated)
    // target_file_name_ : annotated file to be created
    bool writeToImage (std::vector<point>* points_, std::string src_file_name_, std::string target_file_name_, std::string color_ = "black");

};

}

#endif
