#pragma once 

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

/**
 * @brief Struct for 2D points read in from the labelled images
 */
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

/**
 * @brief Class containing input/ouput operations for reading and converting labelled image data and writing to ouput images
 */
class ImageBuffer { 
public: 

  /**
   * @brief Empty constructor
   */
    ImageBuffer (); 

  /**
   * @brief Default destructor
   */
    ~ImageBuffer () = default;

  /**
   * @brief Method for reading labelled image feature data from a json file
   * @param filename_ absolute path to the json file to read data from 
   * @param points_ vector to read feature points to 
   * @return read success 
   * @todo update to handle points from multiple labels in an image
   */
    bool readPoints (std::string filename_, std::vector<point>* points_); 

  /**
   * @brief Method for scaling the 2D feature points wrt the origin (top let corner) of the original image
   * @param points_ vector of feature points 
   * @param scale_ scaling factor (output_point = input_point * scale_)
   */
    void scalePoints (std::vector<point>* points_, float scale_);

  /**
   * @brief Method for interpolating points in a feature point vector for a more dense outline of a feature - helps to converge minimization solution
   * @param points_ vector of feature points 
   * @param density_index_ number of points to add btw each original labelled feature point
   * @todo update to add interpolated points at equal intervals rather than at fractions of the length between labelled points 
   */
    void densifyPoints (std::vector<point>* points_, uint8_t density_index_);

  /**
   * @brief Method for converting 2D point set to planar 3D pcl cloud
   * @param points_ vector of feature points (2D)
   * @param cloud_ 3D point cloud to recieve 2D points (x->x, y->y)
   * @param init_z_pos_ initial z value to set for 3D point cloud points
   */
    void populateCloud (std::vector<point>* points_, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, uint16_t init_z_pos_);

  /**
   * @brief Method for converting 3D pcl cloud to 2D point set by projecting into the x-y plane 
   * @param cloud_ 3D point cloud
   * @param points_ 2D point set
   */
    void flattenCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::vector<point>* points_);

  /**
   * @brief Method for writing 2D point set data to an image by setting corresponding pixels to a specified color  
   * @param points_ 2D point set 
   * @param src_file_name_ absolute name of unannotated image
   * @param target_file_name_ absolute path of image annotated image to create (unnanotated image with written data overlayed)
   * @param color_ color to set pixels in output image (default = "black", options = "red", "green", "blue")
   * @return write success 
   * @todo update to interpolate pixels (lines? splines?) to color between points
   */
    bool writeToImage (std::vector<point>* points_, std::string src_file_name_, std::string target_file_name_, std::string color_ = "black");

};

}

