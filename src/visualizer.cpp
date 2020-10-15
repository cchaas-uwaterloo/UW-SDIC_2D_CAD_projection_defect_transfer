#include "visualizer.hpp"

//TODO_: Update to use existing libbeam visualization functions

namespace cam_cad {

Visualizer::Visualizer(const std::string &name_) {
  point_cloud_display = new beam_matching::PointCloudDisplay(name_);
  point_cloud_display->startSpin();
  num_clouds = 0;
} 

Visualizer::~Visualizer() {
  point_cloud_display->stopSpin();
  delete point_cloud_display;
}

void Visualizer::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
  num_clouds ++;
  point_cloud_display->addPointcloud(cloud_,num_clouds,false);
}

// display camera points in 2D plane
void Visualizer::displayCameraPlane(const std::vector<point> &points_) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  for (uint16_t i = 0; i < points_.size(); i++) {
    pcl::PointXYZ o;
    o.x = points_[i].x;
    o.y = points_[i].y;
    o.z = 0;
    cloud->push_back(o);
  }

  num_clouds ++; 
  point_cloud_display->addPointcloud(cloud,num_clouds,false);
}

// display camera and projected points in 2D with correspondences
void Visualizer::displayCameraPlane(const std::vector<point> &image_points_, const std::vector<point> &projected_points_) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud, cad_cloud; 
  for (uint16_t i = 0; i < image_points_.size(); i++) {
    pcl::PointXYZ o; 
    o.x = image_points_[i].x;
    o.y = image_points_[i].y; 
    o.z = 0; 
    image_cloud->push_back(o);
  }

  for (uint16_t i = 0; i < projected_points_.size(); i++) {
    pcl::PointXYZ o; 
    o.x = projected_points_[i].x;
    o.y = projected_points_[i].y; 
    o.z = 0; 
    
  //TODO_ Find correspondences between projected and camera points and draw lines to illustrate them 

    cad_cloud->push_back(o);
  }

}

}