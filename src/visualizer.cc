#include "visualizer.h"

namespace cam_cad {

Visualizer::Visualizer() {}; 

pcl::visualization::PCLVisualizer::Ptr Visualizer::displayCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_) {

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_, "display cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "display cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);

}

// display camera points in 2D plane
void displayCameraPlane(const std::vector<point> &points_) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  for (uint16_t i = 0; i < points_.size(); i++) {
    pcl::PointXYZ o;
    o.x = points_[i].x;
    o.y = points_[i].y;
    o.z = 0;
    cloud->push_back(o);
  }

}
// display camera and projected points in 2D with correspondences
void displayCameraPlane(const std::vector<point> &image_points_, const std::vector<point> &projected_points_) {

}

}