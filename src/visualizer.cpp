#include "visualizer.hpp"

//TODO_: Update to use existing libbeam visualization functions

namespace cam_cad {

Visualizer::Visualizer(const std::string name_) {
  point_cloud_display = boost::make_shared<beam_matching::PointCloudDisplay>(name_);
  point_cloud_display->startSpin();
  num_clouds = 0;
} 

Visualizer::~Visualizer() {
  point_cloud_display->stopSpin();
}

void Visualizer::displayCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_) {
  num_clouds ++;
  point_cloud_display->addPointcloud(cloud_,num_clouds,false);
}

// display camera and projected points in 2D without correspondences
void Visualizer::displayCameraPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_) {
  num_clouds ++;
  point_cloud_display->addPointcloud(image_cloud_, num_clouds, false);

  num_clouds ++; 
  point_cloud_display->addPointcloud(projected_cloud_, num_clouds, false);

}

// display camera and projected points in 2D with correspondences
void Visualizer::displayCameraPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                                    pcl::CorrespondencesConstPtr corrs_) {

  num_clouds ++;
  point_cloud_display->addPointcloud(image_cloud_, num_clouds, false);

  num_clouds ++; 
  point_cloud_display->addPointcloud(projected_cloud_, num_clouds, false);

  uint16_t line_start_index = 0, 
  line_end_index = 1; 

  //illustrate correspondences
  for (uint16_t i = 0; i < corrs_->size(); i++) {
    uint16_t proj_point_index = corrs_->at(i).index_query;
    uint16_t cam_point_index = corrs_->at(i).index_match;
    void addLine(const pcl::PointXYZ &pt1,
                 const pcl::PointXYZ &pt2,
                 int id1,
                 int id2,
                 bool reset_camera = false);
    point_cloud_display->addLine(projected_cloud_->at(proj_point_index), image_cloud_->at(cam_point_index),
                                 line_start_index, line_end_index, false);
    line_start_index += 2;
    line_end_index += 2;
  }

}



}