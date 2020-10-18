#include "visualizer.hpp"

namespace cam_cad {

Visualizer::Visualizer(const std::string name_) {
  display_name = name_;
} 

Visualizer::~Visualizer() {
  continue_flag = true;
  display1_called = true;
  display2_called = true;
  display3_called = true; 

}

void Visualizer::startVis() {
  point_cloud_display = boost::make_shared<pcl::visualization::PCLVisualizer>(display_name);
  point_cloud_display->setBackgroundColor (0, 0, 0);
  point_cloud_display->addCoordinateSystem (1.0);
  point_cloud_display->initCameraParameters ();

  //start the visualizer spinning in its own thread
  vis_thread = boost::make_shared<std::thread>(&Visualizer::spin, this);
}

void Visualizer::endVis() {
  vis_thread->join();
}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::string id_) {

  display1_called = true;
  
  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the point cloud, add it
  if(!display1_called) {
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_);
    point_cloud_display->addPointCloud(cloud_, id_);
    
  }
  //otherwise, update the existing cloud
  else 
    point_cloud_display->updatePointCloud(cloud_,id_);

  mtx.unlock();
}

// display camera and projected points in 2D without correspondences
void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                                std::string id_image_,
                                std::string id_projected_) {
  
  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the image cloud, add it
  if(!display2_called) {
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected_);
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    
  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
  }

  mtx.unlock();
}

// display camera and projected points in 2D with correspondences
void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                                pcl::CorrespondencesConstPtr corrs_,
                                std::string id_image_,
                                std::string id_projected_) {

  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the image cloud, add it
  if(!display3_called) {
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected_);
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    
  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
  }    

  //remove all correspondence lines and redraw
  point_cloud_display->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1; 

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

  mtx.unlock();

}

//private threaded functions

void Visualizer::spin() {
  while (!point_cloud_display->wasStopped ())
  {
    if (mtx.try_lock()) {
      point_cloud_display->spinOnce (3);
      mtx.unlock();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}




}