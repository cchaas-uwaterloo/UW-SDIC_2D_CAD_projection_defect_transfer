#include "visualizer.hpp"

namespace cam_cad {

Visualizer::Visualizer(const std::string name_) {
  display_name = name_;
} 

Visualizer::~Visualizer() {
  display1_called = false;
  display2_called = false;
  display3_called = false; 
  display4_called = false;

}

void Visualizer::startVis() {
  point_cloud_display = boost::make_shared<pcl::visualization::PCLVisualizer> (display_name);
  point_cloud_display->setBackgroundColor (0, 0, 0);
  point_cloud_display->addCoordinateSystem (1000);
  point_cloud_display->initCameraParameters ();

  printf("Started Vis \n");

  this->continueFlag.test_and_set(std::memory_order_relaxed);

  //start the visualizer spinning in its own thread
  this->vis_thread = std::thread(&Visualizer::spin, this);
}

void Visualizer::endVis() {
  this->continueFlag.clear(std::memory_order_relaxed);
  vis_thread.join();
}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::string id_) {
  
  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the point cloud, add it
  if(!display1_called) {
    point_cloud_display->addPointCloud(cloud_, id_,0);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_);
    point_cloud_display->resetCamera();
    printf("Point cloud added \n");
    
  }
  //otherwise, update the existing cloud
  else 
    point_cloud_display->updatePointCloud(cloud_,id_);

  mtx.unlock();

  display1_called = true;

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
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected_);  
    point_cloud_display->resetCamera();  
  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
  }

  mtx.unlock();

  display2_called = true;

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
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected_); 
  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
  }    

  //remove all correspondence lines and redraw
  point_cloud_display->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1; 
  uint16_t line_id = 0;

  //illustrate correspondences
  for (uint16_t i = 0; i < corrs_->size(); i++) {
    uint16_t proj_point_index = corrs_->at(i).index_query;
    uint16_t cam_point_index = corrs_->at(i).index_match;

    point_cloud_display->addLine(projected_cloud_->at(proj_point_index), image_cloud_->at(cam_point_index),
                                 0, 255, 0, std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id ++;
  }

  mtx.unlock();

  display3_called = true;

}

// display camera, transformed, and projected points with correspondences
void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                        pcl::CorrespondencesConstPtr corrs_,
                        std::string id_image_,
                        std::string id_CAD_,
                        std::string id_projected_) {

  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the image cloud, add it
  if(!display4_called) {
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->addPointCloud(CAD_cloud_, id_CAD_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_CAD_);
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id_projected_); 
    point_cloud_display->resetCamera();
  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(CAD_cloud_, id_CAD_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
    point_cloud_display->resetCamera();
  }    

  //remove all correspondence lines and redraw
  point_cloud_display->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1; 
  uint16_t line_id = 0;

  printf("clouds added to visualizer \n");

  //printf("%n correspondences to add \n", corrs_->size());

  //illustrate correspondences
  for (uint16_t i = 0; i < corrs_->size(); i++) {

    uint16_t proj_point_index = corrs_->at(i).index_query;
    uint16_t cam_point_index = corrs_->at(i).index_match;

    point_cloud_display->addLine(projected_cloud_->at(proj_point_index), image_cloud_->at(cam_point_index),
                                 0, 255, 0, std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id ++;
  }

  mtx.unlock();

  display4_called = true;

}

//private threaded functions

void Visualizer::spin() {
  printf("in vis thread \n");
  while (this->continueFlag.test_and_set(std::memory_order_relaxed) &&
           !(this->point_cloud_display->wasStopped()))
  {
    //printf("vis thread \n");
    //if (mtx.try_lock()) {
      mtx.lock();
      //printf("got mutex for point cloud \n");
      point_cloud_display->spinOnce (3);
      mtx.unlock();
    //}

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}




}