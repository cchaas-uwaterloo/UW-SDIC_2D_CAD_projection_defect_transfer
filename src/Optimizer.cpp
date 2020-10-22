#include "vicon_calibration/optimization/Optimizer.h"

#include <algorithm>
#include <fstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>

void Optimizer::Solve() {
  LoadConfig();
  ResetViewer();
  CheckInputs();
  AddInitials();

  uint16_t iteration = 0;
  bool converged = false;
  while (!converged) {
    iteration++;
    LOG_INFO("Iteration: %d", iteration);
    Clear();
    GetImageCorrespondences();
    GetLidarCorrespondences();
    GetLoopClosureCorrespondences();
    if (optimizer_params_.match_centroids_on_first_iter_only &&
        iteration == 1) {
      optimizer_params_.match_centroids = false;
    }
    AddImageMeasurements();
    AddLidarMeasurements();
    AddLidarCameraMeasurements();
    Optimize();
    converged = HasConverged(iteration);
    UpdateInitials();
  }
  if (iteration >= optimizer_params_.max_correspondence_iterations) {
    LOG_WARN("Reached max iterations, stopping.");
  } else {
    LOG_INFO("Converged after %d iterations.", iteration);
  }
}