#pragma once

#include <algorithm>
#include <vector>

#include "modules/perception/lidar_pointcloud_tracking/common/pcl_types.h"

namespace apollo {
namespace perception {

class FeatureDescriptor {
 public:
  // @brief initialize feature descriptor
  // @param[IN] cloud: given cloud for feature extraction
  // @return nothing
  explicit FeatureDescriptor(
      apollo::perception::pcl_util::PointCloudPtr cloud) {
    cloud_ = cloud;
  }
  ~FeatureDescriptor() {}

  // @brief compute histogram feature of given cloud
  // @param[IN] bin_size: bin size of histogram
  // @param[OUT] feature: histogram feature of given cloud
  // @return nothing
  void ComputeHistogram(const int bin_size, std::vector<float>* feature);

 private:
  void GetMinMaxCenter();
  apollo::perception::pcl_util::PointCloudPtr cloud_;
  pcl_util::Point min_pt_;
  pcl_util::Point max_pt_;
  pcl_util::Point center_pt_;
};  // class FeatureDescriptor

}  // namespace perception
}  // namespace apollo

