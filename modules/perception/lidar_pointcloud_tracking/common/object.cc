#include "modules/perception/lidar_pointcloud_tracking/common/object.h"

namespace apollo {
namespace perception {

Object::Object() {
  cloud.reset(new pcl_util::PointCloud);
  //type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
  velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
}

void Object::clone(const Object& rhs) {
  *this = rhs;
  pcl::copyPointCloud<pcl_util::Point, pcl_util::Point>(*(rhs.cloud), *cloud);
}

}  // namespace perception
}  // namespace apollo
