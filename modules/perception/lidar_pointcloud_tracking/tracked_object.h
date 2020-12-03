#pragma once

#include <memory>
#include <string>
#include <vector>

//#include "Eigen/Core"
#include "modules/perception/lidar_pointcloud_tracking/common/object.h"

namespace apollo {
namespace perception {

struct TrackedObject {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /* NEED TO NOTICE: All the states of track would be collected mainly based on
   * the states of tracked object. Thus, update tracked object's state when you
   * update the state of track !!! */
  TrackedObject() = default;
  explicit TrackedObject(std::shared_ptr<Object> obj_ptr);

  // deep copy (copy point clouds)
  void clone(const TrackedObject& rhs);

  // cloud
  // store transformed object before tracking
  std::shared_ptr<Object> object_ptr;

  Eigen::Vector3f barycenter;

  // bbox
  Eigen::Vector3f center;
  Eigen::Vector3f size;
  Eigen::Vector3f direction;
  Eigen::Vector3f lane_direction;

  // states
  Eigen::Vector3f anchor_point;
  Eigen::Vector3f velocity;
  Eigen::Matrix3f velocity_uncertainty;
  Eigen::Vector3f acceleration;

  // class type
  //ObjectType type;

  // association distance
  // range from 0 to association_score_maximum
  float association_score = 0.0f;
};  // struct TrackedObject

}  // namespace perception
}  // namespace apollo

