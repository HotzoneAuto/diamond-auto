#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"
#include "modules/perception/lidar_pointcloud_tracking/common/object.h"
#include "modules/perception/lidar_pointcloud_tracking/tracked_object.h"

namespace apollo {
namespace perception {

class BaseFilter {
 public:
  typedef Object ObjectType;

  BaseFilter() { name_ = "BaseFilter"; }
  virtual ~BaseFilter() {}

  // @brief initialize the state of filter
  // @param[IN] anchor_point: initial anchor point for filtering
  // @param[IN] velocity: initial velocity for filtering
  // @return nothing
  virtual void Initialize(const Eigen::Vector3f& anchor_point,
                          const Eigen::Vector3f& velocity) = 0;

  // @brief predict the state of filter
  // @param[IN] time_diff: time interval for predicting
  // @return predicted states of filtering
  virtual Eigen::VectorXf Predict(const double time_diff) = 0;

  // @brief update filter with object
  // @param[IN] new_object: recently detected object for current updating
  // @param[IN] old_object: last detected object for last updating
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  virtual void UpdateWithObject(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object,
      const double time_diff) = 0;

  // @brief update filter without object
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  virtual void UpdateWithoutObject(const double time_diff) = 0;

  // @brief get state of filter
  // @param[OUT] anchor_point: anchor point of current state
  // @param[OUT] velocity: velocity of current state
  // @return nothing
  virtual void GetState(Eigen::Vector3f* anchor_point,
                        Eigen::Vector3f* velocity) = 0;

  // @brief get state of filter with acceleration
  // @param[OUT] anchor_point: anchor point of current state
  // @param[OUT] velocity: velocity of current state
  // @param[OUT] velocity_acceleration: acceleration of current state
  // @return nothing
  virtual void GetState(Eigen::Vector3f* anchor_point,
                        Eigen::Vector3f* velocity,
                        Eigen::Vector3f* velocity_acceleration) = 0;

  virtual void GetAccelerationGain(Eigen::Vector3f* acceleration_gain) = 0;

  // @brief get online covariance of filter
  // @param[OUT] online_covariance: online covariance
  // @return noting
  virtual void GetOnlineCovariance(Eigen::Matrix3f* online_covariance) = 0;

  // @brief get name of filter
  // @return name of filter
  std::string Name() { return name_; }

 protected:
  std::string name_;
};  // class BaseFilter

}  // namespace perception
}  // namespace apollo

