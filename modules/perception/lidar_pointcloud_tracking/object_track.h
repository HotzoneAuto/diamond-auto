#pragma once

#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "boost/shared_ptr.hpp"

#include "modules/perception/lidar_pointcloud_tracking/common/object.h"
#include "modules/perception/lidar_pointcloud_tracking/base_filter.h"
#include "modules/perception/lidar_pointcloud_tracking/tracked_object.h"

namespace apollo {
namespace perception {

class ObjectTrack {
 public:
  explicit ObjectTrack(std::shared_ptr<TrackedObject> obj);
  ~ObjectTrack();

  // @brief set track cached history size maximum
  // @param[IN] track_cached_history_size_maximum: track cached history size
  // maximum
  // @return true if set successfully, otherwise return false
  static bool SetTrackCachedHistorySizeMaximum(
      const int track_cached_history_size_maximum);

  // @brief set acceleration noise maximum
  // @param[IN] acceleration_noise_maximum: acceleration noise maximum
  // @return true if set successfully, otherwise return false
  static bool SetAccelerationNoiseMaximum(
      const double acceleration_noise_maximum);

  // @brief set speed noise maximum
  // @param[IN] speed noise maximum: speed noise maximum
  // @return true if set successfully, otherwise return false
  static bool SetSpeedNoiseMaximum(const double speed_noise_maximum);

  // @brief get next available track id
  // @return next available track id
  static int GetNextTrackId();

  // @brief predict the state of track
  // @param[IN] time_diff: time interval for predicting
  // @return predicted states of track
  Eigen::VectorXf Predict(const double time_diff);

  // @brief update track with object
  // @param[IN] new_object: recent detected object for current updating
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithObject(std::shared_ptr<TrackedObject>* new_object,
                        const double time_diff);

  // @brief update track without object
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithoutObject(const double time_diff);

  // @brief update track without object with given predicted state
  // @param[IN] predict_state: given predicted state of track
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithoutObject(const Eigen::VectorXf& predict_state,
                           const double time_diff);

 protected:
  // @brief smooth velocity over track history
  // @param[IN] new_object: new detected object for updating
  // @param[IN] time_diff: time interval from last updating
  // @return nothing
  void SmoothTrackVelocity(const std::shared_ptr<TrackedObject>& new_object,
                           const double time_diff);

  // @brief smooth orientation over track history
  // @return nothing
  void SmoothTrackOrientation();

  // @brief check whether track is static or not
  // @param[IN] new_object: new detected object just updated
  // @param[IN] time_diff: time interval between last two updating
  // @return true if track is static, otherwise return false
  bool CheckTrackStaticHypothesis(const std::shared_ptr<Object>& new_object,
                                  const double time_diff);

  // @brief sub strategy of checking whether track is static or not via
  // considering the velocity angle change
  // @param[IN] new_object: new detected object just updated
  // @param[IN] time_diff: time interval between last two updating
  // @return true if track is static, otherwise return false
  bool CheckTrackStaticHypothesisByVelocityAngleChange(
      const std::shared_ptr<Object>& new_object, const double time_diff);

 private:
  ObjectTrack();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // algorithm setup
  BaseFilter* filter_;

  // basic info
  int idx_;
  int age_;
  int total_visible_count_;
  int consecutive_invisible_count_;
  double period_;

  std::shared_ptr<TrackedObject> current_object_;

  // history
  std::deque<std::shared_ptr<TrackedObject>> history_objects_;

  // states
  // NEED TO NOTICE: All the states would be collected mainly based on states
  // of tracked object. Thus, update tracked object when you update the state
  // of track !!!!!
  bool is_static_hypothesis_;
  Eigen::Vector3f belief_anchor_point_;
  Eigen::Vector3f belief_velocity_;
  Eigen::Matrix3f belief_velocity_uncertainty_;
  Eigen::Vector3f belief_velocity_accelaration_;

 private:
  // global setup
  static int s_track_idx_;
  static int s_track_cached_history_size_maximum_;
  static double s_speed_noise_maximum_;
  static double s_acceleration_noise_maximum_;

  //DISALLOW_COPY_AND_ASSIGN(ObjectTrack);
};  // class ObjectTrack

typedef ObjectTrack* ObjectTrackPtr;

class ObjectTrackSet {
 public:
  ObjectTrackSet();
  ~ObjectTrackSet();

  // @brief set track consecutive invisible maximum
  // @param[IN] track_consecutive_invisible_maximum: track consecutive
  // invisible maximum
  // @return true if set successfully, otherwise return false
  static bool SetTrackConsecutiveInvisibleMaximum(
      const int track_consecutive_invisible_maximum);

  // @brief set track visible ratio minimum
  // @param[IN] track_visible_ratio_minimum: track visible ratio minimum
  // @return true if set successfully, otherwise return false
  static bool SetTrackVisibleRatioMinimum(
      const float track_visible_ratio_minimum);

  // @brief get maintained tracks
  // @return maintained tracks
  std::vector<ObjectTrackPtr>& GetTracks() { return tracks_; }

  // @brief get maintained tracks
  // @return maintained tracks
  const std::vector<ObjectTrackPtr>& GetTracks() const { return tracks_; }

  // @brief get size of maintained tracks
  // @return size of maintained tracks
  int Size() const { return tracks_.size(); }

  // @brief add track to current set of maintained tracks
  // @param[IN] track: adding track
  // @return nothing
  void AddTrack(ObjectTrackPtr track) { tracks_.push_back(track); }

  // @brief remove lost tracks
  // @return number of removed tracks
  int RemoveLostTracks();

  // @brief clear maintained tracks
  // @return nothing
  void Clear();

 public:
  static int s_track_consecutive_invisible_maximum_;
  static float s_track_visible_ratio_minimum_;

 private:
  std::vector<ObjectTrackPtr> tracks_;
};  // class ObjectTrackSet

}  // namespace perception
}  // namespace apollo

