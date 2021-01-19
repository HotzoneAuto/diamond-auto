#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/lidar_pointcloud_tracking/common/tracker_config.h"
#include "modules/perception/lidar_pointcloud_tracking/common/object.h"
#include "modules/perception/lidar_pointcloud_tracking/base_tracker.h"
#include "modules/perception/lidar_pointcloud_tracking/base_matcher.h"
#include "modules/perception/lidar_pointcloud_tracking/object_track.h"
#include "modules/perception/lidar_pointcloud_tracking/tracked_object.h"

namespace apollo {
namespace perception {

class HmObjectTracker : public BaseTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HmObjectTracker();
  virtual ~HmObjectTracker() = default;

  // @brief initialize tracker's configs
  bool Init();

  // @brief track detected objects over consecutive frames
  // @param[IN] objects: recently detected objects
  // @param[IN] timestamp: timestamp of recently detected objects
  // @param[IN] options: tracker options with necessary information
  // @param[OUT] tracked_objects: tracked objects with tracking information
  // @return true if track successfully, otherwise return false
  bool Track(const std::vector<std::shared_ptr<Object>>& objects,
             double timestamp, const TrackerOptions& options,
             std::vector<std::shared_ptr<Object>>* tracked_objects);

  // @brief get object tracks of tracker
  // @return object tracks maintained in tracker
  const std::vector<ObjectTrackPtr>& GetObjectTracks() const;

  std::string name() const { return "HmObjectTracker"; }

 protected:
  // @brief initialize tracker after obtaining detection of first frame
  // @param[IN] objects: recently detected objects
  // @param[IN] timestamp: timestamp of recently detected objects
  // @param[IN] options: tracker options with necessary information
  // @param[OUT] tracked_objects: tracked objects with tracking information
  // @return true if initialize successfully, otherwise return false
  bool InitializeTrack(const std::vector<std::shared_ptr<Object>>& objects,
                       const double timestamp, const TrackerOptions& options,
                       std::vector<std::shared_ptr<Object>>* tracked_objects);

  // @brief transform v2world pose to v2local pose intend to avoid huge value
  // float computing
  // @param[OUT] pose: v2world pose
  // @return nothing
  void TransformPoseGlobal2Local(Eigen::Matrix4d* pose);

  // @brief construct tracked objects via necessary transformation & feature
  // computing
  // @param[IN] objects: objects for construction
  // @param[OUT] tracked_objects: constructed objects
  // @param[IN] pose: pose using for coordinate transformation
  // @param[IN] options: tracker options with necessary information
  // @return nothing
  void ConstructTrackedObjects(
      const std::vector<std::shared_ptr<Object>>& objects,
      std::vector<std::shared_ptr<TrackedObject>>* tracked_objects,
      const Eigen::Matrix4d& pose, const TrackerOptions& options);

  // @brief compute objects' shape feature
  // @param[OUT] object: object for computing shape feature
  // @return nothing
  void ComputeShapeFeatures(std::shared_ptr<TrackedObject>* obj);

  // @brief transform tracked object with given pose
  // @param[OUT] obj: tracked object for transformation
  // @param[IN] pose: pose using for coordinate transformation
  // @return nothing
  void TransformTrackedObject(std::shared_ptr<TrackedObject>* obj,
                              const Eigen::Matrix4d& pose);

  // @brief transform object with given pose
  // @param[OUT] obj: object for transformation
  // @param[IN] pose: pose using for coordinate transformation
  // @return nothing
  void TransformObject(std::shared_ptr<Object>* obj,
                       const Eigen::Matrix4d& pose);

  // @brief compute predicted states of maintained tracks
  // @param[OUT] tracks_predict: predicted states of maintained tracks
  // @param[IN] time_diff: time interval for predicting
  // @return nothing
  void ComputeTracksPredict(std::vector<Eigen::VectorXf>* tracks_predict,
                            const double time_diff);

  // @brief update assigned tracks
  // @param[IN] tracks_predict: predicted states of maintained tracks
  // @param[IN] new_objects: recently detected objects
  // @param[IN] assignments: assignment pair of <track, object>
  // @param[IN] time_diff: time interval for updating
  // @return nothing
  void UpdateAssignedTracks(
      std::vector<Eigen::VectorXf>* tracks_predict,
      std::vector<std::shared_ptr<TrackedObject>>* new_objects,
      const std::vector<std::pair<int, int>>& assignments,
      const double time_diff);

  // @brief update tracks without matched objects
  // @param[IN] tracks_predict: predicted states of maintained tracks
  // @param[IN] unassigned_tracks: index of unassigned tracks
  // @param[IN] time_diff: time interval for updating
  // @return nothing
  void UpdateUnassignedTracks(
      const std::vector<Eigen::VectorXf>& tracks_predict,
      const std::vector<int>& unassigned_tracks, const double time_diff);

  // @brief create new tracks for objects without matched track
  // @param[IN] new_objects: recently detected objects
  // @param[IN] unassigned_objects: index of unassigned objects
  // @return nothing
  void CreateNewTracks(
      const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
      const std::vector<int>& unassigned_objects);

  // @brief delete lost tracks
  // @return nothing
  void DeleteLostTracks();

  // @brief collect tracked results
  // @param[OUT] tracked_objects: tracked objects with tracking information
  // @return nothing
  void CollectTrackedResults(
      std::vector<std::shared_ptr<Object>>* tracked_objects);

 private:
  // algorithm setup
  bool use_histogram_for_match_ = false;

  // matcher
  std::unique_ptr<BaseMatcher> matcher_;

  // tracks
  ObjectTrackSet object_tracks_;

  // set offset to avoid huge value float computing
  Eigen::Vector3d global_to_local_offset_;
  double time_stamp_ = 0.0;
  bool valid_ = false;

  tracker_config::ModelConfigs config_;

  std::vector<std::vector<Eigen::Vector3d>> drops_;
  std::vector<int> index_tracked_obj_;
  bool first_time_;

};  // class HmObjectTracker


}  // namespace perception
}  // namespace apollo
