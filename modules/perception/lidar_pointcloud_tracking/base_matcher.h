#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "modules/perception/lidar_pointcloud_tracking/object_track.h"
#include "modules/perception/lidar_pointcloud_tracking/tracked_object.h"

namespace apollo {
namespace perception {

class BaseMatcher {
 public:
  BaseMatcher() {}
  virtual ~BaseMatcher() {}

  // @brief match detected objects to maintained tracks
  // @param[IN] objects: detected objects
  // @param[IN] tracks: maintained tracks
  // @param[IN] tracks_predict: predicted states of maintained tracks
  // @param[OUT] assignments: matched pair of <track, object>
  // @param[OUT] unassigned_tracks: unmatched tracks
  // @param[OUT] unassigned_objects: unmatched objects
  // @return nothing
  virtual void Match(std::vector<std::shared_ptr<TrackedObject>>* objects,
                     const std::vector<ObjectTrackPtr>& tracks,
                     const std::vector<Eigen::VectorXf>& tracks_predict,
                     std::vector<std::pair<int, int>>* assignments,
                     std::vector<int>* unassigned_tracks,
                     std::vector<int>* unassigned_objects) = 0;

  // @brief get name of matcher
  // @return name of matcher
  virtual std::string Name() const = 0;

 private:
  //DISALLOW_COPY_AND_ASSIGN(BaseMatcher);
};  // class BaseObjectTrackMatcher

}  // namespace perception
}  // namespace apollo

