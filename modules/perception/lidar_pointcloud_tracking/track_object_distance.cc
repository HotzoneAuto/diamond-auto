#include "track_object_distance.h"

#include <algorithm>
#include <vector>

#include "modules/perception/lidar_pointcloud_tracking/common/geometry_util.h"

namespace apollo {
namespace perception {

double TrackObjectDistance::s_location_distance_weight_ = 0.6;
double TrackObjectDistance::s_direction_distance_weight_ = 0.2;
double TrackObjectDistance::s_bbox_size_distance_weight_ = 0.1;
double TrackObjectDistance::s_point_num_distance_weight_ = 0.1;
double TrackObjectDistance::s_histogram_distance_weight_ = 0.5;

bool TrackObjectDistance::SetLocationDistanceWeight(
    const float location_distance_weight) {
  if (location_distance_weight >= 0) {
    s_location_distance_weight_ = location_distance_weight;
    std::cout << "location distance weight of TrackObjectDistance is "
          << s_location_distance_weight_;
    return true;
  }
  std::cout << "invalid location distance weight of TrackeObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetDirectionDistanceWeight(
    const float direction_distance_weight) {
  if (direction_distance_weight >= 0) {
    s_direction_distance_weight_ = direction_distance_weight;
    std::cout << "direction distance weight of TrackObjectDistance is "
          << s_direction_distance_weight_;
    return true;
  }
  std::cout << "invalid direction distance weight of TrackObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetBboxSizeDistanceWeight(
    const float bbox_size_distance_weight) {
  if (bbox_size_distance_weight >= 0) {
    s_bbox_size_distance_weight_ = bbox_size_distance_weight;
    std::cout << "bbox size distance weight of TrackObjectDistance is "
          << s_bbox_size_distance_weight_;
    return true;
  }
  std::cout << "invalid bbox size distance weight of TrackObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetPointNumDistanceWeight(
    const float point_num_distance_weight) {
  if (point_num_distance_weight >= 0) {
    s_point_num_distance_weight_ = point_num_distance_weight;
    std::cout << "point num distance weight of TrackObjectDistance is "
          << s_point_num_distance_weight_;
    return true;
  }
  std::cout << "invalid point num distance weight of TrackObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetHistogramDistanceWeight(
    const float histogram_distance_weight) {
  if (histogram_distance_weight >= 0) {
    s_histogram_distance_weight_ = histogram_distance_weight;
    std::cout << "histogram distance weight of TrackObjectDistance is "
          << s_histogram_distance_weight_;
    return true;
  }
  std::cout << "invalid histogram distance weight of TrackObjectDistance!";
  return false;
}

float TrackObjectDistance::ComputeDistance(
    ObjectTrackPtr track, const Eigen::VectorXf& track_predict,
    const std::shared_ptr<TrackedObject>& new_object) {
  // Compute distance for given track & object
  float location_distance =
      ComputeLocationDistance(track, track_predict, new_object);//重心位置坐标距离差异
  float direction_distance =
      ComputeDirectionDistance(track, track_predict, new_object);//物体方向差异
  float bbox_size_distance = ComputeBboxSizeDistance(track, new_object);//Bbox尺寸差异
  float point_num_distance = ComputePointNumDistance(track, new_object);//点云数量差异
  float histogram_distance = ComputeHistogramDistance(track, new_object);//直方图差异

  float result_distance = s_location_distance_weight_ * location_distance +
                          s_direction_distance_weight_ * direction_distance +
                          s_bbox_size_distance_weight_ * bbox_size_distance +
                          s_point_num_distance_weight_ * point_num_distance +
                          s_histogram_distance_weight_ * histogram_distance;//各个差异*权值
  return result_distance;
}

float TrackObjectDistance::ComputeLocationDistance(
    ObjectTrackPtr track, const Eigen::VectorXf& track_predict,
    const std::shared_ptr<TrackedObject>& new_object) {
  // Compute location distance for given track & object
  // range from 0 to positive infinity
  const std::shared_ptr<TrackedObject>& last_object = track->current_object_;
  Eigen::Vector2f measured_anchor_point = new_object->anchor_point.head(2);
  Eigen::Vector2f predicted_anchor_point = track_predict.head(2);
  Eigen::Vector2f measurement_predict_diff =
      measured_anchor_point - predicted_anchor_point;
  float location_distance = measurement_predict_diff.norm();

  Eigen::Vector2f track_motion_dir = last_object->velocity.head(2);
  float track_speed = track_motion_dir.norm();
  track_motion_dir /= track_speed;
  /* Assume location distance is generated from a normal distribution with
   * symmetric variance. Modify its variance when track speed greater than
   * a threshold. Penalize variance in the orthogonal direction of motion. */
  if (track_speed > 2) {
    Eigen::Vector2f track_motion_orthogonal_dir =
        Eigen::Vector2f(track_motion_dir(1), -track_motion_dir(0));
    float motion_dir_distance =
        track_motion_dir(0) * measurement_predict_diff(0) +
        track_motion_dir(1) * measurement_predict_diff(1);
    float motion_orthogonal_dir_distance =
        track_motion_orthogonal_dir(0) * measurement_predict_diff(0) +
        track_motion_orthogonal_dir(1) * measurement_predict_diff(1);
    location_distance = sqrt(motion_dir_distance * motion_dir_distance * 0.25 +
                             motion_orthogonal_dir_distance *
                                 motion_orthogonal_dir_distance * 4);
  }
  return location_distance;
}

float TrackObjectDistance::ComputeDirectionDistance(
    ObjectTrackPtr track, const Eigen::VectorXf& track_predict,
    const std::shared_ptr<TrackedObject>& new_object) {
  // Compute direction distance for given track & object
  // range from 0 to 2
  const std::shared_ptr<TrackedObject>& last_object = track->current_object_;
  Eigen::Vector3f old_anchor_point = last_object->anchor_point;
  Eigen::Vector3f new_anchor_point = new_object->anchor_point;
  Eigen::Vector3f anchor_point_shift = new_anchor_point - old_anchor_point;
  anchor_point_shift(2) = 0;
  Eigen::Vector3f predicted_track_motion = track_predict.head(6).tail(3);
  predicted_track_motion(2) = 0;

  double cos_theta = 0.994;  // average cos
  if (!anchor_point_shift.head(2).isZero() &&
      !predicted_track_motion.head(2).isZero()) {
    cos_theta = VectorCosTheta2dXy(predicted_track_motion, anchor_point_shift);
  }
  float direction_distance = -cos_theta + 1.0;
  return direction_distance;
}

float TrackObjectDistance::ComputeBboxSizeDistance(
    ObjectTrackPtr track, const std::shared_ptr<TrackedObject>& new_object) {
  // Compute bbox size distance for given track & object
  // range from 0 to 1
  const std::shared_ptr<TrackedObject>& last_object = track->current_object_;
  Eigen::Vector3f old_bbox_dir = last_object->direction;
  Eigen::Vector3f new_bbox_dir = new_object->direction;
  Eigen::Vector3f old_bbox_size = last_object->size;
  Eigen::Vector3f new_bbox_size = new_object->size;

  float size_distance = 0.0;
  double dot_val_00 = fabs(old_bbox_dir(0) * new_bbox_dir(0) +
                           old_bbox_dir(1) * new_bbox_dir(1));
  double dot_val_01 = fabs(old_bbox_dir(0) * new_bbox_dir(1) -
                           old_bbox_dir(1) * new_bbox_dir(0));
  bool bbox_dir_close = dot_val_00 > dot_val_01;

  if (bbox_dir_close) {
    float diff_1 = fabs(old_bbox_size(0) - new_bbox_size(0)) /
                   std::max(old_bbox_size(0), new_bbox_size(0));
    float diff_2 = fabs(old_bbox_size(1) - new_bbox_size(1)) /
                   std::max(old_bbox_size(1), new_bbox_size(1));
    size_distance = std::min(diff_1, diff_2);
  } else {
    float diff_1 = fabs(old_bbox_size(0) - new_bbox_size(1)) /
                   std::max(old_bbox_size(0), new_bbox_size(1));
    float diff_2 = fabs(old_bbox_size(1) - new_bbox_size(0)) /
                   std::max(old_bbox_size(1), new_bbox_size(0));
    size_distance = std::min(diff_1, diff_2);
  }
  return size_distance;
}

float TrackObjectDistance::ComputePointNumDistance(
    ObjectTrackPtr track, const std::shared_ptr<TrackedObject>& new_object) {
  // Compute point num distance for given track & object
  // range from 0 and 1
  const std::shared_ptr<TrackedObject>& last_object = track->current_object_;
  int old_point_number = last_object->object_ptr->cloud->size();
  int new_point_number = new_object->object_ptr->cloud->size();
  float point_num_distance = std::abs(old_point_number - new_point_number) *
                             1.0f /
                             std::max(old_point_number, new_point_number);
  return point_num_distance;
}

float TrackObjectDistance::ComputeHistogramDistance(
    ObjectTrackPtr track, const std::shared_ptr<TrackedObject>& new_object) {
  // Compute histogram distance for given track & object
  // range from 0 to 3
  const std::shared_ptr<TrackedObject>& last_object = track->current_object_;
  std::vector<float>& old_object_shape_features =
      last_object->object_ptr->shape_features;
  std::vector<float>& new_object_shape_features =
      new_object->object_ptr->shape_features;
  if (old_object_shape_features.size() != new_object_shape_features.size()) {
    std::cout << "sizes of compared features not matched. TrackObjectDistance";
    return FLT_MAX;
  }

  float histogram_distance = 0.0;
  for (size_t i = 0; i < old_object_shape_features.size(); ++i) {
    histogram_distance +=
        std::fabs(old_object_shape_features[i] - new_object_shape_features[i]);
  }
  return histogram_distance;
}

}  // namespace perception
}  // namespace apollo
