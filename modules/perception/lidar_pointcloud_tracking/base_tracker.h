#pragma once

// SAMPLE CODE:
//
// class MyTracker : public BaseTracker {
// public:
//     MyTracker() : BaseTracker() {}
//     virtual ~MyTracker() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool track(
//              const std::vector<Object>& objects,
//              double timestamp,
//              const TrackerOptions& options,
//              std::vector<std::shared_ptr<Object>>* tracked_objects) override
//              {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "MyTracker";
//      }
//
// };
//
// // Register plugin.
// REGISTER_TRACKER(MyTracker);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseTracker* tracker =
//          BaseTrackerRegisterer::get_instance_by_name("MyTracker");
// using tracker to do somethings.
// ////////////////////////////////////////////////////

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/lidar_pointcloud_tracking/common/pcl_types.h"
#include "modules/perception/lidar_pointcloud_tracking/common/object.h"

namespace apollo {
namespace perception {

struct TrackerOptions {
  TrackerOptions() = default;
  explicit TrackerOptions(Eigen::Matrix4d *pose) : velodyne_trans(pose) {}

  std::shared_ptr<Eigen::Matrix4d> velodyne_trans;
};

class BaseTracker {
 public:
  BaseTracker() {}
  virtual ~BaseTracker() {}

  virtual bool Init() = 0;

  // @brief: tracking objects.
  // @param [in]: current frame object list.
  // @param [in]: timestamp.
  // @param [in]: options.
  // @param [out]: current tracked objects.
  virtual bool Track(const std::vector<std::shared_ptr<Object>> &objects,
                     double timestamp, const TrackerOptions &options,
                     std::vector<std::shared_ptr<Object>> *tracked_objects) = 0;

  virtual std::string name() const = 0;

};

}  // namespace perception
}  // namespace apollo

