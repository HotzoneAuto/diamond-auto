#pragma once
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <pcl/surface/concave_hull.h>
#include "modules/perception/lidar_pointcloud_tracking/common/convex_hullxy.h"
#include "modules/perception/lidar_pointcloud_tracking/common/pcl_types.h"
#include "modules/perception/lidar_pointcloud_tracking/common/object.h"
#include "modules/perception/lidar_pointcloud_tracking/common/base_object_builder.h"

namespace apollo {
namespace perception {

class MinBoxObjectBuilder : public BaseObjectBuilder {
 public:
  MinBoxObjectBuilder() : BaseObjectBuilder() {}
  virtual ~MinBoxObjectBuilder() {}

  bool Init() override { return true; }

  bool Build(const ObjectBuilderOptions& options,
             std::vector<std::shared_ptr<Object>>* objects) override;
  std::string name() const override { return "MinBoxObjectBuilder"; }

 protected:
  void BuildObject(ObjectBuilderOptions options,
                   std::shared_ptr<Object> object);

  void ComputePolygon2dxy(std::shared_ptr<Object> obj);

  double ComputeAreaAlongOneEdge(std::shared_ptr<Object> obj,
                                 size_t first_in_point, Eigen::Vector3d* center,
                                 double* lenth, double* width,
                                 Eigen::Vector3d* dir);

  void ReconstructPolygon(const Eigen::Vector3d& ref_ct,
                          std::shared_ptr<Object> obj);

  void ComputeGeometricFeature(const Eigen::Vector3d& ref_ct,
                               std::shared_ptr<Object> obj);
  
  void TransformPointCloud(pcl_util::PointCloudPtr cloud,
                         const std::vector<int>& indices,
                         pcl_util::PointDCloud* trans_cloud);

};

}  // namespace perception
}  // namespace apollo

