
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

apollo::drivers::PointCloud lidar_frame;

// Convert online points to pcl pointcloud

pcl::PointCloud<pcl::PointXYZ>::Ptr online_points(
    new pcl::PointCloud<pcl::PointXYZ>());

for (unsigned int i = 0; i < lidar_frame.point.size(); ++i) {
  pcl::PointXYZ p(lidar_frame.x[i], lidar_frame.y[i], lidar_frame.z[i]);
  online_points->push_back(p);
}