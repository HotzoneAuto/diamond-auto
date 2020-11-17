#pragma once
#include <iostream>
#include <string>

#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/proto/obst_box.pb.h"
#include "modules/perception/proto/lidar_pointcloud_conf.pb.h"

using namespace std;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class Lidar_pointcloudcluster : public Component<apollo::drivers::PointCloud,
<<<<<<< HEAD
                                                apollo::drivers::PointCloud> {
 public:
  bool Init() override;
  
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
  filter_and_segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud);

  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg1, 
=======
                                                 apollo::drivers::PointCloud> {
 public:
  bool Init() override;
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
  filter_and_segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud);
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg1,
>>>>>>> master
            const std::shared_ptr<apollo::drivers::PointCloud>& msg2) override;

 private:
  // init para

  struct Color {
    float r, g, b;

    Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
  };

  struct Box {
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
  };

  float filterRes = 0.1;
  Eigen::Vector4f minpoint;
  Eigen::Vector4f maxpoint;
  int maxIterations = 40;
  float distanceThreshold = 0.2;
  float clusterTolerance = 0.8;
  int minsize = 7;
  int maxsize = 800;
<<<<<<< HEAD
  std::shared_ptr<apollo::cyber::Writer<apollo::perception::Obstacles>> obst_writer;
  apollo::perception::LidarPointcloudConf lidar_pointcloud_conf_;
=======
  std::shared_ptr<apollo::cyber::Writer<apollo::perception::Obstacles>>
      obst_writer;
>>>>>>> master
};

CYBER_REGISTER_COMPONENT(Lidar_pointcloudcluster)
