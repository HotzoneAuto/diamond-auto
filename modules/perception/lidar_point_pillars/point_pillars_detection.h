#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <exception>
#include <chrono>
#include <thread>

#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/cloud_viewer.h"
#include "Eigen/Core"

#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/lidar_point_pillars/point_pillars.h"
#include "modules/perception/proto/obst_box.pb.h"
#include "modules/perception/proto/lidar_pointcloud_conf.pb.h"

using namespace std;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

namespace apollo {
namespace perception {
namespace lidar {

class PointPillarsDetection: public Component<apollo::drivers::PointCloud, apollo::drivers::PointCloud>{
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg1, 
            const std::shared_ptr<apollo::drivers::PointCloud>& msg2) override;

 private:

   pcl::PointCloud<pcl::PointXYZI>::Ptr receive_and_voxel(const std::shared_ptr<apollo::drivers::PointCloud>& msg);
   void CloudToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr, 
                     float* out_points_array, 
                     float normalizing_factor);
//    void GetObjects(std::vector<std::shared_ptr<base::Object>>* objects,
//                    const Eigen::Affine3d& pose, std::vector<float>* detections,
//                    std::vector<int>* labels);
   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

   std::unique_ptr<PointPillars> point_pillars_ptr_;
   std::vector<double> points_timestamp_;

   float x_min_;
   float x_max_;
   float y_min_;
   float y_max_;
   float z_min_;
   float z_max_;
   bool reproduce_result_mode = false;
   bool enable_ground_removal = false;
   float ground_removal_height = -1.5;
   float score_threshold = 0.5;
   float nms_overlap_threshold = 0.5;

   string pfe_torch_file = "/apollo/modules/perception/models/point_pillars/pfe.pt";
   string rpn_onnx_file = "/apollo/modules/perception/models/point_pillars/rpn.onnx";

   int num_point_feature = 5;
   int max_num_points = std::numeric_limits<int>::max();

   float normalizing_factor = 255.0;
   int num_output_box_feature = 7;
   int gpu_id = 0;
   bool enable_downsample_beams = false;
   int downsample_beams_factor = 4;
   bool enable_downsample_pointcloud = false;
   float downsample_voxel_size_x = 0.1;
   float downsample_voxel_size_y = 0.1;
   float downsample_voxel_size_z = 0.1;

   Eigen::Vector4f minpoint;
   Eigen::Vector4f maxpoint;
   std::shared_ptr<apollo::cyber::Writer<apollo::perception::Obstacles>> obst_writer;
   apollo::perception::LidarPointcloudConf lidar_pointcloud_conf_;
};  // class PointPillarsDetection
CYBER_REGISTER_COMPONENT(PointPillarsDetection)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
