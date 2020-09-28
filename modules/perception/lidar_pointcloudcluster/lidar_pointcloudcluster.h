#include <iostream>
#include <vector>
#include <unordered_set>
#include <string>
#include <chrono>

#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/proto/obst_box.pb.h"

using namespace std;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class lidar_pointcloudcluster : public Component<apollo::drivers::PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg) override;
 private:
//  pcl::visualization::CloudViewer viewer("point cloud viewer");
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new   pcl::visualization::PCLVisualizer("viewer test"));

  //init para

  struct Color
  {
 
  	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
  };

  struct Box
  {
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
  float clusterTolerance = 0.7;
  int minsize = 7;
  int maxsize = 400;
  std::shared_ptr<apollo::cyber::Writer<apollo::perception::Obstacles>> obst_writer;
};
CYBER_REGISTER_COMPONENT(lidar_pointcloudcluster)
