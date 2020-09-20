/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
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
#include <boost/thread/thread.hpp>
#include <vector>
#include <unordered_set>
#include <string>
#include <chrono>
//#include <vtkProp3D.h>

using namespace std;

//pcl::visualization::CloudViewer viewer("point cloud viewer");
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer test"));

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

float filterRes = 0.4;
Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
int maxIterations = 40;
float distanceThreshold = 0.3;
float clusterTolerance = 0.5;
int minsize = 10;
int maxsize = 140;

void MessageCallback(
		const std::shared_ptr<apollo::drivers::PointCloud>& msg) {
  cout << "-----------------------" << msg->point_size() << endl;
//init viewer
//  viewer->initCameraParameters();
//  int v1(0);
//  int v2(0);
//  viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& cloud = *pcloud;
//  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = msg->point_size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = msg->point(i).x();
    cloud.points[i].y = msg->point(i).y();
    cloud.points[i].z = msg->point(i).z();
  }

//  save point_cloud to pcd file 
//  pcl::io::savePCDFileASCII("/apollo/write_pcd_test.pcd", cloud); 

// visualize point_cloud
//  viewer.showCloud(pcloud);

//voxel frid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(pcloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);
//  viewer.showCloud(pcloud);

//cropbox
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRegion(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> region(true);
  region.setMin(minpoint);
  region.setMax(maxpoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  vector<int> indices;
  pcl::CropBox<pcl::PointXYZ> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for(int point:indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

//segment plane
//  int num_points = cloudRegion->points.size();
//  auto cloud_points = cloudRegion->points;
//  Ransac<pcl::PointXYZ> RansacSeg(maxIterations, distanceThreshold, num_points);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_seg(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ>seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloudRegion);
  seg.segment(*inliers_seg, *coefficients);
//segment obstacles
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());
  for(int index : inliers_seg->indices) {
    planeCloud->points.push_back(cloudRegion->points[index]);
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract_obst;
  extract_obst.setInputCloud(cloudRegion);
  extract_obst.setIndices(inliers_seg);
  extract_obst.setNegative(true);
  extract.filter(*obstCloud);
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(obstCloud, planeCloud);

//cluster
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>clusters;
  typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(segResult.first);
  vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minsize);
  ec.setMaxClusterSize(maxsize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(segResult.first);
  ec.extract(clusterIndices);

  for(pcl::PointIndices getIndices: clusterIndices){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(int index:getIndices.indices) {
      cloudCluster->points.push_back(segResult.first->points[index]);
    } 
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;
    clusters.push_back(cloudCluster);
  }

//bounding box
  int clusterId = 0;
  vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_temp: clusters) {
//    viewer->addPointCloud<pcl::PointXYZ>(cluster_temp, "obstcloud" + std::to_string(clusterId)); 
    pcl::PointXYZ minpoint_temp, maxpoint_temp;
    pcl::getMinMax3D(*cluster_temp, minpoint_temp, maxpoint_temp);

    Box box;
    box.x_min = minpoint_temp.x;
    box.y_min = minpoint_temp.y;
    box.z_min = minpoint_temp.z;
    box.x_max = maxpoint_temp.x;
    box.y_max = maxpoint_temp.y;
    box.z_max = maxpoint_temp.z;
    cout << box.x_min << endl;
    ++clusterId;
  }
//  viewer->setBackgroundColor (0, 0, 0);  
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcloud_handler(pcloud, 250, 0, 0);
//  viewer->addPointCloud(pcloud, "window1", v1);
//  viewer->addPointCloud(pcloud, "cloud", v1);
//  viewer->addPointCloud(pcloud, pcloud_handler, "window2", v2);
//  viewer->spin();

//  viewer.showCloud(cloudRegion);
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  auto listener = listener_node->CreateReader<apollo::drivers::PointCloud>(
		  "/diamond/sensor/lidar/front/PointCloud2", MessageCallback);

  apollo::cyber::WaitForShutdown();
  return 0;
}
