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
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>

using namespace std;

pcl::visualization::CloudViewer viewer("point cloud viewer");
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer test"));

//init para
float filterRes = 0.4;

void MessageCallback(
		const std::shared_ptr<apollo::drivers::PointCloud>& msg) {
  cout << "-----------------------" << msg->point_size() << endl;
//init viewer
//  viewer->initCameraParameters();
//  int v1(0);
//  int v2(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
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
  viewer.showCloud(pcloud);

//voxel frid filter
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::VoxelGrid<pcl::PointXYZ> vg;
//  vg.setInputCloud(pcloud);
//  vg.setLeafSize(filterRes, filterRes, filterRes);
//  vg.filter(*pcloud);
//  viewer.showCloud(pcloud);

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pcloud_handler(pcloud, 250, 0, 0);
//  viewer->addPointCloud(pcloud, "window1", v1);
//  viewer->addPointCloud(pcloud, "cloud", v1);
//  viewer->addPointCloud(pcloud, pcloud_handler, "window2", v2);
//  viewer->spinOnce();
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
