#include "modules/perception/lidar_pointcloudcluster/lidar_pointcloudcluster.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
    new pcl::visualization::PCLVisualizer("viewer test"));

bool Lidar_pointcloudcluster::Init() {
  AINFO << "Commontest component init";

  obst_writer = node_->CreateWriter<apollo::perception::Obstacles>(
      "/diamond/perception/Obstacles");

  minpoint = Eigen::Vector4f(-32, -15, -2, 1);
  maxpoint = Eigen::Vector4f(20, 15, 2, 1);
  return true;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> Lidar_pointcloudcluster::filter_and_segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud) {
//voxel grid filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(origin_cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

//cropbox
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(minpoint);
  region.setMax(maxpoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  vector<int> indices;
  pcl::CropBox<pcl::PointXYZI> roof(true);
  roof.setMin(Eigen::Vector4f(-12, -4.7, -1.0, 1));
  roof.setMax(Eigen::Vector4f(0.0, 4.7, 1.0, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int point : indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  // segment plane
  auto cloud_points = cloudRegion->points;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_seg(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI>seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
//  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloudRegion);
  seg.segment(*inliers_seg, *coefficients);
//segment obstacles
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>());

   for(int index : inliers_seg->indices) {
     planeCloud->points.push_back(cloudRegion->points[index]);
   }

  pcl::ExtractIndices<pcl::PointXYZI> extract_obst;
  extract_obst.setInputCloud(cloudRegion);
  extract_obst.setIndices(inliers_seg);
  extract_obst.setNegative(true);
  extract_obst.filter(*obstCloud);


  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult_fun(obstCloud, planeCloud);
  return segResult_fun;
}

bool Lidar_pointcloudcluster::Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg1, 
                                   const std::shared_ptr<apollo::drivers::PointCloud>& msg2) {
//init viewer
//  viewer->initCameraParameters();
  if ((msg1->point_size() == 0) || (msg2->point_size() == 0)) {
    return false;
  }

  viewer->removeAllPointClouds();
  viewer->removeAllShapes();

//front point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width = msg1->point_size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  for (size_t i = 0; i < msg1->point_size(); ++i) {
    cloud.points[i].x = msg1->point(i).x();
    cloud.points[i].y = msg1->point(i).y();
    cloud.points[i].z = msg1->point(i).z();
    cloud.points[i].intensity = msg1->point(i).intensity();
  }

  vector<int> nan_cloud_inliers;
  pcl::removeNaNFromPointCloud(*pcloud, *pcloud, nan_cloud_inliers);
  pcloud = cloud.makeShared();
//  save point_cloud to pcd file 
//  pcl::io::savePCDFileASCII("/apollo/write_pcd_test.pcd",*pcloud); 

//rear point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_rear(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> cloud_rear;
  cloud_rear.width = msg2->point_size();
  cloud_rear.height = 1;
  cloud_rear.is_dense = false;
  cloud_rear.points.resize(cloud_rear.width * cloud_rear.height);  
  for (size_t i = 0; i < msg2->point_size(); ++i) {
    cloud_rear.points[i].x = -(msg2->point(i).x()) - 12;
    cloud_rear.points[i].y = -msg2->point(i).y();
    cloud_rear.points[i].z = msg2->point(i).z();
    cloud_rear.points[i].intensity = msg2->point(i).intensity();
  }

  pcloud_rear = cloud_rear.makeShared();
  vector<int> nan_cloud_inliers_rear;
  pcl::removeNaNFromPointCloud(*pcloud_rear, *pcloud_rear, nan_cloud_inliers_rear);

//point cloud data fusion
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult_front = filter_and_segment(pcloud);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult_rear = filter_and_segment(pcloud_rear);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_obst(new pcl::PointCloud<pcl::PointXYZI>);

  *pcloud_obst = *(segResult_front.first) + *(segResult_rear.first);
  *pcloud_plane = *(segResult_front.second) + *(segResult_rear.second);

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(pcloud_obst, pcloud_plane);

  viewer->addPointCloud<pcl::PointXYZI>(segResult.second, "plane");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "plane");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, Color(0, 1, 0).r, Color(0, 1, 0).g, Color(0, 1, 0).b, "plane");

  viewer->addPointCloud<pcl::PointXYZI>(segResult.first, "obst");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "obst");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r, Color(1, 0, 0).g, Color(1, 0, 0).b, "obst");

//cluster
  vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>clusters;
  typename pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(segResult.first);
  vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minsize);
  ec.setMaxClusterSize(maxsize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(segResult.first);
  ec.extract(clusterIndices);

  for(pcl::PointIndices getIndices: clusterIndices){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZI>);
    for(int index:getIndices.indices) {
      cloudCluster->points.push_back(segResult.first->points[index]);
    }
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;
    clusters.push_back(cloudCluster);
  }

  // bounding box
  int clusterId = 0;

  auto msg_obstacles = std::make_shared<apollo::perception::Obstacles>();
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_temp: clusters) {
    viewer->addPointCloud<pcl::PointXYZI>(cluster_temp, "obstcloud" + std::to_string(clusterId)); 
    pcl::PointXYZI minpoint_temp, maxpoint_temp;
    pcl::getMinMax3D(*cluster_temp, minpoint_temp, maxpoint_temp);

    Box box;
    box.x_min = minpoint_temp.x;
    box.y_min = minpoint_temp.y;
    box.z_min = minpoint_temp.z;
    box.x_max = maxpoint_temp.x;
    box.y_max = maxpoint_temp.y;
    box.z_max = maxpoint_temp.z;

    auto* msg_box = msg_obstacles->add_obstacles();
    msg_box->set_box_id(clusterId);
    msg_box->set_x_min(box.x_min);
    msg_box->set_y_min(box.y_min);
    msg_box->set_z_min(box.z_min);
    msg_box->set_x_max(box.x_max);
    msg_box->set_y_max(box.y_max);
    msg_box->set_z_max(box.z_max);

    string cube = "box" + std::to_string(clusterId);
    string cubeFill = "boxFill" + std::to_string(clusterId);

    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, Color(1, 0, 0).r, Color(1, 0, 0).g, Color(1, 0, 0).b, cube);   
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r, Color(1, 0, 0).g, Color(1, 0, 0).b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, cube);

    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, Color(1, 0, 0).r, Color(1, 0, 0).g, Color(1, 0, 0).b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r, Color(1, 0, 0).g, Color(1, 0, 0).b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, cubeFill);

    ++clusterId;
  }
  msg_obstacles->set_box_num(clusterId);
  obst_writer->Write(msg_obstacles);  
  viewer->addPointCloud<pcl::PointXYZI>(pcloud_obst, "init cloud");
  viewer->spinOnce();	

  return true;
}
