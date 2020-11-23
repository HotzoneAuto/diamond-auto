#include "modules/perception/lidar_point_pillars/point_pillars_detection.h"

namespace apollo {
namespace perception {
namespace lidar {
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer test"));
// pcl::visualization::CloudViewer viewer("pcd viewer");
bool PointPillarsDetection::Init() {
  cout << "----------------------point pillars init-----------------------" << endl;
  if (!GetProtoConfig(&lidar_pointcloud_conf_)) {
    AERROR << "Unable to load lidar pointcloud conf file: " << ConfigFilePath();
    return false;
  }

  AINFO << "point pillars init";
  x_min_ = Params::kMinXRange;
  x_max_ = Params::kMaxXRange;
  y_min_ = Params::kMinYRange;
  y_max_ = Params::kMaxYRange;
  z_min_ = Params::kMinZRange;
  z_max_ = Params::kMaxZRange;
  obst_writer = node_->CreateWriter<apollo::perception::Obstacles>(
    lidar_pointcloud_conf_.obst_output_channel());
  point_pillars_ptr_.reset(new PointPillars(reproduce_result_mode, score_threshold, 
                                            nms_overlap_threshold, pfe_torch_file, 
                                            rpn_onnx_file));
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer test"));
  minpoint = Eigen::Vector4f(-32, -15, -2, 1);
  maxpoint = Eigen::Vector4f(20, 15, 2, 1);
  cout << "------------------point pillars init finish------------------------" << endl;
  return true;
}

bool PointPillarsDetection::Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg1, 
                                 const std::shared_ptr<apollo::drivers::PointCloud>& msg2) {
  cout << "-----------enter function----------" << endl;
  if ((msg1 == nullptr) || (msg2 == nullptr)) {
    cout << "input null msg1 or msg2" << endl;
    return false;
  }
  if((msg1->point_size() == 0) || (msg2->point_size() == 0)) {
    cout << "pointcloud data size is 0" << endl;
    return false;
  }

  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  if (cudaSetDevice(gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << gpu_id;
    return false;
  }

  //front point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_front(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> cloud_front;
  cloud_front.width = msg1->point_size();
  cloud_front.height = 1;
  cloud_front.is_dense = false;
  cloud_front.points.resize(cloud_front.width * cloud_front.height);
  for (int i = 0; i < msg1->point_size(); ++i) {
    cloud_front.points[i].x = msg1->point(i).x();
    cloud_front.points[i].y = msg1->point(i).y();
    cloud_front.points[i].z = msg1->point(i).z();
    cloud_front.points[i].intensity = msg1->point(i).intensity();
  }

  pcloud_front = cloud_front.makeShared();

  //rear pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_rear(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> cloud_rear;
  cloud_rear.width = msg2->point_size();
  cloud_rear.height = 1;
  cloud_rear.is_dense = false;
  cloud_rear.points.resize(cloud_rear.width * cloud_rear.height);
  for (int i = 0; i < msg2->point_size(); ++i) {
    cloud_rear.points[i].x = -(msg2->point(i).x()) - 12.0;
    cloud_rear.points[i].y = -(msg2->point(i).y());
    cloud_rear.points[i].z = msg2->point(i).z();
    cloud_rear.points[i].intensity = msg2->point(i).intensity();
  }

  pcloud_rear = cloud_rear.makeShared();

  // fuse pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZI>);
  *pcloud = *pcloud_front + *pcloud_rear;

  vector<int> nan_cloud_inliers;
  pcl::removeNaNFromPointCloud(*pcloud, *pcloud, nan_cloud_inliers);

  //voxel grid filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(pcloud);
  vg.setLeafSize(downsample_voxel_size_x, downsample_voxel_size_y, downsample_voxel_size_z);
  vg.filter(*cloudFiltered);

  //crop box
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(minpoint);
  region.setMax(maxpoint);
  region.setInputCloud(cloudFiltered);
  // region.setInputCloud(pcloud);
  region.filter(*cloudRegion);

  vector<int> indices;
  pcl::CropBox<pcl::PointXYZI> roof(true);
  roof.setMin(Eigen::Vector4f(-12, -1.9, -1.0, 1));
  roof.setMax(Eigen::Vector4f(0.0, 1.9, 1.0, 1));
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

  if (enable_ground_removal) {
    z_min_ = std::max(z_min_, static_cast<float>(ground_removal_height));
  }

  viewer->addPointCloud<pcl::PointXYZI>(cloudRegion, "init cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  int num_points = cloudRegion->points.size();
  num_points = std::min(num_points, max_num_points);
  float* points_array = new float[num_points * num_point_feature];
  // float points_array[num_points * num_point_feature];
  points_timestamp_.assign(cloudRegion->points.size(), 0.0);
  CloudToArray(cloudRegion, points_array, normalizing_factor);

  std::vector<float> out_detections;
  std::vector<int> out_labels;
  
  // for(int i = 0; i < (num_points * num_point_feature); i++) {
  //   cout << "-------------points_array[i]-------" << points_array[i] << endl;
  // }

  try {
    point_pillars_ptr_->DoInference(points_array, num_points, &out_detections, &out_labels);
  } catch(exception& e) {
    // cout << e.what() << endl;
    cout << "-----------------exception occured:core dumped----------------" << endl;
    delete[] points_array;
    viewer->spinOnce();
    return false;
  }
  
  int num_objects = out_detections.size() / num_output_box_feature;

  cout << "-----------------num objects : -------------" << num_objects << endl;

  auto msg_obstacles = std::make_shared<apollo::perception::Obstacles>();
  //**********************************************************************
  // label: 0 bus
  //        1 car 
  //        2 unknown movable
  //        3 truck
  //        4 unknown unmovable
  //        5 cyclist
  //        6 motorcyclist
  //        7 pedestrian
  //        8 trafficcone
  //        other unknown
  //**********************************************************************
  for (int j = 0; j < num_objects; ++j) {
    float x = out_detections.at(j * num_output_box_feature + 0);
    float y = out_detections.at(j * num_output_box_feature + 1);
    float z = out_detections.at(j * num_output_box_feature + 2);
    float dx = out_detections.at(j * num_output_box_feature + 4);
    float dy = out_detections.at(j * num_output_box_feature + 3);
    float dz = out_detections.at(j * num_output_box_feature + 5);
    float yaw = out_detections.at(j * num_output_box_feature + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
    yaw = -yaw;

    int label = out_labels.at(j);

    float point_x_min = x - dx / 2;
    float point_y_min = y - dy / 2;
    float point_z_min = z - dz / 2;
    float point_x_max = x + dx / 2;
    float point_y_max = y + dy / 2;
    float point_z_max = z + dz / 2;

    auto* msg_box = msg_obstacles->add_obstacles();
    msg_box->set_box_id(label);
    msg_box->set_x_min(point_x_min);
    msg_box->set_y_min(point_y_min);
    msg_box->set_z_min(point_z_min);
    msg_box->set_x_max(point_x_max);
    msg_box->set_y_max(point_y_max);
    msg_box->set_z_max(point_z_max);

    string cube = "box" + std::to_string(label);
    string cubeFill = "boxFill" + std::to_string(label);

    viewer->addCube(point_x_min, point_x_max, point_y_min, point_y_max, point_z_min,
                    point_z_max, Color(1, 0, 0).r, Color(1, 0, 0).g,
                    Color(1, 0, 0).b, cube);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r,
        Color(1, 0, 0).g, Color(1, 0, 0).b, cube);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, cube);

    viewer->addCube(point_x_min, point_x_max, point_y_min, point_y_max, point_z_min,
                    point_z_max, Color(1, 0, 0).r, Color(1, 0, 0).g,
                    Color(1, 0, 0).b, cubeFill);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r,
        Color(1, 0, 0).g, Color(1, 0, 0).b, cubeFill);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, cubeFill);

    std::cout << "object id: " << j << ", x: " << x << ", y: " << y
              << ", z: " << z << ", dx: " << dx << ", dy: " << dy
              << ", dz: " << dz << ", yaw: " << yaw << ", label: " << label
              << std::endl;
  }

  msg_obstacles->set_box_num(num_objects);
  obst_writer->Write(msg_obstacles);
  viewer->spinOnce();	
  delete[] points_array;
  return true;
}

void PointPillarsDetection::CloudToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr, 
                                         float* out_points_array, 
                                         float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); i++) {
    pcl::PointXYZI point = pc_ptr->at(i);
    // if(point.z < z_min_ || point.z > z_max_ || point.y < y_min_ || point.y > y_max_ || point.x < x_min_ ||
    //     point.x > x_max_) {
    //   continue;
    // }
    out_points_array[i * num_point_feature + 0] = point.x;
    out_points_array[i * num_point_feature + 1] = point.y;
    out_points_array[i * num_point_feature + 2] = point.z;
    // out_points_array[i * num_point_feature + 3] = float(point.intensity) / float(normalizing_factor);
    out_points_array[i * num_point_feature + 3] = static_cast<float>(point.intensity / normalizing_factor);
    out_points_array[i * num_point_feature + 4] = static_cast<float>(points_timestamp_[i]);
    // out_points_array[i * num_point_feature + 4] = 0;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
