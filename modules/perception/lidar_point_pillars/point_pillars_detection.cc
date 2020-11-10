#include "modules/perception/lidar_point_pillars/point_pillars_detection.h"

namespace apollo {
namespace perception {
namespace lidar {
bool PointPillarsDetection::Init() {
  cout << "----------------------point pillars init-----------------------" << endl;
  AINFO << "point pillars init";
  x_min_ = Params::kMinXRange;
  x_max_ = Params::kMaxXRange;
  y_min_ = Params::kMinYRange;
  y_max_ = Params::kMaxYRange;
  z_min_ = Params::kMinZRange;
  z_max_ = Params::kMaxZRange;
  point_pillars_ptr_.reset(new PointPillars(reproduce_result_mode, score_threshold, nms_overlap_threshold, pfe_torch_file, rpn_onnx_file));
  minpoint = Eigen::Vector4f(-32, -15, -2, 1);
  maxpoint = Eigen::Vector4f(20, 15, 2, 1);
  cout << "------------------point pillars init finish------------------------" << endl;
  return true;
}

bool PointPillarsDetection::Proc(const std::shared_ptr<apollo::drivers::PointCloud>& msg) {
  cout << "-----------enter function----------" << endl;
  if (msg == nullptr) {
    cout << "input null msg" << endl;
    return false;
  }
  if(msg->point_size() == 0) {
    cout << "pointcloud data size is 0" << endl;
    return false;
  }
  // if (cudaSetDevice(gpu_id) != cudaSuccess) {
  //   AERROR << "Failed to set device to gpu " << gpu_id;
  //   return false;
  // }

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width = msg->point_size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  for (int i = 0; i < msg->point_size(); ++i) {
    cloud.points[i].x = msg->point(i).x();
    cloud.points[i].y = msg->point(i).y();
    cloud.points[i].z = msg->point(i).z();
    cloud.points[i].intensity = msg->point(i).intensity();
  }

  vector<int> nan_cloud_inliers;
  pcl::removeNaNFromPointCloud(*pcloud, *pcloud, nan_cloud_inliers);
  pcloud = cloud.makeShared();
//   //voxel grid filter
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(
//       new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::VoxelGrid<pcl::PointXYZI> vg;
//   vg.setInputCloud(pcloud);
//   vg.setLeafSize(downsample_voxel_size_x, downsample_voxel_size_y, downsample_voxel_size_z);
//   vg.filter(*cloudFiltered);

//   //crop box
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion(
//       new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::CropBox<pcl::PointXYZI> region(true);
//   region.setMin(minpoint);
//   region.setMax(maxpoint);
//   region.setInputCloud(cloudFiltered);
//   // region.setInputCloud(pcloud);
//   region.filter(*cloudRegion);
// cout << "--------------cloud region points1------" << cloudRegion->points.size() << endl;
//   vector<int> indices;
//   pcl::CropBox<pcl::PointXYZI> roof(true);
//   roof.setMin(Eigen::Vector4f(-12, -1.9, -1.0, 1));
//   roof.setMax(Eigen::Vector4f(0.0, 1.9, 1.0, 1));
//   roof.setInputCloud(cloudRegion);
//   roof.filter(indices);
// cout << "--------------cloud region points2------" << cloudRegion->points.size() << endl;
//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//   for (int point : indices) {
//     inliers->indices.push_back(point);
//   }

//   pcl::ExtractIndices<pcl::PointXYZI> extract;
//   extract.setInputCloud(cloudRegion);
//   extract.setIndices(inliers);
//   extract.setNegative(true);
//   extract.filter(*cloudRegion);
//   cout << "--------------pcloud num points ------" << pcloud->points.size() << endl;
//   cout << "--------------cloud region points------" << cloudRegion->points.size() << endl;
  if (enable_ground_removal) {
    z_min_ = std::max(z_min_, static_cast<float>(ground_removal_height));
  }

  int num_points = pcloud->points.size();
  // num_points = std::min(num_points, max_num_points);
  cout << "--------------num points------------------" << num_points << endl;
  float* points_array = new float[num_points * num_point_feature];
  points_timestamp_.assign(pcloud->points.size(), 0.0);
  CloudToArray(pcloud, points_array, normalizing_factor);

  std::vector<float> out_detections;
  std::vector<int> out_labels;
  
  try {
    point_pillars_ptr_->DoInference(points_array, num_points, &out_detections, &out_labels);
  } catch(exception& e) {
    cout << e.what() << endl;
    cout << "-----------------exception occured----------------" << endl;
    return false;
  }
  
  int num_objects = out_detections.size() / num_output_box_feature;

  cout << "================================" << num_objects << endl;

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
    std::cout << "object id: " << j << ", x: " << x << ", y: " << y
              << ", z: " << z << ", dx: " << dx << ", dy: " << dy
              << ", dz: " << dz << ", yaw: " << yaw << ", label: " << label
              << std::endl;
  }
  return true;
}

void PointPillarsDetection::CloudToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr, 
                                         float* out_points_array, 
                                         float normalizing_factor) {
  cout << "----------------point cloud size: " << pc_ptr->size() << endl;
  for (size_t i = 0; i < pc_ptr->size(); i++) {
    pcl::PointXYZI point = pc_ptr->at(i);
    if(point.z < z_min_ || point.z > z_max_ || point.y < y_min_ || point.y > y_max_ || point.x < x_min_ ||
        point.x > x_max_) {
      continue;
    }
    out_points_array[i * num_point_feature + 0] = point.x;
    out_points_array[i * num_point_feature + 1] = point.y;
    out_points_array[i * num_point_feature + 2] = point.z;
    // out_points_array[i * num_point_feature + 3] = float(point.intensity) / float(normalizing_factor);
    out_points_array[i * num_point_feature + 3] = static_cast<float>(point.intensity / normalizing_factor);
    // out_points_array[i * num_point_feature + 4] = static_cast<float>(points_timestamp_[i]);
    out_points_array[i * num_point_feature + 4] = 0;
  }
}

// void GetObjects(std::vector<std::shared_ptr<base::Object>>* objects,
//                 const Eigen::Affine3d& pose, std::vector<float>* detections,
//                 std::vector<int>* labels) {
//   int num_objects = detections->size() / num_output_box_feature;
//   objects->clear();

//   for(int i = 0; i < num_objects; ++i) {
//     objects->emplace_back(std::shared_ptr<base::Object>(new base::Object()));
//   }

//   for(int i = 0; i < num_objects; ++i) {

//   }
// }
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
