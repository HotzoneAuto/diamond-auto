#include "modules/perception/lidar_point_pillars/point_pillars_detection.h"

namespace apollo {
namespace perception {
namespace lidar {
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
  seq_num_ = 0;
  object_builder_.reset(new apollo::perception::MinBoxObjectBuilder);
  tracker_.reset(new apollo::perception::HmObjectTracker());
  tracker_->Init();
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

  // if (enable_ground_removal) {
  //   z_min_ = std::max(z_min_, static_cast<float>(ground_removal_height));
  // }

//***********************************************************
  // //remove plane 
  // auto cloud_points = cloudRegion->points;
  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // pcl::PointIndices::Ptr inliers_seg(new pcl::PointIndices);

  // pcl::SACSegmentation<pcl::PointXYZI> seg;
  // seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // //  seg.setMaxIterations(maxIterations);
  // seg.setDistanceThreshold(distanceThreshold);
  // seg.setInputCloud(cloudRegion);
  // seg.segment(*inliers_seg, *coefficients);

  // // segment obstacles
  // pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud(
  //     new pcl::PointCloud<pcl::PointXYZI>());
  // pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(
  //     new pcl::PointCloud<pcl::PointXYZI>());

  // for (int index : inliers_seg->indices) {
  //   planeCloud->points.push_back(cloudRegion->points[index]);
  // }

  // pcl::ExtractIndices<pcl::PointXYZI> extract_obst;
  // extract_obst.setInputCloud(cloudRegion);
  // extract_obst.setIndices(inliers_seg);
  // extract_obst.setNegative(true);
  // extract_obst.filter(*obstCloud);
//***********************************************************

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloudRegion, "x");
  viewer->addPointCloud<pcl::PointXYZI>(cloudRegion, fildColor, "init cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "init cloud");
  int num_points = cloudRegion->points.size();
  num_points = std::min(num_points, max_num_points);
  float* points_array = new float[num_points * num_point_feature];
  // float points_array[num_points * num_point_feature];
  // timestamp_ = absl::ToUnixMicros(Clock::Now());
  // cout << "-------------timestamp---------------" << timestamp_ << endl;
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
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> points_vector;
  std::vector<std::shared_ptr<apollo::perception::Object>> objects;
  float intensi = 50.0;
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
    msg_box->set_yaw(yaw);

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i = 0; i < (cloudRegion->size()); i++) {
      pcl::PointXYZI point = cloudRegion->at(i);
      point.intensity = intensi;
      if(point.z >= point_z_min && point.z <= point_z_max && point.y >= point_y_min && point.y <= point_y_max && point.x >= point_x_min &&
          point.x <= point_x_max) {
        temp_cloud->push_back(point);
      }
    }
    intensi += 50;
    // points_vector.push_back(temp_cloud);
    // apollo::perception::Object out_obj;
    std::shared_ptr<apollo::perception::Object> out_obj(new apollo::perception::Object);
    out_obj->cloud = temp_cloud;
    // objects.push_back(std::make_shared<apollo::perception::Object>(out_obj));
    objects.push_back(out_obj);
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(temp_cloud, "x");
    // viewer->addPointCloud<pcl::PointXYZI>(temp_cloud, fildColor);
    string cube = "box" + std::to_string(j+500);
    string cubeFill = "boxFill" + std::to_string(j+500);
    // string detected_label = "label: " + label;
    
    // struct Point position_label_detect;
    // position_label_detect.x = 5.0;
    // position_label_detect.y = 0.0;
    // position_label_detect.z = 1.2;

    // viewer->addText3D(detected_label, position_label_detect, 1.0, 0.0, 1.0, 0.0, to_string(j+2000));
    viewer->addCube(point_x_min, point_x_max, point_y_min, point_y_max, point_z_min,
                    point_z_max, Color(0, 1, 0).r, Color(0, 1, 0).g,
                    Color(0, 1, 0).b, cube);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, Color(0, 1, 0).r,
        Color(0, 1, 0).g, Color(0, 1, 0).b, cube);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, cube);

    viewer->addCube(point_x_min, point_x_max, point_y_min, point_y_max, point_z_min,
                    point_z_max, Color(0, 1, 0).r, Color(0, 1, 0).g,
                    Color(0, 1, 0).b, cubeFill);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, Color(0, 1, 0).r,
        Color(0, 1, 0).g, Color(0, 1, 0).b, cubeFill);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, cubeFill);

    std::cout << "object id: " << j << ", x: " << x << ", y: " << y
              << ", z: " << z << ", dx: " << dx << ", dy: " << dy
              << ", dz: " << dz << ", yaw: " << yaw << ", label: " << label
              << std::endl;
  }

//min_box
  if(object_builder_ != nullptr) {
    apollo::perception::ObjectBuilderOptions object_builder_options;
    if(!object_builder_->Build(object_builder_options, &objects)) {
      return false;
    }
  }

//tracker
  std::shared_ptr<apollo::perception::SensorObjects> out_sensor_objects(new apollo::perception::SensorObjects);
  if (objects.size() > 0) {
    timestamp_ = absl::ToUnixMicros(apollo::common::time::Clock::Now());
    // timestamp_ +=1;
    ++seq_num_;
    std::shared_ptr<Eigen::Matrix4d> velodyne_trans(new Eigen::Matrix4d);
    *velodyne_trans << 1, 0, 0, 0, 
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    out_sensor_objects->timestamp = timestamp_;
    out_sensor_objects->sensor2world_pose = *velodyne_trans;
    out_sensor_objects->seq_num = seq_num_;
    if (tracker_ != nullptr) {
      apollo::perception::TrackerOptions tracker_options;
      tracker_options.velodyne_trans = velodyne_trans;
      // tracker_options.velodyne_trans = velodyne_trans;
      try{
        if(!tracker_->Track(objects, timestamp_, tracker_options, &(out_sensor_objects->objects))) {
          std::cout << "tracker running error!" << std::endl;
        }
      } catch(exception& e) {
        std::cout << e.what() << std::endl;
        return false;
      }
    }
    std::cout << "tracked size = " << out_sensor_objects->objects.size() << std::endl;

    for (int i = 0; i < out_sensor_objects->objects.size(); i++) {
      string cube = "box" + std::to_string(out_sensor_objects->objects[i]->track_id);
      string cubeFill = "boxFill" + std::to_string(out_sensor_objects->objects[i]->track_id);

      float px_min = out_sensor_objects->objects[i]->vertex2[0];
      float py_min = out_sensor_objects->objects[i]->vertex2[1];
      float pz_min = out_sensor_objects->objects[i]->min_height;
      float px_max = out_sensor_objects->objects[i]->vertex3[0];
      float py_max = out_sensor_objects->objects[i]->vertex3[1];
      float pz_max = out_sensor_objects->objects[i]->max_height;
      
      // cout << "-------------object speed: " << out_sensor_objects->objects[i]->track_id << "--------------" << speed << endl;
      // cout << "-------------center point: " << out_sensor_objects->objects[i]->track_id << ": " << out_sensor_objects->objects[i]->center[0] 
      //                                       << " " << out_sensor_objects->objects[i]->center[1] 
      //                                       << " " << out_sensor_objects->objects[i]->center[2] << endl;
      string label_id;
      switch(out_labels[i]) {
        case 0: label_id = "bus"; break;
        case 1: label_id = "car"; break;
        case 2: label_id = "unknown movable"; break;
        case 3: label_id = "truck"; break;
        case 4: label_id = "unknown unmovable"; break;
        case 5: label_id = "cyclist"; break;
        case 6: label_id = "motorcyclist"; break;
        case 7: label_id = "pedestrian"; break;
        case 8: label_id = "trafficcone"; break;
        default: label_id = "unknown"; break;
      }
      int track_id = out_sensor_objects->objects[i]->track_id;
      double speed = sqrt((out_sensor_objects->objects[i]->velocity[0]) * out_sensor_objects->objects[i]->velocity[0] + 
                          (out_sensor_objects->objects[i]->velocity[1]) * out_sensor_objects->objects[i]->velocity[1]);
      int age = static_cast<int>(out_sensor_objects->objects[i]->tracking_time);
      
      // pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      // for(auto p : out_sensor_objects->objects[i]->drops){
      //   // struct Point p1;
      //   // p1.x = p[0];
      //   // p1.y = p[1];
      //   // p1.z = p[2];
      //   pcl::PointXYZI point_temp;
      //   point_temp.x = p[0];
      //   point_temp.y = p[1];
      //   point_temp.z = p[2];
      //   point_temp.intensity = 0;
      //   line_cloud->push_back(point_temp);
      // }
      // viewer->addPointCloud<pcl::PointXYZI>(line_cloud, "init cloud line" + i);
      // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "init cloud line" + i);
      // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, Color(0, 0, 1).r,
      //                                           Color(0, 0, 1).g, Color(0, 0, 1).b, "init cloud line" + i);

      vector<pcl::PointXYZ> line_vec;
      int k = 0;
      for (auto p : out_sensor_objects->objects[i]->drops) {
        pcl::PointXYZ p1;
        p1.x = p[0];
        p1.y = p[1];
        p1.z = 0;
        line_vec.push_back(p1);
        if(line_vec.size() > 2) {
          viewer->addLine(line_vec[k-1], line_vec[k], 0.0, 0.0, 1.0, "line" + to_string(i) + " " + to_string(k));
        }
        k++;
      }
      line_vec.clear();
      float position_x = out_sensor_objects->objects[i]->anchor_point[0];
      float position_y = out_sensor_objects->objects[i]->anchor_point[1];
      float position_z = out_sensor_objects->objects[i]->max_height + 0.2;

      struct Point point_label;
      point_label.x = position_x;
      point_label.y = position_y;
      point_label.z = position_z;
      string label_information = "label: " + label_id + "\n" +
                                 "track_id: " + to_string(track_id) + "\n" + 
                                 "speed: " + to_string(speed) + "\n" + 
                                 "age: " + to_string(age);

      viewer->addText3D(label_information, point_label, 0.1, 1.0, 0.0, 0.0, to_string(i + 1000));

      viewer->addCube(px_min, px_max, py_min, py_max, pz_min,
                    pz_max, Color(1, 0, 0).r, Color(1, 0, 0).g,
                    Color(1, 0, 0).b, cube);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r,
          Color(1, 0, 0).g, Color(1, 0, 0).b, cube);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, cube);

      viewer->addCube(px_min, px_max, py_min, py_max, pz_min,
                    pz_max, Color(1, 0, 0).r, Color(1, 0, 0).g,
                    Color(1, 0, 0).b, cubeFill);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, Color(1, 0, 0).r,
          Color(1, 0, 0).g, Color(1, 0, 0).b, cubeFill);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, cubeFill);
    }
  }

  obst_writer->Write(msg_obstacles);
  viewer->spinOnce();	
  delete[] points_array;
  return true;
}

void PointPillarsDetection::CloudToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr, 
                                         float* out_points_array, 
                                         float normalizing_factor) {
  // timestamp_ = absl::ToUnixMicros(Clock::Now());
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
    // out_points_array[i * num_point_feature + 4] = static_cast<float>(timestamp_);
    // out_points_array[i * num_point_feature + 4] = 0;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
