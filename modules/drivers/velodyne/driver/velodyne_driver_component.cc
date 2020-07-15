/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"

#include "modules/common/util/message_util.h"
#include "modules/drivers/velodyne/driver/velodyne_driver_component.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool VelodyneDriverComponent::Init() {
  AINFO << "Velodyne driver component init";
  Config velodyne_config;
  if (!GetProtoConfig(&velodyne_config)) {
    return false;
  }
  AINFO << "Velodyne config: " << velodyne_config.DebugString();
  // start the driver
  writer_ = node_->CreateWriter<VelodyneScan>(velodyne_config.scan_channel());
  rs_lidar_=0;
  if (velodyne_config.model()==RS32 ||velodyne_config.model()==RS16) {
    RSwriter_ = node_->CreateWriter<PointCloud>(velodyne_config.convert_channel_name());
    pointcloud_ = std::make_shared<PointCloud>();
    pointcloud_->mutable_point()->Reserve(140000);
    rs_lidar_=1;
  }
    
  VelodyneDriver *driver = VelodyneDriverFactory::CreateDriver(velodyne_config);
  if (driver == nullptr) {
    return false;
  }
  dvr_.reset(driver);
  dvr_->Init();
  // spawn device poll thread
  runing_ = true;
  if (rs_lidar_==1)
  device_thread_ = std::shared_ptr<std::thread>(
      new std::thread(std::bind(&VelodyneDriverComponent::RSdevice_poll, this)));
  else
  {
    device_thread_ = std::shared_ptr<std::thread>(
      new std::thread(std::bind(&VelodyneDriverComponent::device_poll, this)));
  }
  
  device_thread_->detach();
  return true;
}

/** @brief Device poll thread main loop. */
void VelodyneDriverComponent::device_poll() {
  while (!apollo::cyber::IsShutdown()) {
    // poll device until end of file
    std::shared_ptr<VelodyneScan> scan = std::make_shared<VelodyneScan>();
    bool ret = dvr_->Poll(scan);
    if (ret) {
      common::util::FillHeader("velodyne", scan.get());
      writer_->Write(scan);
      AINFO<<"Publish a scan";
    } else {
      AWARN << "device poll failed";
    }
  }

  AERROR << "CompVelodyneDriver thread exit";
  runing_ = false;
}
void VelodyneDriverComponent::RSdevice_poll() {
  while (!apollo::cyber::IsShutdown()) {
    // poll device until end of file
    pointcloud_->Clear();
    bool ret = dvr_->RSPoll(pointcloud_);
    if (ret) {
      
      RSwriter_->Write(pointcloud_);
    } else {
      AWARN << "device poll failed";
    }
  }

  AERROR << "CompVelodyneDriver thread exit";
  runing_ = false;
}
}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
