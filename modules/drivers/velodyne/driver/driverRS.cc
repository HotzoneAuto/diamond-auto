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

#include <cmath>
#include <ctime>
#include <string>

#include "modules/drivers/velodyne/driver/driver.h"
#include "modules/drivers/velodyne/driver/rsinput.h"
namespace apollo {
namespace drivers {
namespace velodyne {
using namespace robosense::rslidar;
void RSDriver::Init() {
  rsinput_.reset(new rslidar_driver::InputSocket(config_));
  if (config_.model() == RS32)
    param_.lidar = RS_Type_Lidar32;
  else
    param_.lidar = RS_Type_Lidar16;
  param_.resolution = RS_Resolution_5mm;
  // param_.intensity = robosense::rslidar::RS_INTENSITY_EXTERN;
  param_.echo = RS_Echo_Strongest;
  param_.cut_angle = 0;
  param_.max_distance = config_.max_range();
  param_.min_distance = config_.min_range();
  param_.start_angle = config_.min_angle();
  param_.end_angle = config_.max_angle();
  decoder_ = std::make_shared<RSLidarDecoder>(param_);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool RSDriver::RSPoll(const std::shared_ptr<PointCloud>& pc) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  pc->set_is_dense(false);
  // TODO;
  if (config_.model() == RS32)
    pc->set_height(32);
  else
    pc->set_height(16);
  pc->mutable_header()->set_frame_id(config_.frame_id());
  pc->mutable_header()->set_sequence_num(0);  // TODO
  pc->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
  while (1) {
    int status = -5;
    rslidar_driver::rslidarPacket pkt;
    rslidar_driver::E_INPUT_STATE ret = this->rsinput_->getPacket(&pkt, 200);
    switch (ret) {
      case rslidar_driver::E_ERROR_INVALID_PARAM:
        AERROR << "[driver] invalid param";
        return false;
        break;
      case rslidar_driver::E_ERROR_SOCKET:
        AERROR << "[driver] socket read fail";
        return false;
        break;
      case rslidar_driver::E_ERROR_PKT_LEN:
        AERROR << "[driver] pkt length no match";
        return false;
        break;
      case rslidar_driver::E_PKT_MSOP:
        status = GeneratePointCloud(pkt, pc);
        break;
      case rslidar_driver::E_PKT_DIFOP:
        status = decoder_->processDifopPkt(pkt.data);
        AINFO << "GET DIFOP DECODER";
        break;
      case rslidar_driver::E_PCAP_EMPTY:
        break;
      case rslidar_driver::E_PCAP_REPEAT:
        AINFO << "[driver] pcap repeat";
        break;
      case rslidar_driver::E_OK:
      default:
        break;
    }
    if (status == 1) return true;
    if (status == -2) {
      AERROR << "packet decode error accure";
      return false;
    }
    if (status == -1) {
      AERROR << "input packet buffer pointer invalid";
      return false;
    }
  }

  return true;
}
int RSDriver::GeneratePointCloud(rslidar_driver::rslidarPacket pkt,
                                 const std::shared_ptr<PointCloud>& pc) {
  double timestamp;
  int ret = decoder_->processMsopPkt(pkt.data, pc, timestamp);
  int size = pc->point_size();
  if (size == 0) {
    // we discard this pointcloud if empty
    AERROR << "All points is NAN! Please check velodyne:" << config_.model();
  } else {
    pc->set_measurement_time(pc->mutable_header()->timestamp_sec());
    pc->set_width(pc->point_size() / pc->height());
    pc->mutable_header()->set_lidar_timestamp(
        pc->mutable_header()->timestamp_sec());
  }
  return ret;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
