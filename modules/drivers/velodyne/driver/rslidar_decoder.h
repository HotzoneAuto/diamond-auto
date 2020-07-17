/******************************************************************************
 * Copyright 2019 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
//#ifndef ROBOSENSE_RSLIDAR_DECODER_H
//#define ROBOSENSE_RSLIDAR_DECODER_H

#pragma once

#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "cyber/cyber.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "rslidar_packet.h"
namespace robosense {
namespace rslidar {
using namespace apollo::drivers;
/* packets from LiDAR decoded result */
enum RS_Decode_Result {
  RS_Decode_Fail = -2,   /* packet decode error accure */
  RS_Param_Invalid = -1, /* input packet buffer pointer invalid */
  RS_Decode_ok = 0,      /* packet decode finish without error */
  RS_Frame_Split = 1     /* packet decode ok and match frame split condiction*/
};
/* decoder support RS-LiDAR type */
enum RS_Lidar_Type {
  RS_Type_Lidar16 = 1,     /* RS-LiDar 16 */
  RS_Type_Lidar32,         /* RS-LiDar 32 */
  RS_Type_LidarBpearl,     /* RS-LiDar BPearl */
  RS_Type_LidarBpearl_Mini /* RS-LiDar BPearl mini */
};
/* resolution type of RS-LiDAR */
enum RS_Resolution_Type {
  RS_Resolution_5mm = 0, /* distance precision is 5 millimeter */
  RS_Resolution_10mm     /* distance precision is 10 millimeter */
};
/* echo mode of RS-LiDAR */
enum RS_Echo_Mode {
  RS_Echo_Dual = 0,  /* RS-Lidar received two echo */
  RS_Echo_Strongest, /* RS-Lidar received the strongest echo */
  RS_Echo_Last       /* RS-Lidar received the last echo */
};

/* paramaters for decoder */
typedef struct {
  RS_Lidar_Type lidar;
  RS_Resolution_Type resolution;
  RS_Echo_Mode echo;
  float cut_angle;
  float max_distance;
  float min_distance;
  float start_angle;
  float end_angle;
  std::string channel_filter;
} ST_Param;

/********************************************************************
 * @name class RSLidarDecoder
 * @brief RSLiDAR packets process, decode packets into pointcloud
 ********************************************************************/
class RSLidarDecoder {
 public:
  RSLidarDecoder(ST_Param& param);
  ~RSLidarDecoder();
  RS_Decode_Result processMsopPkt(const uint8_t* pkt,
                                  const std::shared_ptr<PointCloud> pc,
                                  double& timestamp);
  RS_Decode_Result processDifopPkt(const uint8_t* pkt);

 private:
  int decodeMsopPkt(const uint8_t* pkt, const std::shared_ptr<PointCloud> pc,
                    double& timestamp);
  float computeTemperatue(const uint16_t temp_raw);
  float distanceCalibration(const int32_t distance, const int32_t channel,
                            const float temp);
  int32_t azimuthCalibration(float azimuth, int32_t channel);

 private:
  RS_Lidar_Type lidar_type_;           /* RS-LiDAR type */
  RS_Resolution_Type resolution_type_; /* resolution type */
  RS_Echo_Mode echo_mode_;             /* echo mode */

  bool is_bpearl_;

  int32_t rpm_; /* revolutions per minute */

  float Rx_;
  float Ry_;
  float Rz_;

  float max_distance_threshold_; /* distance max threshold */
  float min_distance_threshold_; /* distance min threshold */

  uint16_t start_angle_; /* point angle filter's start angle */
  uint16_t end_angle_;   /* point angle filter's end angle */
  bool angle_flag_;
  uint16_t fov_start_;
  uint16_t fov_end_;
  uint16_t azimuth_diff_max_;

  int32_t pkts_per_frame_;
  int32_t packet_counter_; /* packet counter */

  int32_t cut_angle_;    /* cut angle for split frame */
  int32_t last_azimuth_; /* buffer the last azimuth */

  /* RS-LiDAR calibration data buffer */
  uint32_t calib_init_flag_;
  float vert_angle_calib_[RS32_CHANNEL_NUM];
  float horiz_angle_calib_[RS32_CHANNEL_NUM];
  float curve_rate_[RS32_CHANNEL_NUM];
  float intensity_cali_[7][RS32_CHANNEL_NUM];
  int32_t channel_cali_[RS32_CHANNEL_NUM][RS32_TEMPERATURE_CNT];

  bool channel_filter_[RS32_CHANNEL_NUM];

  /* cos/sin lookup table */
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
};

}  // namespace rslidar
}  // namespace robosense

//#endif //ROBOSENSE_RSLIDAR_RSLIDAR_DECODER_H
