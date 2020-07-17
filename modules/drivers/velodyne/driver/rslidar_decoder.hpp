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
  RSLidarDecoder(ST_Param &param);
  ~RSLidarDecoder();
  RS_Decode_Result processMsopPkt(const uint8_t *pkt,
                                  const std::shared_ptr<PointCloud> pc,
                                  double &timestamp);
  RS_Decode_Result processDifopPkt(const uint8_t *pkt);

 private:
  int decodeMsopPkt(const uint8_t *pkt, const std::shared_ptr<PointCloud> pc,
                    double &timestamp);
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

/****************************************************************
 * @name RSLidarDecoder
 * @brief constructor function, must input some paramaters
 * @param param
 ***************************************************************/
RSLidarDecoder::RSLidarDecoder(ST_Param &param) {
  // paramaters initialize
  this->rpm_ = RS_RPM_600;
  this->pkts_per_frame_ = 84;

  this->packet_counter_ = 0;
  this->last_azimuth_ = RS_DEFAULT_AZIMUTH;

  this->lidar_type_ = param.lidar;
  this->resolution_type_ = param.resolution;
  this->echo_mode_ = param.echo;

  if (this->lidar_type_ == RS_Type_LidarBpearl ||
      this->lidar_type_ == RS_Type_LidarBpearl_Mini) {
    this->is_bpearl_ = true;
  } else {
    this->is_bpearl_ = false;
  }

  if (param.cut_angle > RS_ANGLE_MAX) {
    this->cut_angle_ = 0;
  } else {
    this->cut_angle_ = param.cut_angle * 100;
  }
  // distance threshold
  if (param.max_distance > RS_DEFAULT_DISTANCE_MAX ||
      param.max_distance < RS_DEFAULT_DISTANCE_MIN) {
    this->max_distance_threshold_ = RS_DEFAULT_DISTANCE_MAX;
  } else {
    this->max_distance_threshold_ = param.max_distance;
  }
  if (param.min_distance > RS_DEFAULT_DISTANCE_MAX ||
      param.min_distance > param.max_distance) {
    this->min_distance_threshold_ = RS_DEFAULT_DISTANCE_MIN;
  } else {
    this->min_distance_threshold_ = param.min_distance;
  }

  // start and end angle
  if (param.start_angle > RS_ANGLE_MAX || param.start_angle < RS_ANGLE_MIN ||
      param.end_angle > RS_ANGLE_MAX || param.end_angle < RS_ANGLE_MIN) {
    param.start_angle = RS_ANGLE_MIN;
    param.end_angle = RS_ANGLE_MAX;
  }
  this->start_angle_ = param.start_angle * 100;
  this->end_angle_ = param.end_angle * 100;
  if (this->start_angle_ > this->end_angle_) {
    this->angle_flag_ = false;
  } else {
    this->angle_flag_ = true;
  }
  this->fov_start_ = RS_ANGLE_INT_MIN;
  this->fov_start_ = RS_ANGLE_INT_MAX;

  // packet rate
  int pkt_rate =
      ceil(RS_V0_POINTS_CHANNEL_PER_SECOND / RS_BLOCKS_CHANNEL_PER_PKT);
  if (this->lidar_type_ == RS_Type_Lidar16 &&
      (this->echo_mode_ == RS_Echo_Last ||
       this->echo_mode_ == RS_Echo_Strongest)) {
    pkt_rate = ceil(pkt_rate / 2);
  } else if (this->lidar_type_ == RS_Type_Lidar32 &&
             this->echo_mode_ == RS_Echo_Dual) {
    pkt_rate = pkt_rate * 2;
  }
  this->pkts_per_frame_ = ceil(pkt_rate * 60 / this->rpm_);

  // angle cali
  switch (this->lidar_type_) {
    case RS_Type_Lidar32:
      this->Rx_ = RS32_RX_VALUE;
      this->Ry_ = RS32_RY_VALUE;
      this->Rz_ = RS32_RZ_VALUE;
      this->azimuth_diff_max_ = RS32_AZIMUTH_DIFF_DEFAULT;
      break;
    case RS_Type_LidarBpearl:
      this->Rx_ = RSBPEARL_RX_VALUE;
      this->Ry_ = RSBPEARL_RY_VALUE;
      this->Rz_ = RSBPEARL_RZ_VALUE;
      this->azimuth_diff_max_ = RSBPEARL_AZIMUTH_DIFF_DEFAULT;
      break;
    case RS_Type_LidarBpearl_Mini:
      this->Rx_ = RSBPEARL_MINI_RX_VALUE;
      this->Ry_ = RSBPEARL_MINI_RY_VALUE;
      this->Rz_ = RSBPEARL_MINI_RZ_VALUE;
      this->azimuth_diff_max_ = RSBPEARL_AZIMUTH_DIFF_DEFAULT;
      break;
    case RS_Type_Lidar16:
    default:
      this->Rx_ = RS16_RX_VALUE;
      this->Ry_ = RS16_RY_VALUE;
      this->Rz_ = RS16_RZ_VALUE;
      this->azimuth_diff_max_ = RS16_AZIMUTH_DIFF_DEFAULT;
      break;
  }

  for (int i = 0; i < RS32_CHANNEL_NUM; i++) {
    this->channel_filter_[i] = false;
  }
  std::cout << "[RS_Decoder][param] channel filter: " << param.channel_filter
            << std::endl;
  std::string::size_type pos_1, pos_2;
  const std::string sep_str(",");
  pos_2 = param.channel_filter.find(sep_str);
  pos_1 = 0;
  while (std::string::npos != pos_2) {
    int idx = std::stoi(param.channel_filter.substr(pos_1, pos_2 - pos_1));
    if (idx >= 0 && idx < RS32_CHANNEL_NUM) {
      this->channel_filter_[idx] = true;
    }
    pos_1 = pos_2 + sep_str.size();
    pos_2 = param.channel_filter.find(sep_str, pos_1);
  }
  param.channel_filter.clear();

  // lookup table init
  this->cos_lookup_table_.resize(36000);
  this->sin_lookup_table_.resize(36000);
  for (unsigned int i = 0; i < 36000; i++) {
    double rad = RS_TO_RADS(i / 100.0f);

    this->cos_lookup_table_[i] = std::cos(rad);
    this->sin_lookup_table_[i] = std::sin(rad);
  }

  // calibration data buffer set to 0
  this->calib_init_flag_ = 0x00;
  memset(this->intensity_cali_, 0, sizeof(this->intensity_cali_));
  memset(this->vert_angle_calib_, 0, sizeof(this->vert_angle_calib_));
  memset(this->horiz_angle_calib_, 0, sizeof(this->horiz_angle_calib_));
  memset(this->channel_cali_, 0, sizeof(this->channel_cali_));
  memset(this->curve_rate_, 0, sizeof(this->curve_rate_));

  // print infomation
  AINFO << "[RS_decoder][param][INFO] init lidar type: "
        << ((this->lidar_type_ == RS_Type_Lidar16) ? "RS16" : "RS32")
        << ", npkts: " << this->pkts_per_frame_ << ", rpm: " << this->rpm_
        << ", max_distance: " << this->max_distance_threshold_
        << ", min_distance: " << this->min_distance_threshold_
        << ", star angle: " << this->start_angle_
        << ", end angle: " << this->end_angle_
        << ", cut angle: " << this->cut_angle_;  //
  switch (this->echo_mode_) {
    case RS_Echo_Dual:
      AINFO << ", echo mode: dual";
      break;
    case RS_Echo_Strongest:
      AINFO << ", echo mode: strongest";
      break;
    case RS_Echo_Last:
      AINFO << ", echo mode: last";
      break;
    default:
      AINFO << ", echo mode: unknow";
      break;
  }
}

/****************************************************************
 * @name ~RSLidarDecoder
 * @brief destructor, nothing to do now
 ****************************************************************/
RSLidarDecoder::~RSLidarDecoder() {
  this->cos_lookup_table_.clear();
  this->sin_lookup_table_.clear();
  std::cout << "[RS_Decoder][destructor][info] exit";
}

/****************************************************************
 * @name processDifopPkt
 * @brief  decoder interface for difop packet
 * @param pkt: difop packet data
 * @return RS_Decode_Result
 ***************************************************************/
RS_Decode_Result RSLidarDecoder::processDifopPkt(const uint8_t *pkt) {
  if (pkt == NULL) {
    return RS_Param_Invalid;
  }

  ST_RS16_DifopPkt *rs16_ptr = (ST_RS16_DifopPkt *)pkt;
  if (rs16_ptr->sync != RS_DIFOP_SYNC) {
    return RS_Decode_Fail;
  }

  int echo_mode = this->echo_mode_;
  ST_Version *p_ver = &(rs16_ptr->version);
  if ((p_ver->bottom_sn[0] == 0x08 && p_ver->bottom_sn[1] == 0x02 &&
       p_ver->bottom_sn[2] >= 0x09) ||
      (p_ver->bottom_sn[0] == 0x08 && p_ver->bottom_sn[1] > 0x02)) {
    if (rs16_ptr->echo_mode == 0x01 || rs16_ptr->echo_mode == 0x02) {
      this->echo_mode_ = RS_Echo_Mode(rs16_ptr->echo_mode);
    } else {
      this->echo_mode_ = RS_Echo_Dual;
    }
  } else {
    this->echo_mode_ = RS_Echo_Strongest;
  }

  if ((p_ver->main_sn[1] == 0x00 && p_ver->main_sn[2] == 0x00 &&
       p_ver->main_sn[3] == 0x00) ||
      (p_ver->main_sn[1] == 0xFF && p_ver->main_sn[2] == 0xFF &&
       p_ver->main_sn[3] == 0xFF) ||
      (p_ver->main_sn[1] == 0x55 && p_ver->main_sn[2] == 0xAA &&
       p_ver->main_sn[3] == 0x5A) ||
      (p_ver->main_sn[1] == 0xE9 && p_ver->main_sn[2] == 0x01 &&
       p_ver->main_sn[3] == 0x00)) {
    this->resolution_type_ = RS_Resolution_10mm;
  } else {
    this->resolution_type_ = RS_Resolution_5mm;
  }

  this->fov_start_ = RS_SWAP_SHORT(rs16_ptr->fov.start_angle);
  this->fov_end_ = RS_SWAP_SHORT(rs16_ptr->fov.end_angle);

  if (this->fov_end_ > RS_ANGLE_INT_MAX ||
      this->fov_start_ > RS_ANGLE_INT_MAX) {
    this->fov_start_ = RS_ANGLE_INT_MIN;
    this->fov_end_ = RS_ANGLE_INT_MAX;
  }
  if (this->fov_start_ == RS_ANGLE_INT_MIN &&
      this->fov_end_ == RS_ANGLE_INT_MIN) {
    this->fov_end_ = RS_ANGLE_INT_MAX;
  }

  //    std::cout<<"[DEBUG] difop lidar type: "<<this->lidar_type_<<", echo
  //    mode: "<<this->echo_mode_
  //             <<", npkts: "<<this->pkts_per_frame_<<", rpm: "<<this->rpm_<<",
  //             resolution type: "<<this->resolution_type_
  //             <<", intensity mode: "<<this->intensity_type_<< std::endl;
  // cali data
  if (!(this->calib_init_flag_ & 0x1) &&
      (this->lidar_type_ == RS_Type_Lidar16)) {
    bool curve_flag = true;
    ST_RS16_Intensity *p_intensity = &(rs16_ptr->intensity);
    if ((p_intensity->intensity_cali[0] == 0x00 ||
         p_intensity->intensity_cali[0] == 0xFF) &&
        (p_intensity->intensity_cali[1] == 0x00 ||
         p_intensity->intensity_cali[1] == 0xFF) &&
        (p_intensity->intensity_cali[2] == 0x00 ||
         p_intensity->intensity_cali[2] == 0xFF) &&
        (p_intensity->intensity_cali[3] == 0x00 ||
         p_intensity->intensity_cali[3] == 0xFF)) {
      curve_flag = false;
    }

    if (curve_flag) {
      bool check_flag = true;
      uint8_t checksum;
      for (int k = 0; k < RS16_CHANNEL_NUM; k++) {
        checksum = p_intensity->intensity_cali[15 * k] ^
                   p_intensity->intensity_cali[15 * k + 1];
        for (int n = 1; n < 7; n++) {
          checksum = checksum ^ (p_intensity->intensity_cali[k * 15 + n * 2]) ^
                     (p_intensity->intensity_cali[k * 15 + n * 2 + 1]);
        }
        if (checksum != p_intensity->intensity_cali[k * 15 + 14]) {
          check_flag = false;
          break;
        }
      }

      if (check_flag) {
        uint16_t *inten_p;
        for (int i = 0; i < RS16_CHANNEL_NUM; i++) {
          inten_p = (uint16_t *)(p_intensity->intensity_cali + i * 15);
          for (int k = 0; k < 7; k++) {
            this->intensity_cali_[k][i] = RS_SWAP_SHORT(*(inten_p + k)) * 0.001;
          }
        }

        this->calib_init_flag_ = this->calib_init_flag_ | 0x01;

        std::cout
            << "[RS_decoder][difop][INFO] curves data is wrote in difop packet!"
            << std::endl;
      }
    }
  }

  if (!(this->calib_init_flag_ & 0x2)) {
    bool angle_flag = true;
    const uint8_t *p_pitch_cali;

    if (this->lidar_type_ == RS_Type_Lidar16) {
      p_pitch_cali = rs16_ptr->pitch_cali;
    } else {
      p_pitch_cali = ((ST_RS32_DifopPkt *)pkt)->pitch_cali;
    }

    if (((this->lidar_type_ == RS_Type_Lidar16) &&
         (p_pitch_cali[0] == 0x00 || p_pitch_cali[0] == 0xFF) &&
         (p_pitch_cali[1] == 0x00 || p_pitch_cali[1] == 0xFF) &&
         (p_pitch_cali[2] == 0x00 || p_pitch_cali[2] == 0xFF) &&
         (p_pitch_cali[3] == 0x00 || p_pitch_cali[3] == 0xFF)) ||
        ((this->lidar_type_ == RS_Type_Lidar32 ||
          this->lidar_type_ == RS_Type_LidarBpearl ||
          this->lidar_type_ == RS_Type_LidarBpearl_Mini) &&
         (p_pitch_cali[0] == 0x00 || p_pitch_cali[0] == 0xFF) &&
         (p_pitch_cali[1] == 0x00 || p_pitch_cali[1] == 0xFF) &&
         (p_pitch_cali[2] == 0x00 || p_pitch_cali[2] == 0xFF))) {
      angle_flag = false;
    }

    if (angle_flag) {
      int lsb, mid, msb, neg = 1;
      if (this->lidar_type_ == RS_Type_Lidar16) {
        for (int i = 0; i < RS16_CHANNEL_NUM; i++) {
          if (i < 8) {
            neg = -1;
          } else {
            neg = 1;
          }
          lsb = p_pitch_cali[i * 3];
          mid = p_pitch_cali[i * 3 + 1];
          msb = p_pitch_cali[i * 3 + 2];

          this->vert_angle_calib_[i] =
              ((lsb * 256 * 256 + mid * 256 + msb) * neg) * 0.0001f;
          this->horiz_angle_calib_[i] = 0;
        }
      } else {
        const uint8_t *p_yaw_cali = ((ST_RS32_DifopPkt *)pkt)->yaw_cali;
        for (int i = 0; i < RS32_CHANNEL_NUM; i++) {
          lsb = p_pitch_cali[i * 3];
          if (lsb == 0) {
            neg = 1;
          } else if (lsb == 1) {
            neg = -1;
          }
          mid = p_pitch_cali[i * 3 + 1];
          msb = p_pitch_cali[i * 3 + 2];
          if (this->is_bpearl_) {
            this->vert_angle_calib_[i] = ((mid * 256 + msb) * neg) * 0.01f;
          } else {
            this->vert_angle_calib_[i] = ((mid * 256 + msb) * neg) * 0.001f;
          }

          lsb = p_yaw_cali[i * 3];
          if (lsb == 0) {
            neg = 1;
          } else if (lsb == 1) {
            neg = -1;
          }
          mid = p_yaw_cali[i * 3 + 1];
          msb = p_yaw_cali[i * 3 + 2];
          this->horiz_angle_calib_[i] =
              ((mid * 256 + msb) * neg) * 0.001f * 100;
        }
      }

      this->calib_init_flag_ = this->calib_init_flag_ | 0x2;

      std::cout
          << "[RS_decoder][difop][INFO] angle data is wrote in difop packet!"
          << std::endl;
    }
  }

  if (!(this->calib_init_flag_ & 0x4)) {
    uint16_t rpm = RS_SWAP_SHORT(rs16_ptr->rpm);
    if (rpm == RS_RPM_1200) {
      this->azimuth_diff_max_ *= 2;
    } else if (rpm == RS_RPM_300) {
      this->azimuth_diff_max_ /= 2;
    }
    this->calib_init_flag_ = this->calib_init_flag_ | 0x4;
  }

  return RS_Decode_ok;
}

/********************************************************************
 * @name decodeMsopPkt
 * @brief function for decode msop packet
 * @param pkt: msop packet data
 * @param point_out_vec: pointcloud vector store points
 * @param timestamp: save RSLiDAR's timestamp value
 * @return RS_Decode_Result error or azimuth
 *********************************************************************/
int RSLidarDecoder::decodeMsopPkt(
    const uint8_t *pkt, const std::shared_ptr<PointCloud> point_out_vec,
    double &timestamp) {
  // check input packet buffer
  if (pkt == NULL) {
    return RS_Param_Invalid;
  }
  // check packet sync header
  const ST_MsopPkt *mpkt_ptr = (ST_MsopPkt *)pkt;
  if (mpkt_ptr->header.sync != RS_MSOP_SYNC) {
    return RS_Decode_Fail;
  }

  {  // get timestamp
    std::tm stm;

    stm.tm_year = mpkt_ptr->header.timestamp.year + 100;
    stm.tm_mon = mpkt_ptr->header.timestamp.month - 1;
    stm.tm_mday = mpkt_ptr->header.timestamp.day;
    stm.tm_hour = mpkt_ptr->header.timestamp.hour;
    stm.tm_min = mpkt_ptr->header.timestamp.minute;
    stm.tm_sec = mpkt_ptr->header.timestamp.second;

    timestamp = std::mktime(&stm) +
                RS_SWAP_SHORT(mpkt_ptr->header.timestamp.ms) / 1000 +
                RS_SWAP_SHORT(mpkt_ptr->header.timestamp.us) / 1000000;
  }

  // get the temperature
  const float temperature = computeTemperatue(mpkt_ptr->header.temp_raw);
  // get block offset
  int block_offset;
  if (this->lidar_type_ == RS_Type_Lidar32 &&
      this->echo_mode_ == RS_Echo_Dual) {
    block_offset = 2;
  } else {
    block_offset = 1;
  }
  // distance coef
  float distance_coef;
  if (this->resolution_type_ == RS_Resolution_5mm) {
    distance_coef = RS_RESOLUTION_5MM_COEF;
  } else {
    distance_coef = RS_RESOLUTION_10MM_COEF;
  }
  // traversal all blocks
  for (int blk_idx = 0; blk_idx < RS_BLOCKS_PER_PKT; blk_idx++) {
    // block id check
    if (mpkt_ptr->blocks[blk_idx].id != RS_BLOCK_ID) {
      std::cout << "[RS_decoder][msop][ERROR] msop pkt block id no match. id: "
                << mpkt_ptr->blocks[blk_idx].id << ", index: " << blk_idx
                << std::endl;

      return RS_Decode_Fail;
    }
    // calculate azimuth diff between block
    const int azimuth_blk = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);

    int azi_prev;
    int azi_cur;
    if (blk_idx < (RS_BLOCKS_PER_PKT - block_offset)) {
      azi_prev =
          RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + block_offset].azimuth);
      azi_cur = azimuth_blk;
    } else {
      azi_prev = azimuth_blk;
      azi_cur = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - block_offset].azimuth);
    }

    uint32_t diff =
        (float)((RS_ANGLE_THRESHOLD + azi_prev - azi_cur) % RS_ANGLE_THRESHOLD);
    if (diff > this->azimuth_diff_max_) {
      diff = this->azimuth_diff_max_;
    }
    const float azimuth_diff = (float)(diff);

    float azimuth_channel;
    for (int channel_idx = 0; channel_idx < RS_CHANNELS_PER_BLOCK;
         channel_idx++) {  // traversal all point
      // calculate point's azimuth
      int azimuth_final;
      if (this->lidar_type_ == RS_Type_Lidar16) {
        if (this->echo_mode_ == RS_Echo_Dual) {
          azimuth_channel = azimuth_blk + azimuth_diff * RS_CHANNEL_TOFFSET *
                                              (channel_idx % RS16_CHANNEL_NUM) /
                                              RS16_BLOCK_TDURATION_DUAL;
        } else {
          azimuth_channel =
              azimuth_blk +
              azimuth_diff *
                  (RS_FIRING_TDURATION * (channel_idx / RS16_CHANNEL_NUM) +
                   RS_CHANNEL_TOFFSET * (channel_idx % RS16_CHANNEL_NUM)) /
                  RS16_BLOCK_TDURATION_SINGLE;
        }

        azimuth_final = ((int)round(azimuth_channel)) % RS_ANGLE_THRESHOLD;
      } else {
        azimuth_channel = azimuth_blk + (azimuth_diff * RS_CHANNEL_TOFFSET *
                                         (channel_idx % RS16_CHANNEL_NUM) /
                                         RS_FIRING_TDURATION);
        azimuth_final = azimuthCalibration(azimuth_channel, channel_idx);
      }

      const int distance = RS_SWAP_SHORT(
          mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance);
      const float intensity =
          mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
      const float distance_cali =
          distanceCalibration(distance, channel_idx, temperature) *
          distance_coef;

      int angle_horiz_ori;
      const int angle_horiz = (azimuth_final + 36000) % 36000;
      int angle_vert;
      if (this->lidar_type_ == RS_Type_Lidar16) {
        angle_horiz_ori = angle_horiz;
        // channel_idx = channel_idx%RS16_CHANNEL_NUM;
        angle_vert =
            ((int)(this->vert_angle_calib_[channel_idx % RS16_CHANNEL_NUM] *
                   100) +
             36000) %
            36000;
      } else {
        angle_horiz_ori = (int)(azimuth_channel + 36000) % 36000;
        angle_vert =
            ((int)(this->vert_angle_calib_[channel_idx] * 100) + 36000) % 36000;
      }

      // store point to pointcloud vector
      apollo::drivers::PointXYZIT *point = point_out_vec->add_point();

      if ((distance_cali <= this->max_distance_threshold_ &&
           distance_cali >= this->min_distance_threshold_) &&
          ((this->angle_flag_ && angle_horiz >= this->start_angle_ &&
            angle_horiz <= this->end_angle_) ||
           (!this->angle_flag_ &&
            ((angle_horiz >= this->start_angle_ && angle_horiz <= 36000) ||
             (angle_horiz >= 0 && angle_horiz <= this->end_angle_)))) &&
          this->channel_filter_
              [channel_idx])  //&& this->channel_filter_[channel_idx]
      {
        double x = distance_cali * this->cos_lookup_table_[angle_vert] *
                       this->cos_lookup_table_[angle_horiz] +
                   this->Rx_ * this->cos_lookup_table_[angle_horiz_ori];
        double y = -distance_cali * this->cos_lookup_table_[angle_vert] *
                       this->sin_lookup_table_[angle_horiz] -
                   this->Rx_ * this->sin_lookup_table_[angle_horiz_ori];
        double z =
            distance_cali * this->sin_lookup_table_[angle_vert] + this->Rz_;
        double intensity = intensity;
        point->set_x(x);
        point->set_y(y);
        point->set_z(z);
        point->set_intensity(intensity);
      } else {
        point->set_x(NAN);
        point->set_y(NAN);
        point->set_z(NAN);
        point->set_intensity(0);
      }
    }
  }
  // decode ok, return azimuth
  return RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
}

/****************************************************************
 * @name processMsopPkt
 * @brief msop packet decode interface
 * @param pkt: msop packet data
 * @param timestamp: save RSLiDAR's timestamp
 * @return RS_Decode_Result
 ****************************************************************/
RS_Decode_Result RSLidarDecoder::processMsopPkt(
    const uint8_t *pkt, const std::shared_ptr<PointCloud> pointcloud_vec,
    double &timestamp) {
  if (pkt == NULL) {
    return RS_Param_Invalid;
  }

  int azimuth = decodeMsopPkt(pkt, pointcloud_vec, timestamp);
  if (azimuth < 0) {
    return RS_Decode_Fail;
  }

  this->packet_counter_++;
  if (this->cut_angle_ >= 0) {
    if (azimuth < this->last_azimuth_) {
      this->last_azimuth_ -= RS_ANGLE_THRESHOLD;
    }
    if (this->last_azimuth_ != RS_DEFAULT_AZIMUTH &&
        this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_) {
      this->last_azimuth_ = azimuth;

      //          std::cout<<"[RS_decoder][msop][DEBUG] cut angle pkt num:
      //          "<<this->packet_counter_<<", size:
      //          "<<pointcloud_vec.size()<<std::endl;

      this->packet_counter_ = 0;
      return RS_Frame_Split;
    }
    this->last_azimuth_ = azimuth;
  } else {
    if (this->packet_counter_ >= this->pkts_per_frame_) {
      //          std::cout<<"[RS_decoder][msop][DEBUG] cut pkt num:
      //          "<<this->packet_counter_<<", size:
      //          "<<pointcloud_vec.size()<<std::endl;

      this->packet_counter_ = 0;
      return RS_Frame_Split;
    }
  }

  return RS_Decode_ok;
}

/*************************************************************
 * @name computeTemperatue
 * @brief calculate temperature
 * @param temp_raw: rawdata of temperature sensor
 * @return temperature
 ************************************************************/
float RSLidarDecoder::computeTemperatue(const uint16_t temp_raw) {
  uint8_t neg_flag = (temp_raw >> 8) & 0x80;
  float msb = (temp_raw >> 8) & 0x7F;
  float lsb = (temp_raw & 0xFF00) >> 3;
  float temp;
  if (neg_flag == 0x80) {
    temp = -1 * (msb * 32 + lsb) * 0.0625f;
  } else {
    temp = (msb * 32 + lsb) * 0.0625f;
  }

  return temp;
}

/********************************************************
 * @name distanceCalibration
 * @brief distance calibration function for point
 * @param distance: point's distance
 * @param channel: larser channnel index
 * @param temp: temperature
 * @return distance
 ********************************************************/
float RSLidarDecoder::distanceCalibration(const int distance, const int channel,
                                          const float temp) {
  int temp_index = (int)floor(temp + 0.5);
  if (temp_index < RS_TEMPERATURE_MIN) {
    temp_index = 0;
  } else if (temp_index > RS_TEMPERATURE_MAX) {
    temp_index = RS_TEMPERATURE_MAX - RS_TEMPERATURE_MIN;
  } else {
    temp_index = temp_index - RS_TEMPERATURE_MIN;
  }
  float dist_ret = 0.0f;
  if (distance > this->channel_cali_[channel][temp_index]) {
    dist_ret = (float)(distance - this->channel_cali_[channel][temp_index]);
  }

  return dist_ret;
}

/***********************************************************
 * @name azimuthCalibration
 * @brief azimuth calibration
 * @param azimuth: point's azimuth
 * @param channel: larser channel index
 * @return azimuth
 ***********************************************************/
int RSLidarDecoder::azimuthCalibration(float azimuth, const int channel) {
  int azi_ret;

  if (azimuth > 0.0 && azimuth < 3000.0) {
    azimuth = azimuth + this->horiz_angle_calib_[channel] + 36000.0f;
  } else {
    azimuth = azimuth + this->horiz_angle_calib_[channel];
  }
  azi_ret = (int)azimuth;
  azi_ret %= RS_ANGLE_INT_MAX;

  return azi_ret;
}

}  // namespace rslidar
}  // namespace robosense

//#endif //ROBOSENSE_RSLIDAR_RSLIDAR_DECODER_H
