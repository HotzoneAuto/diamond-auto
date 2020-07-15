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
#ifndef ROBOSENSE_RSLIDAR_PACKET_H
#define ROBOSENSE_RSLIDAR_PACKET_H

#pragma once

#include <cstdint>

namespace robosense
{
namespace rslidar
{
#define RS16_CHANNEL_NUM                (16)
#define RS32_CHANNEL_NUM                (32)


#define RS_V0_POINTS_CHANNEL_PER_SECOND (20000)
#define RS_V1_POINTS_CHANNEL_PER_SECOND (18000)
#define RS_BLOCKS_CHANNEL_PER_PKT       (12)

#define RS_CHANNELS_PER_BLOCK           (32)
#define RS_BLOCKS_PER_PKT               (12)

#define RS16_TEMPERATURE_CNT            (41)
#define RS32_TEMPERATURE_CNT            (51)

#define RS_LENGTH_PACKET                (1248)
#define RS_FIRST_BYTE_PKT               (0x55)

#define RS_SWAP_SHORT(x)                ((((x)&0xFF)<<8)|(((x)&0xFF00)>>8))
#define RS_SWAP_LONG(x)                 ((((x)&0xFF)<<24)|(((x)&0xFF00)<<8)|(((x)&0xFF0000)>>8)|(((x)&0xFF000000)>>24))
#define RS_TO_RADS(x)                   ((x)*(M_PI)/180)


#define RS_MSOP_SYNC                    (0xA050A55A0A05AA55)
#define RS_BLOCK_ID                     (0xEEFF)
#define RS_DIFOP_SYNC                   (0x555511115A00FFA5)

#define RS_CHANNEL_TOFFSET              (3)
#define RS_FIRING_TDURATION             (50)
#define RS16_BLOCK_TDURATION_DUAL       (50)
#define RS16_BLOCK_TDURATION_SINGLE     (100)

#define RS_RESOLUTION_5MM_COEF          (0.005)
#define RS_RESOLUTION_10MM_COEF         (0.01)

#define RS_TEMPERATURE_MIN              (31)
#define RS_TEMPERATURE_MAX              (81)

#define RS_ANGLE_MAX                    (360.0f)
#define RS_ANGLE_MIN                    (0.0f)

#define RS_RPM_300                      (300)
#define RS_RPM_600                      (600)
#define RS_RPM_1200                     (1200)
#define RS_DEFAULT_DISTANCE_MAX         (200.0f)
#define RS_DEFAULT_DISTANCE_MIN         (0.2f)
#define RS_DEFAULT_AZIMUTH              (-36001)
#define RS_ANGLE_INT_MIN                (0)
#define RS_ANGLE_INT_MAX                (36000)
#define RS_ANGLE_THRESHOLD              (36000)

#define RS16_AZIMUTH_DIFF_DEFAULT       (36)
#define RS32_AZIMUTH_DIFF_DEFAULT       (18)
#define RSBPEARL_AZIMUTH_DIFF_DEFAULT   (20)

#define RS16_RX_VALUE                   (0.03825f)
#define RS16_RY_VALUE                   (-0.01088f)
#define RS16_RZ_VALUE                   (0.0f)

#define RS32_RX_VALUE                   (0.03997f)
#define RS32_RY_VALUE                   (-0.01087f)
#define RS32_RZ_VALUE                   (0.0f)

#define RSBPEARL_RX_VALUE               (0.01697f)
#define RSBPEARL_RY_VALUE               (-0.0085f)
#define RSBPEARL_RZ_VALUE               (0.12644f)

#define RSBPEARL_MINI_RX_VALUE          (0.01473f)
#define RSBPEARL_MINI_RY_VALUE          (0.0085f)
#define RSBPEARL_MINI_RZ_VALUE          (0.09427f)


  /* timestamp struct */
  typedef struct
  {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t ms;
    uint16_t us;
  }__attribute__((packed)) ST_Timestamp;

  /* msop packet header struct */
  typedef struct
  {
    uint64_t sync;
    uint8_t reserved1[12];
    ST_Timestamp timestamp;
    uint8_t lidar_type;
    uint8_t reserved2[7];
    uint16_t temp_raw;
    uint8_t reserved3[2];
  }__attribute__((packed)) ST_MsopHeader;

  /* channel data struct */
  typedef struct
  {
    uint16_t distance;
    uint8_t intensity;
  }__attribute__((packed)) ST_Channel;

  /* block data struct */
  typedef struct
  {
    uint16_t id;
    uint16_t azimuth;
    ST_Channel channels[RS_CHANNELS_PER_BLOCK];
  }__attribute__((packed)) ST_MsopBlock;

  /* msop packet struct */
  typedef struct
  {
    ST_MsopHeader header;
    ST_MsopBlock blocks[RS_BLOCKS_PER_PKT];
    uint32_t index;
    uint16_t tail;
  }__attribute__((packed)) ST_MsopPkt;

  /* ethernet struct */
  typedef struct
  {
    uint8_t lidar_ip[4];
    uint8_t host_ip[4];
    uint8_t mac_addr[6];
    uint16_t local_port;
    uint16_t dest_port;
    uint16_t port3;
    uint16_t port4;
  }__attribute__((packed)) ST_EthNet;

  /* fov struct */
  typedef struct
  {
    uint16_t start_angle;
    uint16_t end_angle;
  }__attribute__((packed)) ST_FOV;

  /* firmware version serial number struct */
  typedef struct
  {
    uint8_t main_sn[5];
    uint8_t bottom_sn[5];
  }__attribute__((packed)) ST_Version;

  /* intensity calibration data struct of RSLiDAR 16*/
  typedef struct
  {
    uint8_t intensity_cali[240];
    uint8_t coef;
    uint8_t ver;
  }__attribute__((packed)) ST_RS16_Intensity;

  /* intensity calibration data struct of RSLiDAR 32*/
  typedef struct
  {
    uint8_t reserved[240];
    uint8_t coef;
    uint8_t ver;
  }__attribute__((packed)) ST_RS32_Intensity;

  /* serial number */
  typedef struct
  {
    uint8_t num[6];
  }__attribute__((packed)) ST_SN;

  /* RSLiDAR infomation status struct */
  typedef struct
  {
    uint8_t device_current[3];
    uint8_t main_current[3];
    uint16_t vol_12v;
    uint16_t vol_12vm;
    uint16_t vol_5v;
    uint16_t vol_3v3;
    uint16_t vol_2v5;
    uint16_t vol_1v2;
  }__attribute__((packed)) ST_Status;

  /* RSLiDAR diagno infomation struct */
  typedef struct
  {
    uint8_t reserved1[10];
    uint8_t checksum;
    uint16_t manc_err1;
    uint16_t manc_err2;
    uint8_t gps_status;
    uint16_t temperature1;
    uint16_t temperature2;
    uint16_t temperature3;
    uint16_t temperature4;
    uint16_t temperature5;
    uint8_t reserved2[5];
    uint16_t cur_rpm;
    uint8_t reserved3[7];
  }__attribute__((packed)) ST_Diagno;

  /* difop packet struct of RSLiDAR 16 */
  typedef struct
  {
    uint64_t sync;
    uint16_t rpm;
    ST_EthNet eth;
    ST_FOV fov;
    uint16_t static_base;
    uint16_t lock_phase;
    ST_Version version;
    ST_RS16_Intensity intensity;
    ST_SN sn;
    uint16_t zero_cali;
    uint8_t echo_mode;
    uint16_t sw_ver;
    ST_Timestamp timestamp;
    ST_Status status;
    uint8_t reserved1[11];
    ST_Diagno diagno;
    uint8_t gprmc[86];
    uint8_t static_cali[697];
    uint8_t pitch_cali[48];
    uint8_t reserved2[33];
    uint16_t tail;
  }__attribute__((packed)) ST_RS16_DifopPkt;

    /* difop packet struct of RSLiDAR 32 */
  typedef struct
  {
    uint64_t sync;
    uint16_t rpm;
    ST_EthNet eth;
    ST_FOV fov;
    uint16_t reserved1;
    uint16_t lock_phase;
    ST_Version version;
    ST_RS32_Intensity intensity;
    ST_SN sn;
    uint16_t zero_cali;
    uint8_t echo_mode;
    uint16_t sw_ver;
    ST_Timestamp timestamp;
    ST_Status status;
    uint8_t reserved2[11];
    ST_Diagno diagno;
    uint8_t gprmc[86];
    uint8_t pitch_cali[96];
    uint8_t yaw_cali[96];
    uint8_t reserved3[586];
    uint16_t tail;
  }__attribute__((packed)) ST_RS32_DifopPkt;

} //namespace rslidar
} // namespace robosense

#endif //ROBOSENSE_RSLIDAR_RSLIDAR_PACKET_H