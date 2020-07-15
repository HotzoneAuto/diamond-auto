/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{

namespace rslidar = robosense::rslidar;

/************************************************************
 * @name Convert
 * @brief constructor
 * @param node: ros node handler
 * @param private_nh: ros private node handler
 ************************************************************/
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string cali_dir;
  private_nh.param("cali_path", cali_dir, std::string("."));
  std::string lidar_type;
  private_nh.param("model", lidar_type, std::string("RS16"));
  std::string resolution_type;
  private_nh.param("resolution_type", resolution_type, std::string("0.5cm"));
  int echo_mode;
  private_nh.param("echo_mode", echo_mode, 1);
  float max_distance;
  private_nh.param("max_distance", max_distance, 200.0f);
  float min_distance;
  private_nh.param("min_distance", min_distance, 0.2f);
  if (min_distance >= max_distance)
  {
    min_distance = 0.2f;
    max_distance = 200.0f;
  }
  float start_angle;
  private_nh.param("start_angle", start_angle, 0.0f);
  float end_angle;
  private_nh.param("end_angle", end_angle, 360.0f);

  

  rslidar::ST_Param param;

  if (lidar_type == "RS16")
  {
    param.lidar = rslidar::RS_Type_Lidar16;
    private_nh.param("channel_filter", param.channel_filter, std::string("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15"));
  }
  else if (lidar_type == "RS32")
  {
    param.lidar = rslidar::RS_Type_Lidar32;
    private_nh.param("channel_filter", param.channel_filter, std::string("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31"));
  }
  else if (lidar_type == "RSBPEARL")
  {
    param.lidar = rslidar::RS_Type_LidarBpearl;
    private_nh.param("channel_filter", param.channel_filter, std::string("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31"));
  }
  else if (lidar_type == "RSBPEARL_MINI")
  {
    param.lidar = rslidar::RS_Type_LidarBpearl_Mini;
    private_nh.param("channel_filter", param.channel_filter, std::string("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31"));
  }

  if (resolution_type == "0.5cm")
  {
    param.resolution = rslidar::RS_Resolution_5mm;
  }
  else
  {
    param.resolution = rslidar::RS_Resolution_10mm;
  }

  param.echo = rslidar::RS_Echo_Mode (echo_mode);

  param.cut_angle = 0.0;
  param.max_distance = max_distance;
  param.min_distance = min_distance;
  param.start_angle = start_angle;
  param.end_angle = end_angle;

  //RSLiDAR decoder create
  this->decoder_ = boost::shared_ptr<rslidar::RSLidarDecoder<pcl::PointXYZI>>(new rslidar::RSLidarDecoder<pcl::PointXYZI>(param));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  private_nh.param("output_points_topic", output_points_topic, std::string("rslidar_points"));
  this->pointcloud_pub_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);
  //pointcloud buffer init
  this->pc_buf_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(new pcl::PointCloud<pcl::PointXYZI>);
  this->pc_buf_->header.frame_id = "rslidar";
  this->pc_buf_->is_dense = false;
  this->pc_buf_->clear();

  //subcribe msop packet topic
  std::string msop_pkts_topic;
  private_nh.param("msop_packets_topic", msop_pkts_topic, std::string("rslidar_packets_msop"));
  this->msop_sub_ = node.subscribe(msop_pkts_topic, 10, &Convert::processMsopPkt, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
  //subcribe difop packet topic
  std::string difop_pkts_topic;
  private_nh.param("difop_packets_topic", difop_pkts_topic, std::string("rslidar_packets_difop"));
  this->difop_sub_ = node.subscribe(difop_pkts_topic, 10, &Convert::processDifopPkt, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
}

/**************************************************
 * @name processMsopPkt
 * @brief Callback for msop packet messages
 * @param pkt: msop message pointer
 *************************************************/
void Convert::processMsopPkt(const rslidar_msgs::rslidarPacket::ConstPtr& pkt)
{
  std::vector<pcl::PointXYZI> pc_vec;
  double timestamp;
  const uint8_t* data = &pkt->data[0];

  pc_vec.clear();
  rslidar::RS_Decode_Result result = this->decoder_->processMsopPkt(data, pc_vec, timestamp);
  if (result == rslidar::RS_Decode_Fail || result == rslidar::RS_Param_Invalid)
  {
    return;
  }
  for (int i = 0; i < pc_vec.size(); i++)
  {
    pc_buf_->push_back(pc_vec[i]);
  }
  if (result == robosense::rslidar::RS_Frame_Split)
  {
//    this->pc_buf_->header.stamp = pkt->stamp;

    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*this->pc_buf_, outMsg);

    this->pointcloud_pub_.publish(outMsg);

    this->pc_buf_->clear();
  }
}

/*************************************************************
 * @name processDifopPkt
 * @brief callback for difop packet message
 * @param pkt: difop packet pointer
 *************************************************************/
void Convert::processDifopPkt(const rslidar_msgs::rslidarPacket::ConstPtr &pkt)
{
  const uint8_t* data = &pkt->data[0];
  this->decoder_->processDifopPkt(data);
}

}  // namespace rslidar_pointcloud
