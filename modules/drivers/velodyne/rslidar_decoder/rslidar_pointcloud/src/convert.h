/* -*- mode: C++ -*- */
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

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_pointcloud/CloudNodeConfig.h>
#include "rslidar_decoder.hpp"
#include <rslidar_msgs/rslidarPacket.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace rslidar_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

private:
  void processMsopPkt(const rslidar_msgs::rslidarPacket::ConstPtr& pkt);
  void processDifopPkt(const rslidar_msgs::rslidarPacket::ConstPtr& pkt);

  boost::shared_ptr<robosense::rslidar::RSLidarDecoder<pcl::PointXYZI>> decoder_;
  ros::Subscriber msop_sub_;
  ros::Subscriber difop_sub_;
  ros::Publisher pointcloud_pub_;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pc_buf_;
};

}  // namespace rslidar_pointcloud
#endif
