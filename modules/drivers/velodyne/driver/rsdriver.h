/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#ifndef _RSDRIVER_H_
#define _RSDRIVER_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_driver/rslidarNodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "input.h"

namespace rslidar_driver
{
  /*******************************************************
   * @name class rslidarDriver
   * @brief rslidarDriver, handler packet input from RSLiDAR or pcap
   */
class rslidarDriver
{
public:
  rslidarDriver();

  ~rslidarDriver()
  {
  }

  bool poll(void);

private:

  boost::shared_ptr<Input> input_handler_;
  ros::Publisher msop_pub_;
  ros::Publisher difop_pub_;
};

}  // namespace rslidar_driver

#endif
