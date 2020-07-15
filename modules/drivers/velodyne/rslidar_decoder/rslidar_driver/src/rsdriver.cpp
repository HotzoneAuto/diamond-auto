/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rsdriver.h"

namespace rslidar_driver
{
/****************************************************
 * @name rslidarDriver
 * @brief constructor
 * @param node
 * @param private_nh
 ***************************************************/
rslidarDriver::rslidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string pcap_file;
  private_nh.param("pcap", pcap_file, std::string(""));

  // open rslidar input device or file
  if (pcap_file != "")  // have PCAP file?
  {
    std::string lidar_model;
    private_nh.param("model", lidar_model, std::string("RS16"));

    int pkt_rate = 1000;
    if (lidar_model == "RS16")
    {
      pkt_rate = 1000;
    }
    else if (lidar_model == "RS32")
    {
      pkt_rate = 2000;
    }

    // read data from packet capture file
    this->input_handler_.reset(new rslidar_driver::InputPCAP(private_nh, pkt_rate, pcap_file));
  }
  else
  {
    // read data from live socket
    this->input_handler_.reset(new rslidar_driver::InputSocket(private_nh));
  }

  // msop packet output topic
  std::string output_msop_topic;
  private_nh.param("output_msop_topic", output_msop_topic, std::string("rslidar_packets_msop"));
  this->msop_pub_ = node.advertise<rslidar_msgs::rslidarPacket>(output_msop_topic, 10);

  // difop packet output topic
  std::string output_difop_topic;
  private_nh.param("output_difop_topic", output_difop_topic, std::string("rslidar_packets_difop"));
  this->difop_pub_ = node.advertise<rslidar_msgs::rslidarPacket>(output_difop_topic, 10);
}


/*********************************************************
 * @name poll
 * @brief get packets mainloop
 * @return true unless end of file reached
 *********************************************************/
bool rslidarDriver::poll(void)
{
  rslidar_msgs::rslidarPacketPtr packet_ptr(new rslidar_msgs::rslidarPacket);

  while (ros::ok())
  {
    rslidar_msgs::rslidarPacket packet_msg;
    E_INPUT_STATE ret = this->input_handler_->getPacket(&packet_msg, 100);
    switch (ret)
    {
      case E_ERROR_INVALID_PARAM:
        ROS_ERROR("[driver] invalid param");
        break;
      case E_ERROR_SOCKET:
        ROS_ERROR("[driver] socket read fail");
        break;
      case E_ERROR_PKT_LEN:
        ROS_ERROR("[driver] pkt length no match");
        break;
      case E_PKT_MSOP:
        *packet_ptr = packet_msg;
        this->msop_pub_.publish(packet_ptr);
        break;
      case E_PKT_DIFOP:
        *packet_ptr = packet_msg;
        this->difop_pub_.publish(packet_ptr);
        break;
      case E_PCAP_EMPTY:
//        ROS_ERROR("[driver] pcap filter no match");
        break;
      case E_PCAP_REPEAT:
        ROS_INFO("[driver] pcap repeat");
        break;
      case E_OK:
      default:
        break;
    }
    ros::spinOnce();
  }
}

}  // namespace rslidar_driver
