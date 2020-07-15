/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __RSLIDAR_INPUT_H_
#define __RSLIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <sensor_msgs/TimeReference.h>

namespace rslidar_driver
{

  /* input result error code define */
  enum E_INPUT_STATE
  {
    E_OK = 0,
    E_ERROR_INVALID_PARAM = 1,
    E_ERROR_SOCKET = 2,
    E_ERROR_PKT_LEN = 4,
    E_PKT_MSOP = 8,
    E_PKT_DIFOP = 16,
    E_PCAP_EMPTY = 32,
    E_PCAP_REPEAT = 64,
    E_MAX
  };

/*****************************************************************
 * @name class Input
 * @brief input base class, get packet of RSLiDAR from live or pcap file
 */
class Input
{
public:
  Input(ros::NodeHandle private_nh);//, uint16_t port);

  virtual ~Input()
  {
  }

  virtual E_INPUT_STATE getPacket(rslidar_msgs::rslidarPacket* pkt, const unsigned int timeout) = 0;

protected:
  ros::NodeHandle private_nh_;
  uint16_t msop_port_;
  uint16_t difop_port_;
  std::string dev_ip_;
};


/*****************************************************
 * @name class InputSocket
 * @brief Live rslidar input from socket.
 ****************************************************/
class InputSocket : public Input
{
public:
  InputSocket(ros::NodeHandle private_nh);

  virtual ~InputSocket();

  virtual E_INPUT_STATE getPacket(rslidar_msgs::rslidarPacket* pkt, const unsigned int timeout);

private:
  int setupSocket(const std::string & ip, const uint16_t port);
private:
  int msop_fd_;
  int difop_fd_;
  in_addr dev_addr_;
};

/*****************************************************
 * @name class InputPCAP
 * @brief rslidar input from PCAP dump file.
 ****************************************************/
class InputPCAP : public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh, double packet_rate = 0.0, std::string filename = "",
      bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

  virtual ~InputPCAP();

  virtual E_INPUT_STATE getPacket(rslidar_msgs::rslidarPacket* pkt, const unsigned int timeout);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t* pcap_;
  bpf_program msop_filter_;
  bpf_program difop_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};
}

#endif  // __RSLIDAR_INPUT_H
