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
#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include "cyber/cyber.h"
#include "modules/drivers/velodyne/proto/config.pb.h"
namespace rslidar_driver {
struct rslidarPacket {
  double stamp;
  uint8_t data[1248];
};

/* input result error code define */
enum E_INPUT_STATE {
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
class Input {
 public:
  Input(apollo::drivers::velodyne::Config config_);  //, uint16_t port);

  virtual ~Input() {}

  virtual E_INPUT_STATE getPacket(rslidarPacket* pkt,
                                  const unsigned int timeout) = 0;

 protected:
  apollo::drivers::velodyne::Config config_;
  uint16_t msop_port_;
  uint16_t difop_port_;
  std::string dev_ip_;
};

/*****************************************************
 * @name class InputSocket
 * @brief Live rslidar input from socket.
 ****************************************************/
class InputSocket : public Input {
 public:
  InputSocket(apollo::drivers::velodyne::Config config);

  virtual ~InputSocket();

  virtual E_INPUT_STATE getPacket(rslidarPacket* pkt,
                                  const unsigned int timeout);

 private:
  int setupSocket(const std::string& ip, const uint16_t port);

 private:
  int msop_fd_;
  int difop_fd_;
  in_addr dev_addr_;
};

/*****************************************************
 * @name class InputPCAP
 * @brief rslidar input from PCAP dump file.
 ****************************************************/

}  // namespace rslidar_driver
