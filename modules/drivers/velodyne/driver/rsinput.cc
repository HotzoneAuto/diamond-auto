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
#include "rsinput.h"

// extern volatile sig_atomic_t flag;
namespace rslidar_driver {
static const size_t packet_size = 1248;

static const uint16_t RS_MSOP_DEFAULT_PORT =
    6699;  // rslidar default data port on PC
static const uint16_t RS_DIFOP_DEFAULT_PORT =
    7788;  // rslidar default difop data port on PC
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/**************************************************************
 * @name Input
 * @brief constructor
 * @param private_nh: ROS private handle for calling node.
 **************************************************************/
Input::Input(apollo::drivers::velodyne::Config config) : config_(config) {
  dev_ip_ = config_.dev_ip();
  if (!this->dev_ip_.empty()) {
    AINFO << "[input] device IP: " << this->dev_ip_;
  } else {
    AERROR << "[input] ip is empty.";
  }
  msop_port_ = config_.msop_port();
  difop_port_ = config_.difop_port();

  AINFO << this->msop_port_ << ", difop port: " << this->difop_port_;
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////
/**************************************************************
 * @name InputSocket
 * @brief constructor
 * @param private_nh: ROS private handle for calling node.
 **************************************************************/
InputSocket::InputSocket(apollo::drivers::velodyne::Config config)
    : Input(config) {
  if (this->dev_ip_.empty()) {
    AERROR << "[input] lidar ip is empty()!";
  } else {
    AINFO << "[input] device ip: " << this->dev_ip_;
  }

  this->msop_fd_ = setupSocket(this->dev_ip_, this->msop_port_);
  this->difop_fd_ = setupSocket(this->dev_ip_, this->difop_port_);
}

/***********************************************************
 * @name ~InputSocket
 * @brief destructor, close msop and difop socket handler
 ***********************************************************/
InputSocket::~InputSocket(void) {
  (void)close(this->difop_fd_);
  (void)close(this->msop_fd_);
}

/***********************************************************
 * @name setupSocket
 * @brief create socket handler
 * @param ip: ip address
 * @param port: port number
 * @return socket handler value
 ***********************************************************/
int InputSocket::setupSocket(const std::string& ip, const uint16_t port) {
  int fd = -1;

  if (!ip.empty()) {
    inet_aton(this->dev_ip_.c_str(), &this->dev_addr_);
  }

  fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd == -1) {
    AERROR << "[input] socket fail";  // TODO: ROS_ERROR errno
    return -1;
  }

  int opt = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt,
                 sizeof(opt))) {
    AERROR << "[input] setsockopt error!";
    return -1;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(fd, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1) {
    AERROR << "[input] bind fail";  // TODO: ROS_ERROR errno
    return -1;
  }

  if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "[input] fcntl fail";
    return -1;
  }

  return fd;
}

/*****************************************************
 * @name getPacket
 * @brief get packet interface
 * @param pkt: packet buffer
 * @param timeout
 * @return error code
 *****************************************************/
E_INPUT_STATE InputSocket::getPacket(rslidarPacket* pkt,
                                     const unsigned int timeout) {
  E_INPUT_STATE ret;
  double time1 = apollo::cyber::Time().Now().ToSecond();

  if (pkt == NULL) {
    return E_ERROR_INVALID_PARAM;
  }
  /*
    if (flag == 0)
    {
      abort();
      return E_MAX;
    }
  */
  struct timeval tmout;
  tmout.tv_sec = timeout / 1000;
  tmout.tv_usec = (timeout % 1000) * 1000;

  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(this->msop_fd_, &fds);
  FD_SET(this->difop_fd_, &fds);

  int max_fd = std::max(this->msop_fd_, this->difop_fd_);
  // AINFO<<msop_port_<<" select start "<<tmout.tv_sec;
  int retval = select(max_fd + 1, &fds, NULL, NULL, &tmout);
  // AINFO<<msop_port_<<" select end "<<tmout.tv_usec;
  if (retval == -1 && errno == EINTR) {
    ret = E_ERROR_SOCKET;
  } else if (retval == -1) {
    ret = E_ERROR_SOCKET;
  } else if (retval) {
    ssize_t n = packet_size;
    if (FD_ISSET(this->msop_fd_, &fds)) {
      n = recvfrom(this->msop_fd_, &(pkt->data[0]), packet_size, 0, NULL, NULL);
      if (n < 0) AERROR << "RECV_FROM" << this->msop_fd_ << "error";
      ret = E_PKT_MSOP;
    } else if (FD_ISSET(this->difop_fd_, &fds)) {
      n = recvfrom(this->difop_fd_, &(pkt->data[0]), packet_size, 0, NULL,
                   NULL);
      if (n < 0) AERROR << "RECV_FROM" << this->difop_fd_ << "error";
      ret = E_PKT_DIFOP;
    } else {
      ret = E_OK;
    }

    if (n != packet_size) {
      ret = E_ERROR_PKT_LEN;
    }
  }

  double time2 = apollo::cyber::Time().Now().ToSecond();
  pkt->stamp = (time2 + time1) / 2.0;

  return ret;
}

}  // namespace rslidar_driver
