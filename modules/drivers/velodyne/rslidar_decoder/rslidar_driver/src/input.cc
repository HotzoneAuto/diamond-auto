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
#include "input.h"

extern volatile sig_atomic_t flag;
namespace rslidar_driver
{
static const size_t packet_size = sizeof(rslidar_msgs::rslidarPacket().data);

static const uint16_t RS_MSOP_DEFAULT_PORT = 6699;   // rslidar default data port on PC
static const uint16_t RS_DIFOP_DEFAULT_PORT = 7788;  // rslidar default difop data port on PC
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/**************************************************************
 * @name Input
 * @brief constructor
 * @param private_nh: ROS private handle for calling node.
 **************************************************************/
Input::Input(ros::NodeHandle private_nh) : private_nh_(private_nh)
{
  private_nh.param("device_ip", this->dev_ip_, std::string("192.168.1.200"));
  if (!this->dev_ip_.empty())
  {
    ROS_INFO_STREAM("[input] device IP: " << this->dev_ip_);
  }
  else
  {
    ROS_ERROR("[input] ip is empty.");
  }
  int port;
  private_nh.param("msop_port", port, (int)RS_MSOP_DEFAULT_PORT);
  this->msop_port_ = port;
  private_nh.param("difop_port", port, (int)RS_DIFOP_DEFAULT_PORT);
  this->difop_port_ = port;

  ROS_INFO_STREAM("[input] msop port: "<<this->msop_port_<<", difop port: "<<this->difop_port_);
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////
/**************************************************************
 * @name InputSocket
 * @brief constructor
 * @param private_nh: ROS private handle for calling node.
 **************************************************************/
InputSocket::InputSocket(ros::NodeHandle private_nh) : Input(private_nh)
{
  if (this->dev_ip_.empty())
  {
    ROS_FATAL("[input] lidar ip is empty()!");
  }
  else
  {
    ROS_INFO_STREAM("[input] device ip: "<<this->dev_ip_);
  }

  this->msop_fd_ = setupSocket(this->dev_ip_, this->msop_port_);
  this->difop_fd_ = setupSocket(this->dev_ip_, this->difop_port_);
}

/***********************************************************
 * @name ~InputSocket
 * @brief destructor, close msop and difop socket handler
 ***********************************************************/
InputSocket::~InputSocket(void)
{
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
int InputSocket::setupSocket(const std::string & ip, const uint16_t port)
{
  int fd = -1;

  if (!ip.empty())
  {
    inet_aton(this->dev_ip_.c_str(), &this->dev_addr_);
  }

  fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd == -1)
  {
    ROS_ERROR("[input] socket fail");  // TODO: ROS_ERROR errno
    return -1;
  }

  int opt = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    ROS_ERROR("[input] setsockopt error!");
    return -1;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(fd, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    ROS_ERROR("[input] bind fail");  // TODO: ROS_ERROR errno
    return -1;
  }

  if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    ROS_ERROR("[input] fcntl fail");
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
E_INPUT_STATE InputSocket::getPacket(rslidar_msgs::rslidarPacket* pkt, const unsigned int timeout)
{
  E_INPUT_STATE ret;
  double time1 = ros::Time::now().toSec();

  if (pkt == NULL)
  {
    return E_ERROR_INVALID_PARAM;
  }

  if (flag == 0)
  {
    abort();
    return E_MAX;
  }

  struct timeval tmout;
  tmout.tv_sec = timeout/1000;
  tmout.tv_usec = (timeout%1000)*1000;

  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(this->msop_fd_, &fds);
  FD_SET(this->difop_fd_, &fds);

  int max_fd = std::max(this->msop_fd_, this->difop_fd_);
  int retval = select(max_fd+1, &fds, NULL, NULL, &tmout);

  if (retval == -1 && errno == EINTR)
  {
    ret = E_ERROR_SOCKET;
  }
  else if (retval == -1)
  {
    ret = E_ERROR_SOCKET;
  }
  else if (retval)
  {
    ssize_t n = packet_size;
    if (FD_ISSET(this->msop_fd_, &fds))
    {
      n = recvfrom(this->msop_fd_, &(pkt->data[0]), packet_size, 0, NULL, NULL);
      ret = E_PKT_MSOP;
    }
    else if (FD_ISSET(this->difop_fd_, &fds))
    {
      n = recvfrom(this->difop_fd_, &(pkt->data[0]), packet_size, 0, NULL, NULL);
      ret = E_PKT_DIFOP;
    }
    else
    {
      ret = E_OK;
    }

    if (n != packet_size)
    {
      ret = E_ERROR_PKT_LEN;
    }
  }

  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0);

  return ret;
}
////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/**********************************************************************
 * @name InputPCAP
 * @brief constructor
 * @param private_nh ROS private handle for calling node.
 * @param packet_rate expected device packet frequency (Hz)
 * @param filename PCAP dump file name
 * @param read_once: only play once flag
 * @param read_fast: quick play flag
 * @param repeat_delay: delay time when play next time
 ***********************************************************************/
InputPCAP::InputPCAP(ros::NodeHandle private_nh, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = NULL;
  // get parameters using private node handle
  private_nh.param("read_once", read_once_, false);
  private_nh.param("read_fast", read_fast_, false);
  private_nh.param("repeat_delay", repeat_delay_, 0.0);

  if (read_once_)
  {
    ROS_INFO("[input] Read input file only once.");
  }

  if (read_fast_)
  {
    ROS_INFO("[input] Read input file as quickly as possible.");
  }

  if (repeat_delay_ > 0.0)
  {
    ROS_INFO("[input] Delay %.3f seconds before repeating input file.", repeat_delay_);
  }

  // Open the PCAP dump file
  ROS_INFO_STREAM("[input] PCAP file: " << filename_);
  this->pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);

  if (this->pcap_ == NULL)
  {
    ROS_ERROR("[input] pcap file open fail.");
    return;
  }

  std::string msop_filter("src host ");
  if (this->dev_ip_ != "")  // using specific IP?
  {
    msop_filter += this->dev_ip_;
  }
  msop_filter += " && udp dst port " + std::to_string(this->msop_port_);
  int ret = pcap_compile(pcap_, &this->msop_filter_, msop_filter.c_str(), 1, PCAP_NETMASK_UNKNOWN);
  if (ret < 0)
  {
    ROS_ERROR_STREAM("[input] pcap msop filter compile fail. filter: "<<msop_filter);
  }

  std::string difop_filter("src host ");
  if (this->dev_ip_ != "")  // using specific IP?
  {
    difop_filter += this->dev_ip_;
  }
  difop_filter += " && udp dst port " + std::to_string(this->difop_port_);
  ret = pcap_compile(pcap_, &this->difop_filter_, difop_filter.c_str(), 1, PCAP_NETMASK_UNKNOWN);
  if (ret < 0)
  {
    ROS_ERROR_STREAM("[input] pcap difop filter compile fail. filter: "<<difop_filter);
  }
}

/**************************************************
 * @name ~InputPCAP
 * @brief destructor, close pcap file handler
 *************************************************/
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one rslidar packet. */
/******************************************************
 * @name getPacket
 * @brief get packet interface
 * @param pkt : packet buffer
 * @param timeout
 * @return error code
 ******************************************************/
E_INPUT_STATE InputPCAP::getPacket(rslidar_msgs::rslidarPacket* pkt, const unsigned int timeout)
{
  (void)timeout;

  E_INPUT_STATE ret;

  if (pkt == NULL)
  {
    return E_ERROR_INVALID_PARAM;
  }
  if (flag == 0)
  {
    abort();
    return E_MAX;
  }

  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  int retval = pcap_next_ex(pcap_, &header, &pkt_data);
  if (retval >= 0)
  {
    if ((0 != pcap_offline_filter(&this->msop_filter_, header, pkt_data)))
    {
      memcpy(&(pkt->data[0]), pkt_data+42, packet_size);
      ret = E_PKT_MSOP;
    }
    else if ((0 != pcap_offline_filter(&this->difop_filter_, header, pkt_data)))
    {
      memcpy(&(pkt->data[0]), pkt_data+42, packet_size);
      ret = E_PKT_DIFOP;
    }
    else
    {
      ret = E_PCAP_EMPTY;
    }

    pkt->stamp = ros::Time::now();

    if (!read_fast_)
    {
      packet_rate_.sleep();
    }
  }
  else
  {
    if (repeat_delay_ > 0.0)
    {
      ROS_INFO("[input] end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    ROS_DEBUG("[input] replaying rslidar dump file");

    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    ret = E_PCAP_REPEAT;
  }

  return ret;
}
}//
