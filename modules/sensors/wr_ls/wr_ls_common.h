#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>

#include "modules/sensors/proto/laser_config.pb.h"
#include "modules/sensors/proto/laser_scan.pb.h"
#include "modules/sensors/wr_ls/parser_base.h"
#include "modules/sensors/wr_ls/wr_ls_constants.h"

/*Fixed received buffer size*/
#define RECV_BUFFER_SIZE 65536

namespace apollo {
namespace sensors {
namespace wr_ls {

using apollo::sensors::LaserScan;
using apollo::sensors::WrLsConfig;

class CWrLsCommon {
 public:
  CWrLsCommon(CParserBase *parser);
  virtual ~CWrLsCommon();
  virtual int Init();
  int LoopOnce();
  void CheckAngleRange(WrLsConfig &config);
  void UpdateConfig(WrLsConfig &newConfig, uint32_t level = 0);
  virtual bool RebootDevice();

  double GetExpectedFreq() const { return dExpectedFreq; }

 protected:
  virtual int InitDevice() = 0;
  virtual int InitScanner();
  virtual int StopScanner();
  virtual int CloseDevice() = 0;

  /*Send command/message to the device and print out the response to the
   * console*/
  /**
   * \param [in]  req the command to send
   * \param [out] resp if not NULL, will be filled with the response package to
   * the command.
   */
  virtual int SendDeviceReq(const char *req,
                            std::vector<unsigned char> *resp) = 0;

  /*Read a datagram from the device*/
  /*
   * \param [in]  receiveBuffer data buffer to fill.
   * \param [in]  bufferSize max data size to buffer (0 terminated).
   * \param [out] length the actual amount of data written.
   * */
  virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize,
                          int *length) = 0;

  /*Helper function. Conver response of SendDeviceReq to string*/
  /*
   * \param [in] resp the response from SendDeviceReq
   * \returns response as string with special characters stripped out
   * */
  static std::string StringResp(const std::vector<unsigned char> &resp);

  /*Check the given device identify is supported by this driver*/
  /*
   * \param [in] strIdentify the identifier of the dvice.
   * \return indicate wether it's supported by this driver
   * */
  bool IsCompatibleDevice(const std::string strIdentify) const;

  void DumpLaserMessage(LaserScan &msg);

 protected:
  // diagnostic_updater::Updater mDiagUpdater;

 private:
  ros::NodeHandle mNodeHandler;
  ros::Publisher mScanPublisher;
  ros::Publisher mDataPublisher;

  // Parser
  CParserBase *mParser;

  bool mPublishData;
  double dExpectedFreq;

  unsigned char mRecvBuffer[RECV_BUFFER_SIZE];
  int mDataLength;

  // Diagnostics
  diagnostic_updater::DiagnosedPublisher<LaserScan> *mDiagPublisher;

  // Dynamic Reconfigure
  WrLsConfig mConfig;
  dynamic_reconfigure::Server<WrLsConfig> mDynaReconfigServer;
};
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
