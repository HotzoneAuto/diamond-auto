#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// TODO replace
#include <std_msgs/String.h>

#include "modules/sensors/wr_ls/wr_ls_common.h"
namespace apollo {
namespace sensors {
namespace wr_ls {
class CWrLsCommonMockup : public CWrLsCommon {
 public:
  CWrLsCommonMockup(CParserBase *pParser);
  virtual ~CWrLsCommonMockup();

 protected:
  virtual int InitDevice();
  virtual int InitScanner();
  virtual int CloseDevice();

  virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp);
  virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize,
                          int *length);

 private:
  ros::NodeHandle nhMockup;
  ros::Subscriber subMockup;

  std_msgs::String::ConstPtr ptrMsgMockupData;
  void OnMockupMessage(const std_msgs::String::ConstPtr &msg);
}; /*class CWrLsCommonMockup*/
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
