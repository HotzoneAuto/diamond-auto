#include "modules/sensors/wr_ls/wr_ls_common_mockup.h"
namespace apollo {
namespace sensors {
namespace wr_ls {
CWrLsCommonMockup::CWrLsCommonMockup(CParserBase *pParser)
    : CWrLsCommon(pParser) {
  subMockup = nhMockup.subscribe("datagram", 1,
                                 &CWrLsCommonMockup::OnMockupMessage, this);
}

CWrLsCommonMockup::~CWrLsCommonMockup() {
  // Do Nothing...
}

int CWrLsCommonMockup::CloseDevice() {
  AINFO << "Mockup - Close Device";
  return 0;
}

int CWrLsCommonMockup::SendDeviceReq(const char *req,
                                     std::vector<unsigned char> *resp) {
  AERROR << "Mockup - SendDeviceReq(), this should never be called";
  return ExitError;
}

int CWrLsCommonMockup::InitDevice() {
  AINFO << "Mockup - InitDevice()";
  return ExitSuccess;
}

int CWrLsCommonMockup::InitScanner() {
  AINFO << "Mockup - InitScanner()";
  return ExitSuccess;
}

int CWrLsCommonMockup::GetDataGram(unsigned char *recvBuffer, int buffer_size,
                                   int *length) {
  ADEBUG << "Mockup - GetDataGram";

  /*Wait until data received...*/
  while (!ptrMsgMockupData) {
    if (!ros::ok()) {
      return ExitError;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  /*Copy out the message data*/
  std::vector<char> data(ptrMsgMockupData->data.begin(),
                         ptrMsgMockupData->data.end());
  data.push_back('\0');
  *length = ptrMsgMockupData->data.length();

  /*Reset message data structure*/
  ptrMsgMockupData.reset();

  if (buffer_size < (*length + 1)) {
    AERROR << "Mockup - Memory overflow!";
    return ExitError;
  }

  strncpy(reinterpret_cast<char *>(recvBuffer), &data[0], *length + 1);
  return ExitSuccess;
}

void CWrLsCommonMockup::OnMockupMessage(const std_msgs::String::ConstPtr &msg) {
  if (ptrMsgMockupData) {
    AWARN << "Mockup - Dropping mockup datagram message!";
    ptrMsgMockupData.reset();
  }

  ptrMsgMockupData = msg;
}
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
