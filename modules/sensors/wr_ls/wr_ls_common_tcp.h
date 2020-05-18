#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <boost/asio.hpp>

#include "modules/sensors/wr_ls/wr_ls_common.h"

namespace apollo {
namespace sensors {
namespace wr_ls {

class CWrLsCommonTcp : public CWrLsCommon {
 public:
  CWrLsCommonTcp(const std::string &hostname, const std::string &port,
                 int &timelimit, CParserBase *parser);
  virtual ~CWrLsCommonTcp();

 protected:
  /*Override functions*/
  virtual int InitDevice();
  virtual int CloseDevice();

  virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp);

  virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize,
                          int *length);

  /*Helper functions for boost ASIO*/
  int ReadWithTimeout(size_t timeout_ms, char *buffer, int buffer_size,
                      int *bytes_read = 0, bool *error = NULL);
  void HandleRead(boost::system::error_code error, size_t bytes_transfered);
  void CheckDeadLine();

 private:
  boost::asio::io_service mIOService;
  boost::asio::ip::tcp::socket mSocket;
  boost::asio::deadline_timer mDeadLine;
  boost::asio::streambuf mInputBuffer;
  boost::system::error_code mErrorCode;
  size_t mBytesReceived;

  std::string mHostName;
  std::string mPort;
  int mTimeLimit;
};

inline void CWrLsCommonTcp::HandleRead(boost::system::error_code error,
                                       size_t bytes_transfered) {
  mErrorCode = error;
  mBytesReceived += bytes_transfered;
}
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
