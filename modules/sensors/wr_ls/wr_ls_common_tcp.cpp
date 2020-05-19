#include "modules/sensors/wr_ls/wr_ls_common_tcp.h"

#include <algorithm>
#include <iterator>

#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>

namespace apollo {
namespace sensors {
namespace wr_ls {

using boost::asio::ip::tcp;
using boost::lambda::_1;
using boost::lambda::var;
using apollo::cyber::Node;

CWrLsCommonTcp::CWrLsCommonTcp(const std::string &hostname,
                               const std::string &port, int &timelimit,
                               CParserBase *parser)
    : CWrLsCommon(parser, std::shared_ptr<Node> node),
      mSocket(mIOService),
      mDeadLine(mIOService),
      mHostName(hostname),
      mPort(port),
      mTimeLimit(timelimit) {
  /*Set up the deadline actor to implement timeouts*/
  mDeadLine.expires_at(boost::posix_time::pos_infin);
  CheckDeadLine();
}

int CWrLsCommonTcp::InitDevice() {
  /*Try to resolve the supplied hostname and port*/
  tcp::resolver::iterator iterator;
  try {
    tcp::resolver resolver(mIOService);
    tcp::resolver::query query(mHostName, mPort);
    iterator = resolver.resolve(query);
  } catch (boost::system::system_error &error) {
    AFATAL << "Could not resolve host: " << error.code().value() << ", "
           << error.code().message().c_str();

    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Resolve host failed!");

    return ExitError;
  }

  /*Try to connect to all possible endpoints*/
  boost::system::error_code errCode;
  bool success = false;
  for (; iterator != tcp::resolver::iterator(); ++iterator) {
    std::string strEndPoint =
        boost::lexical_cast<std::string>(iterator->endpoint());
    mSocket.close();

    /*Set the time out length*/
    AINFO << "Waiting %i seconds for device to connect..." << mTimeLimit;
    mDeadLine.expires_from_now(boost::posix_time::seconds(mTimeLimit));

    errCode = boost::asio::error::would_block;
    ADEBUG << "Attempting to connect to " << strEndPoint.c_str();
    mSocket.async_connect(iterator->endpoint(),
                          boost::lambda::var(errCode) = _1);

    /*Wait until timeout*/
    do {
      mIOService.run_one();
    } while (errCode == boost::asio::error::would_block);

    if (!errCode && mSocket.is_open()) {
      success = true;
      AINFO << "Successfully connected to " << strEndPoint.c_str();
      break;
    }

    AERROR << "Failed to connect to " << strEndPoint.c_str();
  }

  /*Check if connecting succeed*/
  if (!success) {
    AFATAL << "Could not connect to host " << mHostName.c_str() << ":"
           << mPort.c_str();
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Could not connect to host!");

    return ExitError;
  }

  mInputBuffer.consume(mInputBuffer.size());
  return ExitSuccess;
}

int CWrLsCommonTcp::CloseDevice() {
  if (mSocket.is_open()) {
    try {
      mSocket.close();
    } catch (boost::system::system_error &e) {
      AERROR << "An error occured during closing of the connection: "
             << e.code().value() << ":" << e.code().message().c_str();
    }
  }
}

void CWrLsCommonTcp::CheckDeadLine() {
  if (mDeadLine.expires_at() <=
      boost::asio::deadline_timer::traits_type::now()) {
    mSocket.close();
    mDeadLine.expires_at(boost::posix_time::pos_infin);
  }

  /*Nothing bad happened, go back to sleep*/
  mDeadLine.async_wait(boost::bind(&CWrLsCommonTcp::CheckDeadLine, this));
}

int CWrLsCommonTcp::ReadWithTimeout(size_t timeout_ms, char *buffer,
                                    int buffer_size, int *bytes_read,
                                    bool *error) {
  /*Set up the deadline to the proper timeout, error and delimiters*/
  mDeadLine.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
  const char end_delim = static_cast<char>(ETX);
  mErrorCode = boost::asio::error::would_block;
  mBytesReceived = 0;

  /*Read until ETX ending indicator*/
  boost::asio::async_read_until(
      mSocket, mInputBuffer, end_delim,
      boost::bind(&CWrLsCommonTcp::HandleRead, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));

  do {
    mIOService.run_one();
  } while (mErrorCode == boost::asio::error::would_block);

  if (mErrorCode) {
    /*
     * would_block means the connection is OK, but nothing coming in in time.
     * If any other error code is set, this means something bad happened.
     *
     * */
    if (mErrorCode != boost::asio::error::would_block) {
      AERROR << "SendDeviceReq: failed attempt to read from socket: "
             << mErrorCode.value() << ": " << mErrorCode.message().c_str();

      //   mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
      //                          "WR_LS - Could not connect to host!");

      if (error != NULL) {
        *error = true;
      }
    }

    return ExitError;
  }

  /*Avoid a buffer overflow by limiting the data we read*/
  size_t bytes_to_read =
      mBytesReceived > buffer_size - 1 ? buffer_size - 1 : mBytesReceived;
  size_t i = 0;
  std::istream is(&mInputBuffer);
  if (buffer != 0) {
    is.read(buffer, bytes_to_read);
    buffer[bytes_to_read] = 0;

    /*Consume the reset of message if necessary*/
    if (bytes_to_read < mBytesReceived) {
      AWARN << "Dropping %zu bytes to avoid buffer overflow"
            << mBytesReceived - bytes_to_read;
      mInputBuffer.consume(mBytesReceived - bytes_to_read);
    }
  } else {
    /*No buffer is provided, just drop the data*/
    AWARN << "WR_LS: Buffer overflow!";
    mInputBuffer.consume(mBytesReceived);
  }

  /*Set return bytes_read*/
  if (bytes_read != NULL) {
    *bytes_read = bytes_to_read;
  }

  return ExitSuccess;
}

int CWrLsCommonTcp::SendDeviceReq(const char *req,
                                  std::vector<unsigned char> *resp) {
  if (!mSocket.is_open()) {
    AERROR << "SendDeviceReq: Socket NOT open";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - SendDeviceReq: socket NOT open!");

    return ExitError;
  }

  /*Write request to device*/
  try {
    boost::asio::write(mSocket, boost::asio::buffer(req, strlen(req)));
  } catch (boost::system::system_error &e) {
    AERROR << "Write error for command: %s", req;
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - SendDeviceReq: Write command failed!");

    return ExitError;
  }

  /*Wait and get the response*/
  size_t timeout = 1000;
  const int BUF_SIZE = 1000;
  char buffer[BUF_SIZE];
  int bytes_read;
  if (ReadWithTimeout(timeout, buffer, BUF_SIZE, &bytes_read, NULL) ==
      ExitError) {
    AERROR_EVERY(1.0)
        << "SendDeviceReq: No full reply available for read after 1s";
    // mDiagUpdater.broadcast(
    //     diagnostic_msgs::DiagnosticStatus::ERROR,
    //     "WR_LS - SendDeviceReq: No full response for read after 5s.");

    return ExitError;
  }

  if (resp != NULL) {
    resp->resize(bytes_read);
    std::copy(buffer, buffer + bytes_read, &(*resp)[0]);
  }

  return ExitSuccess;
}

int CWrLsCommonTcp::GetDataGram(unsigned char *receiveBuffer, int bufferSize,
                                int *length) {
  if (!mSocket.is_open()) {
    AERROR << "GetDataGram: Socket NOT open";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - GetDataGram: socket NOT open!");

    return ExitError;
  }

  /*Try to read from device*/
  std::vector<unsigned char> resp;
  size_t timeout = 1000;
  bool error = false;
  char *buffer = reinterpret_cast<char *>(receiveBuffer);

  if (ReadWithTimeout(timeout, buffer, bufferSize, length, &error) !=
      ExitSuccess) {
    AERROR_EVERY(1.0)
        << "GetDataGram: No full reply available for read after 1s";
    // mDiagUpdater.broadcast(
    //     diagnostic_msgs::DiagnosticStatus::ERROR,
    //     "WR_LS - GetDataGram: No full response for read after 5s.");

    /*attempt to reconnect when the connection was terminated*/
    if (!mSocket.is_open()) {
      sleep(1);
      AINFO << "Device lost - attempt to reconnect";
      return Init();
    }

    return error ? ExitError : ExitSuccess;
  }

  /*simulate data frame*/
#if 0
        for (int i = *length; i > 5; i--)
        {
            receiveBuffer[i + 1] = receiveBuffer[i];
        }
        receiveBuffer[5] = 0x2A;
        receiveBuffer[6] = 0xCC;
        (*length)++;
#endif
  /*Uncomment this code block for debug..*/
  /*
          printf("\n=======================================\n");
          AINFO <<"Data Count: "<< *length;
          int idx = 1;
          while(idx <= *length)
          {
                  printf("%02X ", static_cast<uint8_t>(receiveBuffer[idx -1]));

                  idx++;
                  if(idx % 48 == 0)
                          printf("\n");
          }
          printf("\n");
          printf("\n=======================================\n");
  */

  return ExitSuccess;
}

CWrLsCommonTcp::~CWrLsCommonTcp() {
  StopScanner();
  CloseDevice();
}

}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
