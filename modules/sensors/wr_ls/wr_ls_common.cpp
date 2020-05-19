#include "modules/sensors/wr_ls/wr_ls_common.h"

#include <cstdio>
#include <cstring>

namespace apollo {
namespace sensors {
namespace wr_ls {

CWrLsCommon::CWrLsCommon(CParserBase *parser, std::shared_ptr<Node> node)
    :node_(node),
      dExpectedFreq(15.0), /* Default frequency */
      mParser(parser) {
  /*Initialize receive buffer*/
  memset(mRecvBuffer, RECV_BUFFER_SIZE, 0);
  mDataLength = 0;

  /*Set reconfigure callback*/
  //   dynamic_reconfigure::Server<wr_ls::WrLsConfig>::CallbackType f;
  //   f = boost::bind(&wr_ls::CWrLsCommon::UpdateConfig, this, _1, _2);
  //   mDynaReconfigServer.setCallback(f);

  /*Set data publisher (used for debug)*/
  // ros::NodeHandle ndHomePublisher("~");  // for config parameter...
  // ndHomePublisher.param<bool>("publish_datagram", mPublishData, false);
  if (mPublishData) {
    /*datagram publish is enabled*/
    // mDataPublisher = mNodeHandler.advertise<std_msgs::String>("datagram", 1000);
    datagram_writer_ = node_->CreateWriter<apollo::sensors::DataGram>("/apollo/sensors/wr_ls/datagram");
  }

  /*Set scan publisher*/
  // mScanPublisher = mNodeHandler.advertise<LaserScan>("scan", 1000);
  scan_writer_ = node_->CreateWriter<LaserScan>("/apollo/sensors/wr_ls/laser_scan");

//   mDiagUpdater.setHardwareID("none");
//   mDiagPublisher = new diagnostic_updater::DiagnosedPublisher<LaserScan>(
//       mScanPublisher, mDiagUpdater,
//       /* frequency should be target +- 10% */
//       diagnostic_updater::FrequencyStatusParam(&dExpectedFreq, &dExpectedFreq,
//                                                0.1, 10),
//       /*timestamp delta can be from 1.1 to 1.3x what it ideally is*/
//       diagnostic_updater::TimeStampStatusParam(
//           -1, 1.3 * 1.0 / dExpectedFreq - mConfig.time_offset));

//   ROS_ASSERT(mDiagPublisher);
}

int CWrLsCommon::StopScanner() {
  int result = 0;

#ifdef CMD_STOP_STREAM_DATA /* TODO: Enable following code block when stop \
                               command defined. */
  result = SendDeviceReq(CMD_STOP_STREAM_DATA, NULL);
  if (0 != result) {
    // use printf because we couldn't use AERROR from destructor
    printf("STOP Scan ERROR!\n");
  } else {
    printf("Streaming scan data stopped.\n");
  }
#endif
  return result;
}

bool CWrLsCommon::RebootDevice() {
#ifdef CMD_REBOOT_DEVICE /*TODO: Enable following code block when commands \
                            defined.*/
  /*Set maintenance access mode to allow reboot to be sent*/
  std::vector<unsigned char> respAccess;
  int result = SendDeviceReq(CMD_SET_MAINTENANCE_ACCESS_MODE, &respAccess);
  if (0 != result) {
    AERROR << "WR_LS - Error setting access mode";
    // mDiagUpdater.broadcast(diagnostic_msgs::DisgnosticStatus::ERROR,
    //                        "WR_LS - Error setting access mode");

    return false;
  }

  std::string strAccessResp = StringResp(respAccess);
  if (strAccessResp != "sAN SetAccessMode 1") {
    AERROR << "WR_LS - Error setting access mode, unexpected response : "
                     << strAccessResp;
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error setting access mode.");

    return false;
  }

  /*send reboot command*/
  std::vector<unsigned char> respReboot result =
      SendDeviceReq(CMD_REBOOT, &respReboot);
  if (0 != result) {
    AERROR << "WR_LS - Error rebooting device";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error rebooting device");

    return false;
  }

  std::string strRebootResp = StringResp(respReboot);
  if (strRebootResp != "sAN mSCreboot") {
    AERROR << "WR_LS - Error setting access mode, unexpected response : "
                     << strRebootResp;
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error rebooting device");

    return false;
  }

  AINFO << "WR_LS - Rebooted scanner";
#endif
  return true;
}

int CWrLsCommon::Init() {
  int result = InitDevice();
  if (0 != result) {
    AFATAL << "Failed to init device: " << result;
    return result;
  }

  result = InitScanner();
  if (0 != result) {
    AFATAL << "Failed to init scanner: " << result;
  }

  return result;
}

int CWrLsCommon::InitScanner() {
#ifdef CMD_DEVICE_INFO /*TODO: Enable following code block when command \
                          defined*/
  /*Read device identify*/
  std::vector<unsigned char> respIdentify;
  int result = SendDeviceReq(CMD_READ_IDENTIFY, &respIdentify);
  if (0 != result) {
    AERROR << "WR_LS - Error reading variable 'DeviceIdent'.";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error reading variable 'DeviceIdent'.");
  }

  /*Read device variable 'SerialNumber' by name.*/
  std::vector<unsigned char> respSerialNumber;
  result = SendDeviceReq(CMD_READ_SERIAL_NUMBER, &respSerialNumber);
  if (0 != result) {
    AERROR << "WR_LS - Error reading variable 'SerialNumber'.";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error reading variable 'SerialNumber'.");
  }

  /*Set hardware ID based on device identify and serial number*/
  std::string strIdentify = StringResponse(respIdentify);
  std::string strSerialNumber = StringResponse(respSerialNumber);
//   mDiagUpdater.setHardwareID(strIdentify + " " + strSerialNumber);

  if (!IsCompatibleDevice(strIdentify)) {
    AERROR << "WR_LS - Error Unsuppored identify %s", strIdentify;
    return ExitFatal;
  }

  /*Read device variable 'FirmwareVersion' by name.*/
  result = SendDeviceReq(CMD_READ_FIRMWARE_VERSION, NULL);
  if (0 != result) {
    AERROR << "WR_LS - Error reading variable 'FirmwareVersion'.";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error reading variable 'FirmwareVersion'.");
  }

  /*Read Device State*/
  std::vector<unsigned char> respDeviceState;
  result = SendDeviceReq(CMD_READ_DEVICE_STATE, &respDeviceState);
  if (0 != result) {
    AERROR "WR_LS - Error reading variable 'devicestate'.";
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Error reading variable 'devicestate'.");
  }
  std::string strDeviceState = StringResponse(respDeviceState);

  /*Check device state:
   * 0: Busy,
   * 1: Ready,
   * 2: Error */
  if (strDeviceState == "sRA SCdevicestate 0") {
    AWARN << "Laser scanner is busy.";
  } else if (strDeviceState == "sRA SCdevicestate 1") {
    ADEBUG << "Laser scanner is ready.";
  } else if (strDeviceState == "sRA SCdevicedstate 2") {
    AERROR << "Laser scanner error state: " << strDeviceState;
    if (mConfig.auto_reboot) {
      rebootDevice();
    }
  } else {
    ROS_WARN_STREAM(
        "Laser scanner reports unknown devicestate: " << strDeviceState);
  }

  /*Start data streaming*/
  result = SendDeviceReq(CMD_START_STREAM_DATA, NULL);
  if (0 != result) {
    AERROR << "WR_LS - Error when starting streaming 'LMDscandata'.";
    // mDiagUpdater.broadcast(
    //     diagnostic_msgs::DiagnosticStatus::ERROR,
    //     "WR_LS - Error when starting streaming 'LMDscandata'.");

    return ExitError;
  }
#endif
  return ExitSuccess;
}

std::string wr_ls::CWrLsCommon::StringResp(
    const std::vector<unsigned char> &resp) {
  std::string strResp;
  for (std::vector<unsigned char>::const_iterator it = resp.begin();
       it != resp.end(); it++) {
    if (*it > 13) {
      strResp.push_back(*it);
    }
  }

  return strResp;
}

bool wr_ls::CWrLsCommon::IsCompatibleDevice(
    const std::string strIdentify) const {
  // TODO: Always return true
  return true;
}

int CWrLsCommon::LoopOnce() {
//   mDiagUpdater.update();

  int dataLength = 0;
  static unsigned int iteration_count = 0;

  int result = GetDataGram(mRecvBuffer + mDataLength,
                           RECV_BUFFER_SIZE - mDataLength, &dataLength);
  if (0 != result) {
    AERROR << "WR_LS - Read Error when getting datagram: " << result;
    // mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
    //                        "WR_LS - Read Error when getting datagram.");

    return ExitError;
  } else {
    ADEBUG << "WR_LS - Received data gram. Data Length " << dataLength;
    mDataLength += dataLength;
  }

  if (mDataLength < FRAME_LENGTH) /*Fixed data length of 1630*/
  {
    // AWARN("WR_LS - Failed to read data gram");
    return ExitSuccess; /*return success to continue looping*/
  } else {
    ADEBUG << "WR_LS - Received one frame. Total Length " << mDataLength;
    if (mDataLength % FRAME_LENGTH != 0) {
      AERROR << "WR_LS - Invalid data length!";
    }
  }

  /*Data requested, skip frames*/
  if (iteration_count++ % (mConfig.skip + 1) != 0) {
    AINFO << "WR_LS - Skip frame";
    return ExitSuccess;
  }

  /*One full frame received. Start Data processing...*/
  if (mPublishData) {
    apollo::sensors::DataGram data_msg;
    data_msg.data = std::string(reinterpret_cast<char *>(mRecvBuffer));
    // mDataPublisher.publish(data_msg);
    datagram_writer_->Write(data_msg);
  }

  LaserScan msg;

  /*data are started with 0xaa, and data length is fixed size: 1630*/
  char *posBuffer = (char *)mRecvBuffer;
  int startIndex = 0;
  char *start = strchr(posBuffer + startIndex, STX);
  if (start == NULL) {
    AERROR << "WR_LS - Invalide data! Header NOT found!";
  }

  char *end = NULL;
  while (start != NULL && (startIndex + FRAME_LENGTH <= mDataLength)) {
    size_t length = FRAME_LENGTH;
    end = start + length; /*Fixed total length 1631*/
    *end = '\0';

    int success = mParser->Parse(start, length, mConfig, msg);
    if (ExitSuccess == success) {
      if (mConfig.debug_mode) {
        DumpLaserMessage(msg);
      }
      // mDiagPublisher->publish(msg);
    }

    posBuffer = end + 1;
    start = strchr(posBuffer, STX);
    startIndex += FRAME_LENGTH;
  }

  memset(mRecvBuffer, RECV_BUFFER_SIZE, 0);
  mDataLength = 0;

  return ExitSuccess;  // return success to continue
}

void CWrLsCommon::CheckAngleRange(wr_ls::WrLsConfig &config) {
  if (config.min_ang > config.max_ang) {
    AWARN
        << "Minimum angle must be greater than maxmum angle. Adjusting min_ang";
    config.min_ang = config.max_ang;
  }
}

void CWrLsCommon::UpdateConfig(wr_ls::WrLsConfig &newConfig, uint32_t level) {
  CheckAngleRange(newConfig);
  mConfig = newConfig;
}

void CWrLsCommon::DumpLaserMessage(LaserScan &msg) {
  ADEBUG << "Laser Message to send:";
  ADEBUG << "Header frame_id: " << msg.header.frame_id.c_str();
  ADEBUG << "Header timestamp: " << msg.header.timestamp_sec;
  ADEBUG << "angle_min: " << msg.angle_min;
  ADEBUG << "angle_max: " << msg.angle_max;
  ADEBUG << "angle_increment: " << msg.angle_increment;
  ADEBUG << "time_increment: " << msg.time_increment;
  ADEBUG << "scan_time: " << msg.scan_time;
  ADEBUG << "range_min: " << msg.range_min;
  ADEBUG << "range_max: " << msg.range_max;
}
CWrLsCommon::~CWrLsCommon() {
  // delete mDiagPublisher;
  AINFO << "wr_ls drvier exiting.";
}
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
