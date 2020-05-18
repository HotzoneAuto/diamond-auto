#include "modules/sensors/wr_ls_component.h"

#include <string>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

#include "modules/sensors/wr_ls/wr_ls1207de_parser.h"
#include "modules/sensors/wr_ls/wr_ls_common_tcp.h" 

namespace apollo {
namespace sensors {

bool WRLSComponent::Init() {
  InitDeviceAndSensor();
  return true;
}

void WRLSComponent::InitDeviceAndSensor() {
  # TODO configuration
  std::string strHostName = "192.168.0.10";
	std::string strPort = "2112";
  /*Get configured time limit*/
	int iTimeLimit = 5;
  /*Create and initialize parser*/
	wr_ls::CWrLs1207DEParser *pParser = new wr_ls::CWrLs1207DEParser();

  /*Setup TCP connection and attempt to connect/reconnect*/
  wr_ls::CWrLsCommon *pWrLs = NULL;
  int result = wr_ls::ExitError;
  if (pWrLs != NULL) {
    delete pWrLs;
  }

  pWrLs = new wr_ls::CWrLsCommonTcp(strHostName, strPort, iTimeLimit, pParser);
  result = pWrLs->Init();

  /*Device has been initliazed successfully*/
  while (!cyber::IsShutdown() && (result == wr_ls::ExitSuccess)) {
    ros::spinOnce();
    result = pWrLs->LoopOnce();
  }

  if (result == wr_ls::ExitFatal) {
    return result;
  }
}

WRLSComponent::~RealsenseComponent() {}

}  // namespace sensors
}  // namespace apollo
