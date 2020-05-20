#include "modules/sensors/wr_ls_component.h"

#include <string>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace apollo {
namespace sensors {

bool WRLSComponent::Init() {

  ACHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_wr_ls_config_file,
                                                 &config_))
      << "failed to load planning config file " << FLAGS_wr_ls_config_file;

  AINFO << config_.frame_id();
  
  /*Create and initialize parser*/
  pParser = new wr_ls::CWrLs1207DEParser();

  Run();

  return true;
}

void WRLSComponent::Run() {

  int result = wr_ls::ExitError;
  if (pWrLs != nullptr) {
    delete pWrLs;
  }

  // TODO (fengzongbao) configuration
  std::string strHostName = "192.168.0.10";
  std::string strPort = "2112";

  /*Get configured time limit*/
  int iTimeLimit = 5;
  pWrLs = new wr_ls::CWrLsCommonTcp(strHostName, strPort, iTimeLimit, pParser, node_);
  result = pWrLs->Init();

  /*Device has been initliazed successfully*/
  while (!cyber::IsShutdown() && (result == wr_ls::ExitSuccess)) {
    // ros::spinOnce();
    result = pWrLs->LoopOnce();
  }

  if (result == wr_ls::ExitFatal) {
    AFATAL << "FATAL FOR SENSOR" << result;
    // return result;
  }
}

WRLSComponent::~WRLSComponent() {
  if (pWrLs != nullptr) {
    delete pWrLs;
  }

  if (pParser != nullptr) {
    delete pParser;
  }
}

}  // namespace sensors
}  // namespace apollo
